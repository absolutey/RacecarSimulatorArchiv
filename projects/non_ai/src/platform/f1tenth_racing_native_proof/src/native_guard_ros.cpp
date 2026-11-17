// Explicitly selected, SOLO-only native ROS actuator Guard.
// No ROS/DDS credentials are inferred from topic names. Real hardware and
// H2H permission require separate qualification; defaults remain Python Guard.
#include <rclcpp/rclcpp.hpp>
#include "native_guard_intent_bounds.hpp"
#include <rclcpp/qos.hpp>
#include <control_msgs/msg/car_state.hpp>
#include <control_msgs/msg/actuator_intent.hpp>
#include <control_msgs/msg/actuator_guard_status.hpp>
#include <control_msgs/msg/guarded_actuator_command.hpp>
#include <control_msgs/msg/active_target_trajectory.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <openssl/sha.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

extern "C" {
void* f1_create_native_stop(const double*,const unsigned char*,int,int);
void f1_destroy_native_stop(void*);
int f1_step_native(void*,double*,double,double,double);
int f1_evaluate_native_stop(void*,const double*,double,double,int64_t,double*);
}
namespace {
using namespace std::chrono_literals;
using State=control_msgs::msg::CarState;
using Intent=control_msgs::msg::ActuatorIntent;
using Ref=control_msgs::msg::ActiveTargetTrajectory;
using Guarded=control_msgs::msg::GuardedActuatorCommand;
using Status=control_msgs::msg::ActuatorGuardStatus;
using ROSStamp=builtin_interfaces::msg::Time;
int64_t ns(ROSStamp t){return int64_t(t.sec)*1000000000LL+t.nanosec;}
int64_t steady_ns(){return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();}
std::string sha256(const std::string& path){
  std::ifstream f(path,std::ios::binary);if(!f)throw std::runtime_error("NATIVE_MAP_NOT_FOUND");
  SHA256_CTX c;SHA256_Init(&c);std::array<char,65536> buf{};
  while(f){f.read(buf.data(),buf.size());if(f.gcount())SHA256_Update(&c,buf.data(),f.gcount());}
  unsigned char digest[SHA256_DIGEST_LENGTH];SHA256_Final(digest,&c);
  std::ostringstream s;s<<std::hex<<std::setfill('0');for(auto v:digest)s<<std::setw(2)<<int(v);
  return s.str();
}
struct PGM{std::vector<unsigned char> data;int width=0,height=0;};
PGM pgm(const std::string& path){
  std::ifstream f(path,std::ios::binary);
  std::string tag;std::getline(f,tag);if(tag!="P5")throw std::runtime_error("NATIVE_PGM_MAGIC_INVALID");
  std::string shape;do {std::getline(f,shape);}while(!shape.empty()&&shape[0]=='#');
  PGM v;std::istringstream(shape)>>v.width>>v.height;
  std::string maxval;std::getline(f,maxval);
  if(v.width<=0||v.height<=0||int64_t(v.width)*v.height>10000000 || maxval!="255")
    throw std::runtime_error("NATIVE_PGM_HEADER_INVALID");
  v.data.resize(size_t(v.width)*v.height);
  f.read(reinterpret_cast<char*>(v.data.data()),v.data.size());
  if(!f||f.gcount()!=static_cast<std::streamsize>(v.data.size()))
    throw std::runtime_error("NATIVE_PGM_LENGTH_INVALID");
  return v;
}
std::array<double,15> physical(const State& m){
  return {m.px,m.py,m.yaw,m.v,m.vx,m.vy,m.omega,m.a,m.ax,m.ay,
          m.slip_angle,m.accel,m.jerk,m.steer,m.steer_vel};
}
bool finite(const std::array<double,15>& s){
  for(double v:s)if(!std::isfinite(v))return false;
  return true;
}
}

class NativeGuard final:public rclcpp::Node{
 public:
 explicit NativeGuard():Node("actuator_guard0"){
  epoch_=declare_parameter<std::string>("run_epoch", "");
  producer_=declare_parameter<std::string>("producer", "R_B");
  track_=declare_parameter<std::string>("track_id", "");
  map_hash_=declare_parameter<std::string>("map_sha256", "");
  const auto path=declare_parameter<std::string>("map_file", "");
  const auto parameters=declare_parameter<std::vector<double>>("native_config",std::vector<double>{});
  auto solo=declare_parameter<bool>("solo",true);
  const auto max_speed=declare_parameter<double>("max_speed_cap_mps",0.);
  if(!solo||epoch_.empty()||track_.empty()||map_hash_.size()!=64||path.empty()||
     (producer_!="R_B" && producer_!="FOUNDATION_BASELINE")||
     parameters.size()!=36||!std::isfinite(max_speed)||max_speed<=0.0||
     !std::isfinite(parameters[12])||max_speed>parameters[12])
    throw std::runtime_error("NATIVE_GUARD_INCOMPLETE_TRUST_CONFIGURATION");
  // Simulator limits are the only actuator envelope; no inherited 5 m/s cap.
  if(parameters[10]<=0.0 || parameters[13]<=0.0 || parameters[14]<=0.0 ||
     parameters[11]<=0.0 || parameters[15]<=0.0)
    throw std::runtime_error("NATIVE_GUARD_INVALID_PHYSICAL_ACTUATOR_LIMITS");
  steering_limit_=parameters[10];
  acceleration_limit_=parameters[13];
  braking_limit_=parameters[14];
  speed_cap_=max_speed;
  if(sha256(path)!=map_hash_)throw std::runtime_error("NATIVE_GUARD_MAP_SHA_MISMATCH");
  auto map=pgm(path);
  for(double p:parameters)if(!std::isfinite(p))
    throw std::runtime_error("NATIVE_GUARD_NONFINITE_PARAMETER");
  native_=f1_create_native_stop(parameters.data(),map.data.data(),map.width,map.height);
  if(!native_)throw std::runtime_error("NATIVE_GUARD_ENGINE_CONSTRUCTION_FAILED");
  const auto reliable=rclcpp::QoS(rclcpp::KeepLast(16)).reliable();
  const auto sensor=rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
  const auto command=rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
  const auto latched=rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  guarded_=create_publisher<Guarded>("guarded_actuator_cmd0",command);
  telemetry_=create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("ackermann_cmd0",command);
  status_=create_publisher<Status>("actuator_guard_status0",latched);
  on_state_=create_subscription<State>("state0",reliable,[this](const State::SharedPtr m){on_state(*m);});
  on_intent_=create_subscription<Intent>("actuator_intent0",command,[this](const Intent::SharedPtr m){on_intent(*m);});
  on_ref_=create_subscription<Ref>("active_target_trajectory",latched,[this](const Ref::SharedPtr m){on_ref(*m);});
  on_start_=create_subscription<ROSStamp>("race_start_time",latched,[this](const ROSStamp::SharedPtr m){start_ns_=ns(*m);});
  on_collision_=create_subscription<std_msgs::msg::Bool>("collision0",sensor,[this](const std_msgs::msg::Bool::SharedPtr m){if(m->data)brake("COLLISION_LATCHED",true);});
  on_vehicle_=create_subscription<std_msgs::msg::Bool>("vehicle_collision",sensor,[this](const std_msgs::msg::Bool::SharedPtr m){if(m->data)brake("COLLISION_LATCHED",true);});
  timer_=create_wall_timer(25ms,[this]{watchdog();});
 }
 ~NativeGuard()override{if(native_)f1_destroy_native_stop(native_);}
 private:
 int64_t clock_ns(){return get_clock()->now().nanoseconds();}
 bool recent_state(){
   auto now=clock_ns(),st=steady_ns();
   return has_state_ && st>=rx_steady_ && st-rx_steady_<=75000000 &&
          now>=ns(state_.physics_stamp) && now-ns(state_.physics_stamp)<=75000000 &&
          finite(physical(state_));
 }
 void on_state(const State& m){
   int64_t st=ns(m.physics_stamp),now=clock_ns();
   if(has_state_ && (m.physics_sequence<=state_.physics_sequence||st<=ns(state_.physics_stamp))){
     brake("STATE_SEQUENCE_OR_TIME_REGRESSION",true);return;
   }
   if(!finite(physical(m))||st>now||now-st>75000000){
     brake("STATE_INVALID_OR_STALE",true);return;
   }
   state_=m;has_state_=true;rx_steady_=steady_ns();
 }
 void on_ref(const Ref& r){
   if(!r.active_reference_valid||r.track_id!=track_||r.map_image_sha256!=map_hash_){
     if(running_)brake("NATIVE_ACTIVE_REFERENCE_WITHDRAWN",true);
     ref_id_.clear();revision_=0;running_=false;return;
   }
   if(ref_id_!=r.active_reference_id||revision_!=r.active_revision){
     if(running_){brake("NATIVE_REFERENCE_REVISION_CHANGED",true);return;}
     ref_id_=r.active_reference_id;revision_=r.active_revision;
   }
 }
 void brake(const std::string& reason,bool permanent){
   reason_=reason;running_=false;if(permanent)latched_fault_=true;
   // A stale or non-finite CarState CANNOT issue a fresh braking lease.
   // The simulator's independent deadman applies on lost state/clock.
   if(!recent_state())return;
   auto v=state_.v;
   Guarded cmd;
   cmd.stamp=static_cast<ROSStamp>(get_clock()->now());
   int64_t expiry=ns(cmd.stamp)+75000000;
   cmd.expires_ros_stamp.sec=int32_t(expiry/1000000000LL);
   cmd.expires_ros_stamp.nanosec=uint32_t(expiry%1000000000LL);
   cmd.run_epoch=epoch_;cmd.source_id=producer_;cmd.intent_sequence=0;
   cmd.source_state_sequence=state_.physics_sequence;
   cmd.active_reference_id=ref_id_;cmd.active_revision=revision_;
   cmd.track_id=track_;cmd.map_sha256=map_hash_;
   cmd.authority_mode="MINIMUM_RISK_BRAKE";
   cmd.stop_certificate_verified=false;
   cmd.steering_rad=0.;
   cmd.acceleration_mps2=(v>0.025 ? -2.0 : v< -0.025 ? 2.0 : 0.0);
   guarded_->publish(cmd);
 }
 void on_intent(const Intent& m){
   const int64_t now=clock_ns(),steady=steady_ns();
   if(latched_fault_||start_ns_<=0||now<start_ns_||ref_id_.empty()||!recent_state())return;
   const auto created=ns(m.created_ros_stamp),expires=ns(m.expires_ros_stamp);
   const auto exact=ns(m.source_state_stamp)==ns(state_.physics_stamp)&&
                    m.source_state_sequence==state_.physics_sequence;
   // A certificate must not authorize an unbounded command or a forged
   // hours-long lease. Do NOT rely on the simulator to reject this after
   // the C++ Guard has already marked the intent CERTIFIED.
   if(!exact || !native_guard_intent_bounds::admissible(
       m.steering_rad,m.acceleration_mps2,created,expires,now,
       steering_limit_,acceleration_limit_,braking_limit_) ||
      m.sequence<=last_seq_||m.run_epoch!=epoch_||m.source_id!=producer_||
      m.track_id!=track_||m.map_sha256!=map_hash_||
      m.active_reference_id!=ref_id_||m.active_revision!=revision_||
      !std::isfinite(m.steering_rad)||!std::isfinite(m.acceleration_mps2)||
      std::abs(state_.v)>speed_cap_+0.05){
     brake("NATIVE_INTENT_IDENTITY_OR_LEASE_REJECTED",true);return;
   }
   std::array<double,15> z=physical(state_);
   const auto begun=steady_ns(),deadline=begun+15000000;
   const int step=f1_step_native(native_,z.data(),m.steering_rad,m.acceleration_mps2,.025);
   double proof[8]={};
   const int result=step==0?f1_evaluate_native_stop(native_,z.data(),
       m.steering_rad,m.acceleration_mps2,deadline,proof):-1;
   if(step!=0||result!=0||proof[0]!=1.0||proof[1]!=0.0||
      steady_ns()>=deadline||clock_ns()>expires||!recent_state()){
     brake("NATIVE_STOP_CERTIFICATE_UNPROVEN",true);return;
   }
   // Only this Guard publishes simulator-authoritative commands. Source state,
   // command and proof are bound to a SINGLE live physics tick. ROS callback
   // serialization prevents state update between the final check and publish.
   Guarded output;
   output.stamp=static_cast<ROSStamp>(get_clock()->now());output.expires_ros_stamp=m.expires_ros_stamp;
   output.run_epoch=epoch_;output.source_id=producer_;output.intent_sequence=m.sequence;
   output.source_state_sequence=m.source_state_sequence;
   output.active_reference_id=ref_id_;output.active_revision=revision_;
   output.track_id=track_;output.map_sha256=map_hash_;
   output.authority_mode="CERTIFIED";output.stop_certificate_verified=true;
   output.steering_rad=m.steering_rad;output.acceleration_mps2=m.acceleration_mps2;
   if(clock_ns()>expires||steady_ns()>deadline){brake("NATIVE_PRE_PUBLISH_LEASE_EXPIRED",true);return;}
   guarded_->publish(output);
   ackermann_msgs::msg::AckermannDriveStamped ack;
   ack.header.stamp=output.stamp;
   ack.drive.steering_angle=m.steering_rad;ack.drive.acceleration=m.acceleration_mps2;
   telemetry_->publish(ack);
   running_=true;last_seq_=m.sequence;last_intent_steady_=steady;
   reason_="NATIVE_CERTIFIED_STOP";
 }
 void watchdog(){
   const auto now=clock_ns(),st=steady_ns();
   // The watchdog uses WALL time, while /clock advances on simulator
   // physics ticks. Equal adjacent samples are normal (timer jitter): they
   // cannot instantly create an irreversible collision-style fault. A true
   // regression is immediate; stagnant ROS time beyond the bounded steady
   // lease fails closed. The simulator has its OWN independent deadman.
   if(now<last_ros_ns_ && last_ros_ns_>0){
     brake("NATIVE_ROS_CLOCK_REGRESSION",true);
   }else if(now>last_ros_ns_){
     last_clock_progress_steady_ns_=st;
   }else if(last_ros_ns_>0 && last_clock_progress_steady_ns_>0 &&
            st-last_clock_progress_steady_ns_>75000000){
     brake("NATIVE_ROS_CLOCK_STALLED",true);
   }
   if(start_ns_>0&&now>=start_ns_&&!recent_state()){
     brake("NATIVE_STATE_WATCHDOG",true);
   }
   else if(running_&&(st-last_intent_steady_>100000000)){brake("NATIVE_INTENT_WATCHDOG",true);}
   last_ros_ns_=now;
   Status s;
   s.stamp=static_cast<ROSStamp>(get_clock()->now());s.run_epoch=epoch_;
   s.mode=latched_fault_?"SAFE_HOLD":running_?"RUNNING":"ARMED";
   s.reason=reason_;s.source_id=producer_;s.active_reference_id=ref_id_;
   s.last_accepted_sequence=last_seq_;s.ready=!latched_fault_&&recent_state()&&!ref_id_.empty();
   s.certificate_verified=running_&&!latched_fault_;
   s.emergency_unrecoverable=latched_fault_;
   status_->publish(s);
 }
 void* native_=nullptr;
 std::string epoch_,producer_,track_,map_hash_,ref_id_,reason_="NATIVE_PRESTART";
 double speed_cap_=0.,steering_limit_=0.,acceleration_limit_=0.,braking_limit_=0.;
 uint32_t revision_=0;
 State state_{};bool has_state_=false,running_=false,latched_fault_=false;
 uint64_t last_seq_=0;
 int64_t rx_steady_=0,start_ns_=0,last_ros_ns_=0,last_intent_steady_=0;
 int64_t last_clock_progress_steady_ns_=0;
 rclcpp::Publisher<Guarded>::SharedPtr guarded_;
 rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr telemetry_;
 rclcpp::Publisher<Status>::SharedPtr status_;
 rclcpp::Subscription<State>::SharedPtr on_state_;
 rclcpp::Subscription<Intent>::SharedPtr on_intent_;
 rclcpp::Subscription<Ref>::SharedPtr on_ref_;
 rclcpp::Subscription<ROSStamp>::SharedPtr on_start_;
 rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr on_collision_,on_vehicle_;
 rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char**argv){
 rclcpp::init(argc,argv);
 try{
  auto guard=std::make_shared<NativeGuard>();rclcpp::spin(guard);
 }catch(const std::exception&ex){
  fprintf(stderr,"NATIVE_GUARD_FATAL:%s\n",ex.what());rclcpp::shutdown();return 2;
 }
 rclcpp::shutdown();return 0;
}
