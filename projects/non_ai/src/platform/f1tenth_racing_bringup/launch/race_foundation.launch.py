from pathlib import Path
import hashlib
import uuid

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (AndSubstitution, LaunchConfiguration,
                                  NotSubstitution, OrSubstitution)
from launch_ros.actions import Node
import xacro


def description_node(car_id, environment, xacro_path, condition=None):
    vehicle = environment["vehicle"]["description"]
    mappings = {
        "suffix": str(car_id),
        "chassis_length": str(vehicle["chassis_length_m"]),
        "chassis_width": str(vehicle["chassis_width_m"]),
        "chassis_height": str(vehicle["chassis_height_m"]),
        "chassis_z": str(vehicle["chassis_ground_offset_m"]),
        "wheel_radius": str(vehicle["wheel_radius_m"]),
        "wheel_length": str(vehicle["wheel_length_m"]),
        "wheel_x": str(vehicle["wheel_center_x_m"]),
        "wheel_y": str(vehicle["wheel_center_y_m"]),
        "lidar_x": str(vehicle["lidar_x_m"]),
        "lidar_y": str(vehicle["lidar_y_m"]),
        "lidar_z": str(vehicle["lidar_z_m"]),
        "lidar_radius": str(vehicle["lidar_visual_radius_m"]),
        "lidar_height": str(vehicle["lidar_visual_height_m"]),
        "color": " ".join(map(str, vehicle[f"car{car_id}_color_rgba"])),
    }
    return Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=f"car{car_id}",
        name="robot_state_publisher",
        parameters=[{"robot_description": xacro.process_file(str(xacro_path), mappings=mappings).toxml()}],
        condition=condition,
    )


def validate_execution_profile(*, profile, solo, use_r_b0, controller_kind,
                               trajectory_candidate, use_opponent_perception,
                               use_opponent_prediction, use_overtaking):
    """Explicit race-mode intent; no accidental unqualified H2H startup.

    This is *configuration* admission, NOT dynamic safety/trajectory
    qualification. The independent Guard alone retains actuation authority.
    """
    choices = {
        "SOLO_BASELINE": ("pursuit", "baseline"),
        "SOLO_SCP_EXPERIMENTAL": ("pursuit", "scp"),
        "SOLO_MPC_EXPERIMENTAL": ("mpc", "scp"),
        "H2H_MPC_DYNAMIC_EXPERIMENTAL": ("mpc", "scp"),
        "SOLO_SPIELBERG_MPC_PRELIMINARY": ("mpc", "baseline"),
        "SOLO_SPIELBERG_MPC_SCP_EXPERIMENTAL": ("mpc", "scp"),
        "H2H_SPIELBERG_MPC_SCP_EXPERIMENTAL": ("mpc", "scp"),
        "H2H_MPC_BASELINE_PRELIMINARY": ("mpc", "baseline"),
    }
    if not use_r_b0:
        raise RuntimeError("PROFILE_REQUIRES_RB_AND_INDEPENDENT_GUARD")
    # Development inspection is NOT a driving/H2H qualification.
    # All planning outputs remain observable, but S0-R must not adopt R-E.
    if profile == "DEV_COMPONENTS_OBSERVE":
        if controller_kind not in ("pursuit", "mpc"):
            raise RuntimeError("DEV_INVALID_RB_CONTROLLER")
        if trajectory_candidate not in ("baseline", "scp"):
            raise RuntimeError("DEV_INVALID_RA_CANDIDATE")
        if solo and (use_opponent_perception or use_opponent_prediction or use_overtaking):
            raise RuntimeError("DEV_RED_PIPELINE_REQUIRES_RED_SIM")
        if use_opponent_prediction and not use_opponent_perception:
            raise RuntimeError("DEV_PREDICTION_REQUIRES_PERCEPTION")
        if use_overtaking and not (use_opponent_perception and use_opponent_prediction):
            raise RuntimeError("DEV_OVERTAKING_REQUIRES_PERCEPTION_AND_PREDICTION")
        return profile
    if solo:
        if profile.startswith("H2H_"):
            raise RuntimeError("H2H_PROFILE_REQUIRES_RED_CAR")
        if use_opponent_perception or use_opponent_prediction or use_overtaking:
            raise RuntimeError("SOLO_PROFILE_MUST_DISABLE_H2H_PIPELINE")
    else:
        if profile not in ("H2H_MPC_DYNAMIC_EXPERIMENTAL","H2H_MPC_BASELINE_PRELIMINARY",
                            "H2H_SPIELBERG_MPC_SCP_EXPERIMENTAL"):
            raise RuntimeError("H2H_PROFILE_REQUIRED")
        if not (use_opponent_perception and use_opponent_prediction and use_overtaking):
            raise RuntimeError("H2H_REQUIRES_PERCEPTION_PREDICTION_OVERTAKING")
    if profile not in choices:
        raise RuntimeError("UNKNOWN_EXECUTION_PROFILE:" + str(profile))
    expected = choices[profile]
    if (controller_kind, trajectory_candidate) != expected:
        raise RuntimeError("EXECUTION_PROFILE_PARAMETER_MISMATCH:" + profile)
    return profile


def _instantiate_nodes(context):
    def flag(arg):
        return LaunchConfiguration(arg).perform(context).lower() in ("1", "true")
    validate_execution_profile(
        profile=LaunchConfiguration("execution_profile").perform(context),
        solo=flag("solo"), use_r_b0=flag("use_r_b0"),
        controller_kind=LaunchConfiguration("controller_kind").perform(context),
        trajectory_candidate=LaunchConfiguration("trajectory_candidate").perform(context),
        use_opponent_perception=flag("use_opponent_perception"),
        use_opponent_prediction=flag("use_opponent_prediction"),
        use_overtaking=flag("use_overtaking"),
    )
    dev_observe = LaunchConfiguration("execution_profile").perform(context) == "DEV_COMPONENTS_OBSERVE"
    if dev_observe and flag("use_native_guard_ros"):
        raise RuntimeError("DEV_REQUIRES_PYTHON_GUARD")
    if not flag("solo") and not flag("dynamic_red_guard_enabled"):
        raise RuntimeError("H2H_GUARD_RED_PROOF_REQUIRED")
    if flag("use_native_guard_ros") and not flag("solo"):
        raise RuntimeError("NATIVE_CPP_GUARD_H2H_NOT_QUALIFIED")
    sim_share = Path(get_package_share_directory("f1tenth_racing_sim"))
    bringup_share = Path(get_package_share_directory("f1tenth_racing_bringup"))
    control_share = Path(get_package_share_directory("f1tenth_racing_control"))
    reference_share = Path(get_package_share_directory("f1tenth_racing_reference"))
    description_share = Path(get_package_share_directory("f1tenth_racing_description"))
    environment_file = LaunchConfiguration("environment_file").perform(context)
    environment_file = str(Path(environment_file).resolve(strict=True))
    environment = yaml.safe_load(Path(environment_file).read_text())
    if not isinstance(environment,dict): raise RuntimeError("invalid selected environment")
    foundation=yaml.safe_load((bringup_share / "config/foundation.yaml").read_text())
    mpc=yaml.safe_load((control_share / "config/r_b3_mpc.yaml").read_text())
    mpc_formulation = LaunchConfiguration('mpc_formulation').perform(context)
    if mpc_formulation not in ('ltv','nmpc'):
        raise RuntimeError('MPC_FORMULATION_NOT_SUPPORTED')
    profile=foundation["race_profile"]
    xacro_path=description_share / "urdf/vehicle.urdf.xacro"
    guard_epoch=str(uuid.uuid4())
    # AUTO means the simulator's declared physical top speed, NOT a 5m/s
    # inherited qualification cap. Every requested actuation still faces Guard.
    def physical_cap(name):
        requested=LaunchConfiguration(name).perform(context)
        physics_max=float(environment['dynamics']['limits']['speed_abs_max_m_s'])
        value=physics_max if requested.upper()=='AUTO' else float(requested)
        if not (0.0<value<=physics_max):
            raise RuntimeError(name+': requested speed exceeds environment physics bound')
        return value
    speed_cap=physical_cap('speed_cap_mps')
    fallback_speed_cap=physical_cap('fallback_speed_cap_mps')
    interaction_speed_cap=physical_cap('interaction_speed_cap_mps')
    use_r_b0=LaunchConfiguration("use_r_b0").perform(context).lower() in ("true", "1")
    guard_producer="R_B" if use_r_b0 else "FOUNDATION_BASELINE"
    expected_track_id=LaunchConfiguration("expected_track_id").perform(context) or environment["environment_identity"]["active_track"]
    expected_map_sha256=LaunchConfiguration("expected_map_sha256").perform(context) or environment["map"]["image_sha256"]
    supplied_policy=LaunchConfiguration("nominal_policy_file").perform(context)
    nominal_policy=(supplied_policy if supplied_policy else str(reference_share / 'config' /
        ('spielberg_nominal_reference.json' if expected_track_id=='spielberg' else
         'nominal_reference.json')))
    if not Path(nominal_policy).is_file():
        raise RuntimeError('TRACK_NOMINAL_POLICY_MISSING:'+expected_track_id)
    with Path(nominal_policy).open() as source:
        policy=yaml.safe_load(source)
    if (policy.get('track_id')!=expected_track_id or
            policy.get('map_image_sha256')!=expected_map_sha256):
        raise RuntimeError('NOMINAL_POLICY_MAP_IDENTITY_MISMATCH')
    selected_candidate=LaunchConfiguration('trajectory_candidate').perform(context)
    if selected_candidate not in policy.get('candidates',{}):
        raise RuntimeError('TRACK_TRAJECTORY_CANDIDATE_MISSING:'+expected_track_id+':'+selected_candidate)
    active_profile=LaunchConfiguration('execution_profile').perform(context)
    if ('SPIELBERG' in active_profile and expected_track_id!='spielberg') or (            active_profile in ('H2H_MPC_BASELINE_PRELIMINARY','H2H_SPIELBERG_MPC_SCP_EXPERIMENTAL') and expected_track_id!='spielberg'):
        raise RuntimeError('EXECUTION_PROFILE_TRACK_MISMATCH')
    if (expected_track_id != environment["environment_identity"]["active_track"] or
            expected_map_sha256 != environment["map"]["image_sha256"]):
        raise RuntimeError("launch environment and selected track/map mismatch")
    map_file=LaunchConfiguration("map_file").perform(context) or str(sim_share / environment["map"]["image_file"])
    centerline_file=LaunchConfiguration("centerline_file").perform(context) or str(sim_share / environment["map"]["centerline_file"])
    for fname, declared in ((map_file, environment["map"]["image_sha256"]),
                            (centerline_file, environment["map"]["centerline_sha256"])):
        if not Path(fname).is_file() or hashlib.sha256(Path(fname).read_bytes()).hexdigest() != declared:
            raise RuntimeError("selected track asset missing or SHA mismatch: " + str(fname))
    native_guard_params=None
    if flag("use_native_guard_ros"):
        import importlib.util
        helper=Path(get_package_share_directory('f1tenth_racing_native_proof')) / 'CREATE_NATIVE_GUARD_LAUNCH_PARAMS.py'
        spec=importlib.util.spec_from_file_location('f1_native_guard_parameters',helper)
        module=importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        from ament_index_python.packages import get_package_prefix
        from f1tenth_racing_sim.safety.native_stop_proof import check_experimental_parity_receipt
        lib=Path(get_package_prefix('f1tenth_racing_native_proof'))/'lib/libf1tenth_native_stop.so'
        receipt=LaunchConfiguration('native_guard_receipt').perform(context)
        if not receipt:
            raise RuntimeError('NATIVE_CPP_GUARD_INSTALLED_PARITY_RECEIPT_REQUIRED')
        check_experimental_parity_receipt(str(lib),receipt,
            expected_map_sha256=expected_map_sha256,active_map_file=map_file)
        native_guard_params=module.build_native_guard_parameters(
            environment,map_file,epoch=guard_epoch,producer=guard_producer,
            speed_cap=speed_cap)
    return [
        Node(package="f1tenth_racing_sim", executable="simulator", name="competition_simulator0", output="screen", parameters=[{"car_id": 0, "guard_run_epoch": guard_epoch, "guard_producer": guard_producer, "independent_watchdog_required": True, "initial_track_s_m": profile["blue_spawn_s_m"], "dynamic_opponent_id": 1, "publish_vehicle_collision": True, "publish_environment": True, "command_timeout_s": foundation["runtime_safety"]["command_timeout_s"],
            "config_file": environment_file,
            "map_file": map_file,
            "centerline_file": centerline_file}]),
        Node(package="f1tenth_racing_sim", executable="simulator", name="competition_simulator1", output="screen", parameters=[{"car_id": 1, "initial_track_s_m": profile["red_spawn_s_m"], "dynamic_opponent_id": 0, "publish_vehicle_collision": False, "publish_environment": False, "publish_lidar": LaunchConfiguration("publish_red_lidar"), "command_timeout_s": foundation["runtime_safety"]["command_timeout_s"],
            "config_file": environment_file,
            "map_file": map_file,
            "centerline_file": centerline_file}], condition=UnlessCondition(LaunchConfiguration("solo"))),
        description_node(0, environment, xacro_path),
        description_node(1, environment, xacro_path, UnlessCondition(LaunchConfiguration("solo"))),
        Node(package="f1tenth_racing_control", executable="actuator_guard",
             name="actuator_guard0", output="screen", parameters=[{
                 "environment_file": environment_file,
                 "map_file": map_file,
                 "track_id": expected_track_id,
                 "map_sha256": expected_map_sha256,
                 "run_epoch": guard_epoch,
                 "producer": guard_producer,
                 "solo": LaunchConfiguration("solo"),
                 "max_speed_cap_mps": speed_cap,
                 "dynamic_red_guard_enabled": LaunchConfiguration("dynamic_red_guard_enabled"),
                 "sim_online_adaptive": LaunchConfiguration("sim_online_adaptive"),
                 "experimental_native_guard": LaunchConfiguration("experimental_native_guard"),
                 "native_guard_library": LaunchConfiguration("native_guard_library"),
                 "native_guard_receipt": LaunchConfiguration("native_guard_receipt"),
                 "native_dual_veto_required": LaunchConfiguration("native_dual_veto_required"),
             }],condition=UnlessCondition(LaunchConfiguration("use_native_guard_ros"))),
        Node(package="f1tenth_racing_native_proof", executable="native_actuator_guard",
             name="actuator_guard0",output="screen",
             parameters=[native_guard_params] if native_guard_params else [],
             condition=IfCondition(LaunchConfiguration("use_native_guard_ros"))),
        # Separate process, separate executor: absence/stall of this veto
        # forces simulator-local deadman even if an old Guard command survives.
        Node(package="f1tenth_racing_control", executable="independent_guard_watchdog",
             name="independent_guard_watchdog0", output="screen", parameters=[{
                 "run_epoch": guard_epoch,
                 "producer": guard_producer,
             }]),
        Node(package="f1tenth_racing_evaluator", executable="evaluator", output="screen", parameters=[{"centerline_file": centerline_file}]),
        Node(package="f1tenth_racing_bringup", executable="race_coordinator", output="screen", parameters=[{"require_h2h_ready": (not dev_observe and flag("use_overtaking")),"require_actuator_guard": True,"expected_guard_source":guard_producer,"expected_guard_epoch":guard_epoch}]),
        Node(package="foundation_baseline", executable="blue_foundation_controller", output="screen", parameters=[{"target_speed_m_s": foundation["blue_foundation"]["target_speed_m_s"],"environment_file":environment_file,"centerline_file":centerline_file,"guard_run_epoch":guard_epoch}], condition=UnlessCondition(LaunchConfiguration("use_r_b0"))),
        Node(package="f1tenth_racing_reference", executable="nominal_active_reference", output="screen", parameters=[{"trajectory_candidate": LaunchConfiguration("trajectory_candidate"),
            "nominal_policy_file": nominal_policy,
            "environment_file": environment_file,
            "allow_preliminary_spielberg": active_profile in (
                'SOLO_SPIELBERG_MPC_PRELIMINARY', 'H2H_MPC_BASELINE_PRELIMINARY'),
            "raceline_data_dir": LaunchConfiguration("raceline_data_dir"),
            "expected_track_id": expected_track_id,
            "expected_map_sha256": expected_map_sha256, "accept_overtaking_plans": not dev_observe}], condition=IfCondition(LaunchConfiguration("use_r_b0"))),
        Node(package="f1tenth_racing_control", executable="r_b0_controller", output="screen", parameters=[{"qualified_speed_cap_mps": speed_cap, "fallback_speed_cap_mps": fallback_speed_cap,
            "sim_online_adaptive": LaunchConfiguration("sim_online_adaptive"), "controller_kind": LaunchConfiguration("controller_kind"), "controller_profile": LaunchConfiguration("controller_profile"), "mpc_formulation": LaunchConfiguration("mpc_formulation"), "diagnostic_withhold_primary": LaunchConfiguration("diagnostic_withhold_primary"),
            "experimental_native_guardahead": LaunchConfiguration("experimental_native_guardahead"),
            "native_guardahead_library": LaunchConfiguration("native_guardahead_library"),
            "native_guardahead_receipt": LaunchConfiguration("native_guardahead_receipt"),
            "primary_process_enabled": LaunchConfiguration("primary_process_enabled"),
            "expected_track_id": expected_track_id,
            "expected_map_sha256": expected_map_sha256,
            "high_speed_contract_file": LaunchConfiguration("high_speed_contract_file"),
            "high_speed_trajectory_id": LaunchConfiguration("high_speed_trajectory_id"),
            "high_speed_trusted_public_key_file": LaunchConfiguration("high_speed_trusted_public_key_file"),
            "high_speed_trusted_public_key_sha256": LaunchConfiguration("high_speed_trusted_public_key_sha256"),
            "environment_file": environment_file,
            "centerline_file": centerline_file,"guard_run_epoch":guard_epoch}], condition=IfCondition(LaunchConfiguration("use_r_b0"))),
        Node(package="f1tenth_racing_perception", executable="opponent_perception", output="screen", parameters=[{"selected_tracker": LaunchConfiguration("opponent_tracker"), "environment_file": environment_file, "expected_track_id": expected_track_id, "expected_map_sha256": expected_map_sha256}], condition=IfCondition(LaunchConfiguration("use_opponent_perception"))),
        Node(package="f1tenth_racing_prediction", executable="opponent_prediction", output="screen", parameters=[{
            "selected_predictor": LaunchConfiguration("opponent_predictor"),
            "horizon_s": LaunchConfiguration("opponent_prediction_horizon_s"),
            "timestep_s": LaunchConfiguration("opponent_prediction_timestep_s"),
            "environment_file": environment_file, "centerline_file": centerline_file,
            "expected_track_id":expected_track_id,"expected_map_sha256":expected_map_sha256,
        }], condition=IfCondition(LaunchConfiguration("use_opponent_prediction"))),
        Node(package="f1tenth_racing_overtaking", executable=LaunchConfiguration("overtaking_executable"), output="screen", parameters=[{"qualified_speed_cap_mps": interaction_speed_cap,"environment_file":environment_file,"centerline_file":centerline_file,"expected_track_id":expected_track_id,"expected_map_sha256":expected_map_sha256}], condition=IfCondition(LaunchConfiguration("use_overtaking"))),
        Node(package="f1tenth_racing_opponent", executable="red_baseline_controller", output="screen", parameters=[{"target_speed_m_s": foundation["opponent"]["target_speed_m_s"],"environment_file":environment_file,"centerline_file":centerline_file}], condition=UnlessCondition(LaunchConfiguration("solo"))),
        Node(package="f1tenth_racing_telemetry", executable="flight_recorder", output="screen", parameters=[{
            "run_root": LaunchConfiguration("telemetry_run_root"),
            "run_id": LaunchConfiguration("telemetry_run_id"),
            "trajectory_candidate": LaunchConfiguration("trajectory_candidate"),
            "nominal_policy_file": nominal_policy,
            "raceline_data_dir": LaunchConfiguration("raceline_data_dir"),
            "environment_file": environment_file,
            "expected_track_id": expected_track_id,
            "expected_map_sha256": expected_map_sha256,
            "record_rosbag": LaunchConfiguration("record_rosbag"),
            "review_phase": LaunchConfiguration("review_phase"),
            "execution_speed_cap_mps": speed_cap,
            "fallback_speed_cap_mps": fallback_speed_cap,
            "controller_kind": LaunchConfiguration("controller_kind"),
            "controller_profile": LaunchConfiguration("controller_profile"),
            "candidate_id": LaunchConfiguration("candidate_id"),
            "controller_config_sha256": hashlib.sha256(
                (control_share / "config/r_b3_mpc.yaml").read_bytes()
            ).hexdigest(),
            "solver_formulation_id": (
                'NMPC_NONLINEAR_DYNAMICS_SLSQP_V1' if mpc_formulation=='nmpc'
                else 'LTV_PACEJKA_ADMM_V2'),
            "solver_horizon_steps": mpc["horizon_steps"],
            "solver_timestep_s": mpc["timestep_s"],
            "solver_deadline_s": mpc["primary_deadline_s"],
        }], condition=IfCondition(OrSubstitution(
            LaunchConfiguration("use_flight_recorder"),
            LaunchConfiguration("use_spectator_hud"),
        ))),
        Node(package="f1tenth_racing_telemetry", executable="spectator_hud", output="screen",
             parameters=[{
                 "review_phase": LaunchConfiguration("review_phase"),
                 "solo": LaunchConfiguration("solo"),
                 "execution_speed_cap_mps": speed_cap,
                 "trajectory_candidate": LaunchConfiguration("trajectory_candidate"),
                 "nominal_policy_file": nominal_policy,
                 "raceline_data_dir": LaunchConfiguration("raceline_data_dir"),
                 "expected_track_id": expected_track_id,
                 "expected_map_sha256": expected_map_sha256,
             }],
             condition=IfCondition(LaunchConfiguration("use_spectator_hud"))),
        Node(package="rviz2", executable="rviz2", arguments=["-d", str(bringup_share / "rviz/foundation.rviz")],
             condition=IfCondition(AndSubstitution(LaunchConfiguration("use_rviz"), NotSubstitution(LaunchConfiguration("use_spectator_hud")))), output="screen"),
        Node(package="rviz2", executable="rviz2", arguments=["-d", str(bringup_share / "rviz/h2h_spectator.rviz")],
             condition=IfCondition(AndSubstitution(LaunchConfiguration("use_rviz"), LaunchConfiguration("use_spectator_hud"))), output="screen"),
    ]

def generate_launch_description():
    sim_share=Path(get_package_share_directory("f1tenth_racing_sim"))
    return LaunchDescription([
        DeclareLaunchArgument("map_file", default_value=""),

        DeclareLaunchArgument("centerline_file", default_value=""),

        DeclareLaunchArgument("environment_file", default_value=str(sim_share / "config/competition_environment.yaml")),
        DeclareLaunchArgument("nominal_policy_file", default_value=""),
        DeclareLaunchArgument("raceline_data_dir", default_value=""),
        DeclareLaunchArgument("expected_track_id", default_value=""),

        DeclareLaunchArgument("expected_map_sha256", default_value=""),

        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_r_b0", default_value="true"),
        DeclareLaunchArgument("solo", default_value="true"),
        DeclareLaunchArgument("speed_cap_mps", default_value="AUTO"),
        DeclareLaunchArgument("fallback_speed_cap_mps", default_value="AUTO"),
        DeclareLaunchArgument("sim_online_adaptive", default_value="true"),
        # Never enable the unqualified native R-B preflight merely by launch.
        DeclareLaunchArgument("experimental_native_guard", default_value="false"),
        DeclareLaunchArgument("native_guard_library", default_value=""),
        DeclareLaunchArgument("native_guard_receipt", default_value=""),
        DeclareLaunchArgument("native_dual_veto_required", default_value="false"),
        DeclareLaunchArgument("use_native_guard_ros", default_value="false"),
        DeclareLaunchArgument("primary_process_enabled", default_value="false"),
        DeclareLaunchArgument("experimental_native_guardahead", default_value="false"),
        DeclareLaunchArgument("native_guardahead_library", default_value=""),
        DeclareLaunchArgument("native_guardahead_receipt", default_value=""),
        DeclareLaunchArgument("interaction_speed_cap_mps", default_value="AUTO"),
        DeclareLaunchArgument("dynamic_red_guard_enabled", default_value="false"),
        DeclareLaunchArgument("trajectory_candidate", default_value="scp"),
        # DEV_COMPONENTS_OBSERVE: R-C/R-D/R-E observation without S0-R override.
        # Guard and independent watchdog remain mandatory.
        DeclareLaunchArgument("execution_profile", default_value="SOLO_MPC_EXPERIMENTAL"),
        DeclareLaunchArgument("controller_kind", default_value="mpc"),
        DeclareLaunchArgument("controller_profile", default_value="default"),
        DeclareLaunchArgument("mpc_formulation", default_value="ltv"),
        DeclareLaunchArgument("high_speed_contract_file", default_value=""),
        DeclareLaunchArgument("high_speed_trajectory_id", default_value=""),
        DeclareLaunchArgument("high_speed_trusted_public_key_file", default_value=""),
        DeclareLaunchArgument("high_speed_trusted_public_key_sha256", default_value=""),
        DeclareLaunchArgument("diagnostic_withhold_primary", default_value="false"),
        DeclareLaunchArgument("candidate_id", default_value="B0"),
        DeclareLaunchArgument("use_opponent_perception", default_value="false"),
        DeclareLaunchArgument("opponent_tracker", default_value="cv"),
        DeclareLaunchArgument("use_opponent_prediction", default_value="false"),
        DeclareLaunchArgument("opponent_predictor", default_value="track_cv"),
        DeclareLaunchArgument("opponent_prediction_horizon_s", default_value="9.0"),
        DeclareLaunchArgument("opponent_prediction_timestep_s", default_value="0.1"),
        DeclareLaunchArgument("use_overtaking", default_value="false"),
        DeclareLaunchArgument("overtaking_executable", default_value="overtaking_successor"),
        DeclareLaunchArgument("publish_red_lidar", default_value="true"),
        DeclareLaunchArgument("use_flight_recorder", default_value="false"),
        DeclareLaunchArgument("telemetry_run_root", default_value="reports/runs"),
        DeclareLaunchArgument("telemetry_run_id", default_value=""),
        DeclareLaunchArgument("record_rosbag", default_value="true"),
        DeclareLaunchArgument("use_spectator_hud", default_value="false"),
        DeclareLaunchArgument("review_phase", default_value="FOUNDATION"),
        OpaqueFunction(function=_instantiate_nodes),
    ])
