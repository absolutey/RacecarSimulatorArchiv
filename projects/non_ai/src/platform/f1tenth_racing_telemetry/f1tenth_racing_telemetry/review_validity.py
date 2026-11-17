from dataclasses import dataclass


@dataclass(frozen=True)
class CollisionTransition:
    kind: str
    episode_id: int
    wall: bool
    vehicle: bool


class LapCollisionReview:
    """Lap-local validity and edge-counted collision review state."""

    def __init__(self):
        self.lap = 0
        self.lap_valid = True
        self.wall_active = False
        self.vehicle_active = False
        self.episode_id = 0
        self.wall_episodes = 0
        self.vehicle_episodes = 0
        self._episode_wall = False
        self._episode_vehicle = False

    def set_lap(self, lap):
        if lap > self.lap:
            self.lap_valid = True
        self.lap = lap

    @property
    def episode_wall(self):
        return self._episode_wall

    @property
    def episode_vehicle(self):
        return self._episode_vehicle

    def update(self, *, wall=None, vehicle=None):
        was_active = self.wall_active or self.vehicle_active
        if wall is not None:
            self.wall_active = wall
        if vehicle is not None:
            self.vehicle_active = vehicle
        active = self.wall_active or self.vehicle_active

        if active and not was_active:
            self.episode_id += 1
            self.lap_valid = False
            self._episode_wall = self.wall_active
            self._episode_vehicle = self.vehicle_active
            self.wall_episodes += int(self._episode_wall)
            self.vehicle_episodes += int(self._episode_vehicle)
            return CollisionTransition(
                "ENTER", self.episode_id, self._episode_wall, self._episode_vehicle
            )
        if active:
            if self.wall_active and not self._episode_wall:
                self._episode_wall = True
                self.wall_episodes += 1
            if self.vehicle_active and not self._episode_vehicle:
                self._episode_vehicle = True
                self.vehicle_episodes += 1
            return None
        if was_active:
            return CollisionTransition(
                "EXIT", self.episode_id, self._episode_wall, self._episode_vehicle
            )
        return None
