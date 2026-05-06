#!/usr/bin/env python

from functools import cached_property

from lerobot.teleoperators.so101_leader.config_so101_leader import SO101LeaderConfig
from lerobot.teleoperators.so101_leader.so101_leader import SO101Leader

from ..teleoperator import Teleoperator
from .config_so101_leader_to_bi_so101 import SO101LeaderToBiSO101Config


class SO101LeaderToBiSO101(Teleoperator):
    """Mirror one SO-101 leader action stream onto two SO-101 follower arms."""

    config_class = SO101LeaderToBiSO101Config
    name = "so101_leader_to_bi_so101"

    def __init__(self, config: SO101LeaderToBiSO101Config):
        super().__init__(config)
        self.config = config
        leader_config = SO101LeaderConfig(
            id=config.id,
            calibration_dir=config.calibration_dir,
            port=config.port,
            use_degrees=config.use_degrees,
        )
        self.leader = SO101Leader(leader_config)

    @cached_property
    def action_features(self) -> dict[str, type]:
        return {f"left_{motor}.pos": float for motor in self.leader.bus.motors} | {
            f"right_{motor}.pos": float for motor in self.leader.bus.motors
        }

    @cached_property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self.leader.is_connected

    def connect(self, calibrate: bool = True) -> None:
        self.leader.connect(calibrate)

    @property
    def is_calibrated(self) -> bool:
        return self.leader.is_calibrated

    def calibrate(self) -> None:
        self.leader.calibrate()

    def configure(self) -> None:
        self.leader.configure()

    def setup_motors(self) -> None:
        self.leader.setup_motors()

    def get_action(self) -> dict[str, float]:
        leader_action = self.leader.get_action()
        return {f"left_{key}": value for key, value in leader_action.items()} | {
            f"right_{key}": value for key, value in leader_action.items()
        }

    def send_feedback(self, feedback: dict[str, float]) -> None:
        # Keep the interface compatible; there is no force feedback on SO-101.
        return None

    def disconnect(self) -> None:
        self.leader.disconnect()
