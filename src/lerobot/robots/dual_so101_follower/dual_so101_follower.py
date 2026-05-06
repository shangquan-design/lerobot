#!/usr/bin/env python

# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import logging
import time
from functools import cached_property
from typing import Any

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.robots.so101_follower import SO101Follower
from lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig

from ..robot import Robot
from .config_dual_so101_follower import DualSO101FollowerConfig

logger = logging.getLogger(__name__)


class DualSO101Follower(Robot):
    """
    Two SO-101 follower arms driven by one unprefixed single-arm action.

    This is intended for one SO-101 leader arm controlling two SO-101 follower
    arms in parallel. The same action dict is sent to both follower arms.
    """

    config_class = DualSO101FollowerConfig
    name = "dual_so101_follower"

    def __init__(self, config: DualSO101FollowerConfig):
        super().__init__(config)
        self.config = config

        first_arm_config = SO101FollowerConfig(
            id=config.first_arm_id or (f"{config.id}_first" if config.id else None),
            calibration_dir=config.calibration_dir,
            port=config.first_arm_port,
            disable_torque_on_disconnect=config.first_arm_disable_torque_on_disconnect,
            max_relative_target=config.first_arm_max_relative_target,
            use_degrees=config.first_arm_use_degrees,
            cameras={},
        )

        second_arm_config = SO101FollowerConfig(
            id=config.second_arm_id or (f"{config.id}_second" if config.id else None),
            calibration_dir=config.calibration_dir,
            port=config.second_arm_port,
            disable_torque_on_disconnect=config.second_arm_disable_torque_on_disconnect,
            max_relative_target=config.second_arm_max_relative_target,
            use_degrees=config.second_arm_use_degrees,
            cameras={},
        )

        self.first_arm = SO101Follower(first_arm_config)
        self.second_arm = SO101Follower(second_arm_config)
        self.cameras = make_cameras_from_configs(config.cameras)
        self._last_first_action: dict[str, Any] = dict.fromkeys(self.first_arm.action_features, 0.0)
        self._last_second_action: dict[str, Any] = dict.fromkeys(self.second_arm.action_features, 0.0)

    @property
    def _motors_ft(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in self.first_arm.bus.motors}

    @property
    def _observation_motors_ft(self) -> dict[str, type]:
        return {f"first_{motor}.pos": float for motor in self.first_arm.bus.motors} | {
            f"second_{motor}.pos": float for motor in self.second_arm.bus.motors
        }

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3) for cam in self.cameras
        }

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._observation_motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        return (
            self.first_arm.bus.is_connected
            and self.second_arm.bus.is_connected
            and all(cam.is_connected for cam in self.cameras.values())
        )

    def connect(self, calibrate: bool = True) -> None:
        self.first_arm.connect(calibrate)
        self.second_arm.connect(calibrate)

        for cam in self.cameras.values():
            cam.connect()

    @property
    def is_calibrated(self) -> bool:
        return self.first_arm.is_calibrated and self.second_arm.is_calibrated

    def calibrate(self) -> None:
        self.first_arm.calibrate()
        self.second_arm.calibrate()

    def configure(self) -> None:
        self.first_arm.configure()
        self.second_arm.configure()

    def setup_motors(self) -> None:
        self.first_arm.setup_motors()
        self.second_arm.setup_motors()

    def get_observation(self) -> dict[str, Any]:
        obs_dict = {}

        obs_dict.update({f"first_{key}": value for key, value in self._last_first_action.items()})
        obs_dict.update({f"second_{key}": value for key, value in self._last_second_action.items()})

        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            try:
                obs_dict[cam_key] = cam.async_read(timeout_ms=1000)
            except TimeoutError as e:
                logger.warning(f"{self} skipped {cam_key}: {e}")
                continue
            dt_ms = (time.perf_counter() - start) * 1e3
            logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")

        return obs_dict

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        send_action_first = self.first_arm.send_action(action)
        send_action_second = self.second_arm.send_action(action)
        self._last_first_action = send_action_first
        self._last_second_action = send_action_second

        # Keep the returned action compatible with a single SO101 leader while
        # still making both physical writes happen.
        return send_action_first | send_action_second

    def disconnect(self):
        self.first_arm.disconnect()
        self.second_arm.disconnect()

        for cam in self.cameras.values():
            cam.disconnect()
