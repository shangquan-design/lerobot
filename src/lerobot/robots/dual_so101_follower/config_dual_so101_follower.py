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

from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig

from ..config import RobotConfig


@RobotConfig.register_subclass("dual_so101_follower")
@dataclass
class DualSO101FollowerConfig(RobotConfig):
    first_arm_port: str
    second_arm_port: str

    # Optional
    first_arm_id: str | None = None
    second_arm_id: str | None = None
    first_arm_disable_torque_on_disconnect: bool = True
    first_arm_max_relative_target: int | None = None
    first_arm_use_degrees: bool = False
    second_arm_disable_torque_on_disconnect: bool = True
    second_arm_max_relative_target: int | None = None
    second_arm_use_degrees: bool = False

    # Cameras shared by the two arms.
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
