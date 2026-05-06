#!/usr/bin/env python

from dataclasses import dataclass

from ..config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("so101_leader_to_bi_so101")
@dataclass
class SO101LeaderToBiSO101Config(TeleoperatorConfig):
    port: str
    use_degrees: bool = False
