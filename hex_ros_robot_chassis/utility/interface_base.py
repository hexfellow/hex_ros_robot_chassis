#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-09-05
################################################################

from collections import deque
from typing import Any, Optional
from abc import ABC, abstractmethod

from hex_util_msg.dataclass.dataclass_base import HexDcBaseTime
from hex_util_msg.dataclass.dataclass_robo import (
    HexDcRoboChsCtrlStamped,
    HexDcRoboChsStateStamped,
)

JOINT_STATE_NAME = ["joint_wheel1", "joint_wheel2", "joint_wheel3"]


class ChassisInterfaceBase(ABC):

    def __init__(self, name: str = "unknown"):
        self._name = name

        ### ros parameters
        self._rate_param = {}
        self._robot_param = {}

        ### rx msg queues
        self._chs_ctrl_deque = deque(maxlen=100)

        print(f"#### ChassisInterfaceBase init: {self._name} ####")

    ####################
    ### ros infrastructure
    ####################
    @abstractmethod
    def ok(self) -> bool:
        raise NotImplementedError("ChassisInterfaceBase.ok")

    @abstractmethod
    def shutdown(self):
        raise NotImplementedError("ChassisInterfaceBase.shutdown")

    @abstractmethod
    def sleep(self):
        raise NotImplementedError("ChassisInterfaceBase.sleep")

    @abstractmethod
    def now_ns(self) -> int:
        raise NotImplementedError("ChassisInterfaceBase.now_ns")

    @abstractmethod
    def now_stamp(self) -> HexDcBaseTime:
        raise NotImplementedError("ChassisInterfaceBase.now_stamp")

    ####################
    ### logging
    ####################
    @abstractmethod
    def logd(self, msg, *args, **kwargs):
        raise NotImplementedError("ChassisInterfaceBase.logd")

    @abstractmethod
    def logi(self, msg, *args, **kwargs):
        raise NotImplementedError("ChassisInterfaceBase.logi")

    @abstractmethod
    def logw(self, msg, *args, **kwargs):
        raise NotImplementedError("ChassisInterfaceBase.logw")

    @abstractmethod
    def loge(self, msg, *args, **kwargs):
        raise NotImplementedError("ChassisInterfaceBase.loge")

    @abstractmethod
    def logf(self, msg, *args, **kwargs):
        raise NotImplementedError("ChassisInterfaceBase.logf")

    ####################
    ### parameters
    ####################
    def get_rate_param(self) -> dict:
        return self._rate_param

    def get_robot_param(self) -> dict:
        return self._robot_param

    ####################
    ### publishers
    ####################
    @abstractmethod
    def pub_chs_state(self, out: HexDcRoboChsStateStamped):
        raise NotImplementedError("ChassisInterfaceBase.pub_chs_state")

    @abstractmethod
    def pub_odom(self, out: HexDcRoboChsStateStamped):
        raise NotImplementedError("ChassisInterfaceBase.pub_odom")

    @abstractmethod
    def pub_joint_state(self, out: HexDcRoboChsStateStamped):
        raise NotImplementedError("ChassisInterfaceBase.pub_joint_state")

    @abstractmethod
    def pub_tf(self, out: HexDcRoboChsStateStamped):
        raise NotImplementedError("ChassisInterfaceBase.pub_tf")

    ####################
    ### subscribers
    ####################
    @staticmethod
    def deque_helper(dq: deque, latest: bool = False) -> Optional[Any]:
        if not latest:
            if dq:
                return dq.popleft()
            else:
                return None
        else:
            if dq:
                ret = dq[-1]
                dq.clear()
                return ret
            else:
                return None

    # chs ctrl
    def get_chs_ctrl(
        self,
        latest: bool = False,
    ) -> Optional[HexDcRoboChsCtrlStamped]:
        return self.deque_helper(self._chs_ctrl_deque, latest)
