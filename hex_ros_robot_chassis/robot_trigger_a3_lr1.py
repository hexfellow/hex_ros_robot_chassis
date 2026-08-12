#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-09-05
################################################################

import os
import sys
from turtle import speed
from typing import Optional

import numpy as np

scrpit_path = os.path.abspath(os.path.dirname(__file__))
sys.path.append(scrpit_path)
from utility import DataInterface

from hex_driver_robot import HexRobotTriggerA3Lr1, HexRobotTriggerA3Lr1Params

from hex_util_msg.dataclass.dataclass_robo import (
    HexDcRoboChsCtrlMode,
    HexDcRoboChsCtrlStamped,
    HexDcRoboChsState,
    HexDcRoboChsStateStamped,
)
from hex_util_msg.dataclass.dataclass_base import (
    HexDcBaseHeader,
    HexDcBaseTime,
    HexDcBaseJntState,
    HexDcBaseVector3,
    HexDcBaseQuaternion,
    HexDcBasePose,
    HexDcBaseTwist,
    HexDcBaseOdometry,
)
from hex_util_runtime import hex_ts_to_ns, ns_now


# Trigger A3 LR1 3-wheel joint names
JOINT_STATE_NAME = ["joint_1", "joint_2", "joint_3"]


class RobotTriggerA3Lr1:

    def __init__(self):
        ### utility
        self.__data_interface = DataInterface("hex_ros_robot_trigger_a3_lr1")

        ### parameters
        rate_param = self.__data_interface.get_rate_param()
        robot_param = self.__data_interface.get_robot_param()
        self.__data_interface.logi(f"ctrl_rate: {rate_param['ros']} hz")
        self.__data_interface.logi(f"rate_state: {rate_param['state']} hz")
        self.__data_interface.logi(f"robot_host: {robot_param['host']}")
        self.__data_interface.logi(f"robot_port: {robot_param['port']}")
        self.__data_interface.logi(f"robot_frame_id: {robot_param['frame_id']}")
        self.__data_interface.logi(f"state_buffer_size: {robot_param['state_buffer_size']}")
        self.__data_interface.logi(f"sens_ts: {robot_param['sens_ts']}")
        self.__data_interface.logi(f"enable_kcp: {robot_param['enable_kcp']}")

        ### robot driver
        self.__robot = HexRobotTriggerA3Lr1(HexRobotTriggerA3Lr1Params(
            host=robot_param["host"],
            port=robot_param["port"],
            ctrl_rate=rate_param["ros"],
            state_buffer_size=robot_param["state_buffer_size"],
            sens_ts=robot_param["sens_ts"],
            enable_kcp=robot_param["enable_kcp"],
        ))
        self.__robot.start()

        self.__data_interface.set_joint_names(JOINT_STATE_NAME)
        self.__robot.clear_odom_bias()

        ### derived
        self.__state_decim = max(
            1,
            int(round(rate_param["ros"] / rate_param["state"])),
        )
        self.__robot_frame_id = robot_param["frame_id"]
        self.__sens_ts = robot_param["sens_ts"]

    def __apply_chs_ctrl(self, ctrl: HexDcRoboChsCtrlStamped):
        """Dispatch chs_ctrl to the robot driver based on control mode."""
        chs_ctrl = ctrl.chs_ctrl
        mode = chs_ctrl.ctrl_mode

        if mode == HexDcRoboChsCtrlMode.VEL:
            # VEL mode: vel → (vx, vy, omega)
            vel = chs_ctrl.vel
            self.__robot.set_chs_vel_cmd({
                "vx": vel.linear.x,
                "vy": vel.linear.y,
                "omega": vel.angular.z,
            })

        elif mode == HexDcRoboChsCtrlMode.MIT:

            speeds = chs_ctrl.jnt.vel
            max_currents = chs_ctrl.jnt.eff
            
            if len(max_currents) >len(JOINT_STATE_NAME)or len(speeds) > len(JOINT_STATE_NAME):
                
                self.__data_interface.logw(f"Parameter count exceeds wheel count.")
                max_currents=np.zeros(len(JOINT_STATE_NAME))
                speeds=np.zeros(len(JOINT_STATE_NAME))
            
            self.__robot.set_chs_per_motor_spd_cmd({
                "speed": speeds,
                "max_current": max_currents,
            })

        # NONE: no-op

    def __build_chs_state(
        self,
    ) -> Optional[HexDcRoboChsStateStamped]:
        """Build HexDcRoboChsStateStamped from driver state getters."""
        motor_status = self.__robot.get_chassis_motor_status()
        if motor_status is None:
            return None

        ts_ns = hex_ts_to_ns(motor_status["ts"]) if self.__sens_ts else ns_now()

        # vehicle_position: (x, y, yaw), vehicle_speed: (vx, vy, omega)
        pos_raw = self.__robot.get_vehicle_position()
        spd_raw = self.__robot.get_vehicle_speed()

        if pos_raw is not None:
            odom_pose = HexDcBasePose(
                position=HexDcBaseVector3(x=pos_raw[0], y=pos_raw[1], z=0.0),
                orientation=HexDcBaseQuaternion(
                    x=0.0, y=0.0,
                    z=np.sin(pos_raw[2] * 0.5),
                    w=np.cos(pos_raw[2] * 0.5),
                ),
            )
        else:
            odom_pose = HexDcBasePose(
                position=HexDcBaseVector3(x=0.0, y=0.0, z=0.0),
                orientation=HexDcBaseQuaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            )
            self.__data_interface.logw("Vehicle position is None, using default pose.")

        if spd_raw is not None:
            odom_twist = HexDcBaseTwist(
                linear=HexDcBaseVector3(x=spd_raw[0], y=spd_raw[1], z=0.0),
                angular=HexDcBaseVector3(x=0.0, y=0.0, z=spd_raw[2]),
            )
        else:
            odom_twist = HexDcBaseTwist(
                linear=HexDcBaseVector3(x=0.0, y=0.0, z=0.0),
                angular=HexDcBaseVector3(x=0.0, y=0.0, z=0.0),
            )
            self.__data_interface.logw("Vehicle speed is None, using default twist.")

        return HexDcRoboChsStateStamped(
            header=HexDcBaseHeader(
                stamp=HexDcBaseTime(
                    secs=int(ts_ns // 1_000_000_000),
                    nsecs=int(ts_ns % 1_000_000_000),
                ),
                frame_id=self.__robot_frame_id,
            ),
            chs_state=HexDcRoboChsState(
                jnt=HexDcBaseJntState(
                    position=motor_status["pos"],
                    velocity=motor_status["vel"],
                    effort=motor_status["eff"],
                ),
                odom=HexDcBaseOdometry(pose=odom_pose, twist=odom_twist),
            ),
        )

    def run(self):
        state_count = 0
        while self.__data_interface.ok() and self.__robot.is_working():
            # 1. drain to the latest control frame
            ctrl = self.__data_interface.get_chs_ctrl(latest=True)
            if ctrl is not None:
                self.__apply_chs_ctrl(ctrl)

            # 3. publish robot state at the requested rate
            state_count += 1
            if state_count >= self.__state_decim:
                state_count = 0

                chs_state = self.__build_chs_state()
                if chs_state is not None:
                    self.__data_interface.pub_chs_state(chs_state)
                    self.__data_interface.pub_odom(chs_state)
                    self.__data_interface.pub_joint_state(chs_state)
                    self.__data_interface.pub_tf(chs_state)

            self.__data_interface.sleep()

    def shutdown(self):
        try:
            self.__robot.stop()
        except Exception:
            pass
        try:
            self.__data_interface.shutdown()
        except Exception:
            pass


def main():
    robot_trigger_a3_lr1 = RobotTriggerA3Lr1()
    try:
        robot_trigger_a3_lr1.run()
    except KeyboardInterrupt:
        pass
    finally:
        robot_trigger_a3_lr1.shutdown()


if __name__ == '__main__':
    main()
