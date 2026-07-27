#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-09-05
################################################################

import numpy as np
import threading

from hex_util_runtime import ns_now

import rclpy
import rclpy.node

from builtin_interfaces.msg import Time
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, TransformStamped, Vector3
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from hex_ros_msgs.msg import (
    HexRosJnt,
    HexRosRoboChsStateStamped,
    HexRosRoboChsCtrlStamped,
)

from hex_util_msg.dataclass.dataclass_base import (
    HexDcBaseHeader,
    HexDcBaseTime,
    HexDcBaseVector3,
    HexDcBaseQuaternion,
    HexDcBasePose,
    HexDcBaseJntFull,
    HexDcBaseJntState,
    HexDcBaseOdometry,
    HexDcBaseTwist,
)
from hex_util_msg.dataclass.dataclass_robo import (
    HexDcRoboChsCtrl,
    HexDcRoboChsCtrlMode,
    HexDcRoboChsCtrlStamped,
    HexDcRoboChsState,
    HexDcRoboChsStateStamped,
)

from .interface_base import ChassisInterfaceBase
from .interface_base import JOINT_STATE_NAME

from rclpy.logging import LoggingSeverity

class DataInterface(ChassisInterfaceBase):

    def __init__(self, name: str = "unknown"):
        rclpy.init()
        self.__node = rclpy.node.Node(name)
        self.__logger = self.__node.get_logger()
        # self.__logger.set_level(LoggingSeverity.DEBUG)
        self._shutting_down = False
        self.__spin_thread = threading.Thread(target=self.__spin)
        self.__spin_thread.start()

        super().__init__(name)

        ### rate parameters
        self.__node.declare_parameter('ctrl_rate', 1000.0)
        self.__node.declare_parameter('rate_state', 500.0)
        self._rate_param["ros"] = self.__node.get_parameter('ctrl_rate').value
        self._rate_param["state"] = self.__node.get_parameter('rate_state').value
        self.__rate = self.__node.create_rate(self._rate_param["ros"])

        ### robot parameters
        self.__node.declare_parameter('robot_host', "192.168.1.100")
        self.__node.declare_parameter('robot_port', 8439)
        self.__node.declare_parameter('robot_frame_id', "base_link")
        self.__node.declare_parameter('state_buffer_size', 200)
        self.__node.declare_parameter('sens_ts', True)
        self.__node.declare_parameter('enable_kcp', True)
        self.__node.declare_parameter('use_ros_time', False)
        self._robot_param = {
            "host": self.__node.get_parameter('robot_host').value,
            "port": self.__node.get_parameter('robot_port').value,
            "frame_id": self.__node.get_parameter('robot_frame_id').value,
            "state_buffer_size": self.__node.get_parameter('state_buffer_size').value,
            "sens_ts": self.__node.get_parameter('sens_ts').value,
            "enable_kcp": self.__node.get_parameter('enable_kcp').value,
        }

        ### time source — PTP (ns_now) or ROS clock
        self._use_ros_time = self.__node.get_parameter('use_ros_time').value

        ### publisher — chs_state
        self.__chs_state_pub = self.__node.create_publisher(
            HexRosRoboChsStateStamped,
            'chs_state',
            10,
        )
        ### publisher — odom
        self.__odom_pub = self.__node.create_publisher(
            Odometry,
            'odom',
            10,
        )
        ### publisher — joint_states (for robot_state_publisher / rviz)
        self.__joint_state_pub = self.__node.create_publisher(
            JointState,
            'joint_states',
            10,
        )
        ### publisher — /tf
        self.__tf_pub = self.__node.create_publisher(
            TFMessage,
            '/tf',
            10,
        )

        ### subscriber — chs_ctrl
        self.__chs_ctrl_sub = self.__node.create_subscription(
            HexRosRoboChsCtrlStamped,
            'chs_ctrl',
            self.__chs_ctrl_callback,
            10,
        )
        self.__chs_ctrl_sub

    def sleep(self):
        self.__rate.sleep()

    ####################
    ### ros infrastructure
    ####################
    def ok(self) -> bool:
        return rclpy.ok()

    def shutdown(self):
        if self._shutting_down:
            return
        self._shutting_down = True
        try:
            self.__node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        self.__spin_thread.join()

    def __spin(self):
        try:
            rclpy.spin(self.__node)
        except rclpy.executors.ExternalShutdownException:
            pass

    ####################
    ### logging
    ####################
    def logd(self, msg, *args, **kwargs):
        self.__logger.debug(msg, *args, **kwargs)

    def logi(self, msg, *args, **kwargs):
        self.__logger.info(msg, *args, **kwargs)

    def logw(self, msg, *args, **kwargs):
        self.__logger.warning(msg, *args, **kwargs)

    def loge(self, msg, *args, **kwargs):
        self.__logger.error(msg, *args, **kwargs)

    def logf(self, msg, *args, **kwargs):
        self.__logger.fatal(msg, *args, **kwargs)

    ####################
    ### time source
    ####################
    def now_ns(self) -> int:
        if self._use_ros_time:
            return self.__node.get_clock().now().nanoseconds
        return ns_now()

    ####################
    ### publishers
    ####################
    def pub_chs_state(self, out: HexDcRoboChsStateStamped):
        msg = HexRosRoboChsStateStamped()
        stamp = Time(
            sec=int(out.header.stamp.secs),
            nanosec=int(out.header.stamp.nsecs),
        )
        msg.header.stamp = stamp
        msg.header.frame_id = out.header.frame_id

        jnt = out.chs_state.jnt
        msg.chs_state.jnt.header.stamp = stamp
        msg.chs_state.jnt.header.frame_id = out.header.frame_id
        msg.chs_state.jnt.name = JOINT_STATE_NAME
        msg.chs_state.jnt.position = \
            np.asarray(jnt.position, dtype=np.float64).tolist()
        msg.chs_state.jnt.velocity = \
            np.asarray(jnt.velocity, dtype=np.float64).tolist()
        msg.chs_state.jnt.effort = \
            np.asarray(jnt.effort, dtype=np.float64).tolist()

        odom = out.chs_state.odom
        msg.chs_state.odom.header.stamp = stamp
        msg.chs_state.odom.header.frame_id = "odom"
        msg.chs_state.odom.child_frame_id = out.header.frame_id
        msg.chs_state.odom.pose.pose.position.x = odom.pose.position.x
        msg.chs_state.odom.pose.pose.position.y = odom.pose.position.y
        msg.chs_state.odom.pose.pose.position.z = odom.pose.position.z
        msg.chs_state.odom.pose.pose.orientation.x = odom.pose.orientation.x
        msg.chs_state.odom.pose.pose.orientation.y = odom.pose.orientation.y
        msg.chs_state.odom.pose.pose.orientation.z = odom.pose.orientation.z
        msg.chs_state.odom.pose.pose.orientation.w = odom.pose.orientation.w
        msg.chs_state.odom.twist.twist.linear.x = odom.twist.linear.x
        msg.chs_state.odom.twist.twist.linear.y = odom.twist.linear.y
        msg.chs_state.odom.twist.twist.linear.z = odom.twist.linear.z
        msg.chs_state.odom.twist.twist.angular.x = odom.twist.angular.x
        msg.chs_state.odom.twist.twist.angular.y = odom.twist.angular.y
        msg.chs_state.odom.twist.twist.angular.z = odom.twist.angular.z

        self.__chs_state_pub.publish(msg)

    def pub_odom(self, out: HexDcRoboChsStateStamped):
        msg = Odometry()
        stamp = Time(
            sec=int(out.header.stamp.secs),
            nanosec=int(out.header.stamp.nsecs),
        )
        msg.header.stamp = stamp
        msg.header.frame_id = "odom"
        msg.child_frame_id = out.header.frame_id
        msg.pose.pose.position.x = out.chs_state.odom.pose.position.x
        msg.pose.pose.position.y = out.chs_state.odom.pose.position.y
        msg.pose.pose.position.z = out.chs_state.odom.pose.position.z
        msg.pose.pose.orientation.x = out.chs_state.odom.pose.orientation.x
        msg.pose.pose.orientation.y = out.chs_state.odom.pose.orientation.y
        msg.pose.pose.orientation.z = out.chs_state.odom.pose.orientation.z
        msg.pose.pose.orientation.w = out.chs_state.odom.pose.orientation.w
        msg.twist.twist.linear.x = out.chs_state.odom.twist.linear.x
        msg.twist.twist.linear.y = out.chs_state.odom.twist.linear.y
        msg.twist.twist.linear.z = out.chs_state.odom.twist.linear.z
        msg.twist.twist.angular.x = out.chs_state.odom.twist.angular.x
        msg.twist.twist.angular.y = out.chs_state.odom.twist.angular.y
        msg.twist.twist.angular.z = out.chs_state.odom.twist.angular.z
        self.__odom_pub.publish(msg)

    def pub_joint_state(self, out: HexDcRoboChsStateStamped):
        msg = JointState()
        msg.header.stamp = Time(
            sec=int(out.header.stamp.secs),
            nanosec=int(out.header.stamp.nsecs),
        )
        msg.header.frame_id = out.header.frame_id
        msg.name = JOINT_STATE_NAME
        msg.position = \
            np.asarray(out.chs_state.jnt.position, dtype=np.float64).tolist()
        msg.velocity = \
            np.asarray(out.chs_state.jnt.velocity, dtype=np.float64).tolist()
        msg.effort = \
            np.asarray(out.chs_state.jnt.effort, dtype=np.float64).tolist()
        self.__joint_state_pub.publish(msg)

    def pub_tf(self, out: HexDcRoboChsStateStamped):
        tf_msg = TFMessage()
        transform = TransformStamped()
        transform.header.stamp = Time(
            sec=int(out.header.stamp.secs),
            nanosec=int(out.header.stamp.nsecs),
        )
        transform.header.frame_id = "odom"
        transform.child_frame_id = out.header.frame_id
        transform.transform.translation.x = out.chs_state.odom.pose.position.x
        transform.transform.translation.y = out.chs_state.odom.pose.position.y
        transform.transform.translation.z = out.chs_state.odom.pose.position.z
        transform.transform.rotation.x = out.chs_state.odom.pose.orientation.x
        transform.transform.rotation.y = out.chs_state.odom.pose.orientation.y
        transform.transform.rotation.z = out.chs_state.odom.pose.orientation.z
        transform.transform.rotation.w = out.chs_state.odom.pose.orientation.w
        tf_msg.transforms.append(transform)
        self.__tf_pub.publish(tf_msg)

    ####################
    ### subscribers
    ####################
    def __chs_ctrl_callback(self, msg: HexRosRoboChsCtrlStamped):
        self._chs_ctrl_deque.append(self.__chs_ctrl_msg_to_dc(msg))

    @staticmethod
    def __chs_ctrl_msg_to_dc(
            msg: HexRosRoboChsCtrlStamped) -> HexDcRoboChsCtrlStamped:
        header = HexDcBaseHeader(
            stamp=HexDcBaseTime(
                secs=int(msg.header.stamp.sec),
                nsecs=int(msg.header.stamp.nanosec),
            ),
            frame_id=msg.header.frame_id,
        )

        chs_msg = msg.chs_ctrl
        chs_ctrl = HexDcRoboChsCtrl(
            ctrl_mode=HexDcRoboChsCtrlMode(int(chs_msg.ctrl_mode)),
            jnt=DataInterface.__jnt_to_dc(chs_msg.jnt),
            vel=HexDcBaseTwist(
                linear=HexDcBaseVector3(
                    x=chs_msg.vel.linear.x,
                    y=chs_msg.vel.linear.y,
                    z=chs_msg.vel.linear.z,
                ),
                angular=HexDcBaseVector3(
                    x=chs_msg.vel.angular.x,
                    y=chs_msg.vel.angular.y,
                    z=chs_msg.vel.angular.z,
                ),
            ),
        )

        return HexDcRoboChsCtrlStamped(
            header=header,
            chs_ctrl=chs_ctrl,
        )

    @staticmethod
    def __jnt_to_dc(jnt: HexRosJnt) -> HexDcBaseJntFull:
        return HexDcBaseJntFull(
            pos=np.asarray(jnt.pos, dtype=np.float64),
            vel=np.asarray(jnt.vel, dtype=np.float64),
            eff=np.asarray(jnt.eff, dtype=np.float64),
            kp=np.asarray(jnt.kp, dtype=np.float64),
            kd=np.asarray(jnt.kd, dtype=np.float64),
            lim_vel=np.asarray(jnt.lim_vel, dtype=np.float64),
            lim_acc=np.asarray(jnt.lim_acc, dtype=np.float64),
        )
