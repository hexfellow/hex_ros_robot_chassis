#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-09-05
################################################################

import numpy as np

import rospy

from hex_util_runtime import ns_now

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


class DataInterface(ChassisInterfaceBase):

    def __init__(self, name: str = "unknown"):
        rospy.init_node(name, anonymous=True)
        super().__init__(name)

        ### rate parameters
        self._rate_param["ros"] = rospy.get_param('~ctrl_rate', 1000.0)
        self._rate_param["state"] = rospy.get_param('~rate_state', 500.0)
        self.__rate = rospy.Rate(self._rate_param["ros"])

        ### robot parameters
        self._robot_param = {
            "host": rospy.get_param('~robot_host', "192.168.1.100"),
            "port": rospy.get_param('~robot_port', 8439),
            "frame_id": rospy.get_param('~robot_frame_id', "base_link"),
            "state_buffer_size": rospy.get_param('~state_buffer_size', 200),
            "sens_ts": rospy.get_param('~sens_ts', True),
            "enable_kcp": rospy.get_param('~enable_kcp', True),
        }

        ### time source — PTP (ns_now) or ROS clock
        self._use_ros_time = rospy.get_param('~use_ros_time', False)

        ### publisher — chs_state
        self.__chs_state_pub = rospy.Publisher(
            'chs_state',
            HexRosRoboChsStateStamped,
            queue_size=10,
        )
        ### publisher — odom
        self.__odom_pub = rospy.Publisher(
            'odom',
            Odometry,
            queue_size=10,
        )
        ### publisher — joint_states
        self.__joint_state_pub = rospy.Publisher(
            'joint_states',
            JointState,
            queue_size=10,
        )
        ### publisher — /tf
        self.__tf_pub = rospy.Publisher(
            '/tf',
            TFMessage,
            queue_size=10,
        )

        ### subscriber — chs_ctrl
        self.__chs_ctrl_sub = rospy.Subscriber(
            'chs_ctrl',
            HexRosRoboChsCtrlStamped,
            self.__chs_ctrl_callback,
        )
        self.__chs_ctrl_sub

    def sleep(self):
        self.__rate.sleep()

    ####################
    ### ros infrastructure
    ####################
    def ok(self) -> bool:
        return not rospy.is_shutdown()

    def shutdown(self):
        pass

    ####################
    ### logging
    ####################
    def logd(self, msg, *args, **kwargs):
        rospy.logdebug(msg, *args, **kwargs)

    def logi(self, msg, *args, **kwargs):
        rospy.loginfo(msg, *args, **kwargs)

    def logw(self, msg, *args, **kwargs):
        rospy.logwarn(msg, *args, **kwargs)

    def loge(self, msg, *args, **kwargs):
        rospy.logerr(msg, *args, **kwargs)

    def logf(self, msg, *args, **kwargs):
        rospy.logfatal(msg, *args, **kwargs)

    ####################
    ### time source
    ####################
    def now_ns(self) -> int:
        if self._use_ros_time:
            return rospy.Time.now().to_nsec()
        return ns_now()

    def now_stamp(self) -> HexDcBaseTime:
        now = rospy.Time.now()
        return HexDcBaseTime(secs=now.secs, nsecs=now.nsecs)

    ####################
    ### publishers
    ####################
    def pub_chs_state(self, out: HexDcRoboChsStateStamped):
        msg = HexRosRoboChsStateStamped()
        # ros time stamp
        now_stamp_dc = self.now_stamp()
        msg.header.stamp = rospy.Time(
            int(now_stamp_dc.secs),
            int(now_stamp_dc.nsecs),
        )
        msg.header.frame_id = out.header.frame_id

        jnt = out.chs_state.jnt
        hardware_stamp = rospy.Time(
            int(out.header.stamp.secs),
            int(out.header.stamp.nsecs),
        )
        msg.chs_state.jnt.header.stamp = hardware_stamp
        msg.chs_state.jnt.header.frame_id = out.header.frame_id
        msg.chs_state.jnt.name = JOINT_STATE_NAME
        msg.chs_state.jnt.position = \
            np.asarray(jnt.position, dtype=np.float64).tolist()
        msg.chs_state.jnt.velocity = \
            np.asarray(jnt.velocity, dtype=np.float64).tolist()
        msg.chs_state.jnt.effort = \
            np.asarray(jnt.effort, dtype=np.float64).tolist()

        odom = out.chs_state.odom
        msg.chs_state.odom.header.stamp = rospy.Time(
            int(now_stamp_dc.secs),
            int(now_stamp_dc.nsecs),
        )
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
        now_stamp_dc = self.now_stamp()
        msg.header.stamp = rospy.Time(
            int(now_stamp_dc.secs),
            int(now_stamp_dc.nsecs),
        )
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
        now_stamp_dc = self.now_stamp()
        msg.header.stamp = rospy.Time(
            int(now_stamp_dc.secs),
            int(now_stamp_dc.nsecs),
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
        now_stamp_dc = self.now_stamp()
        transform.header.stamp = rospy.Time(
            int(now_stamp_dc.secs),
            int(now_stamp_dc.nsecs),
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
                secs=int(msg.header.stamp.secs),
                nsecs=int(msg.header.stamp.nsecs),
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
