#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import common

from jaco_msgs.msg import JointVelocity
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
from utils.util_functions import init_tf_stamped, pose2transform
from utils.data_container import DataContainer

REMAPPABLE_NODE_NAME = 'gazebo_interface'

PARAM_NAME_MOBILE_VW_CMD_TOPIC = '~bs_vw_cmd_topic'
PARAM_NAME_JOINT1_CMD_TOPIC = '~joint1_cmd_topic'
PARAM_NAME_JOINT2_CMD_TOPIC = '~joint2_cmd_topic'
PARAM_NAME_JOINT3_CMD_TOPIC = '~joint3_cmd_topic'
PARAM_NAME_JOINT4_CMD_TOPIC = '~joint4_cmd_topic'
PARAM_NAME_JOINT5_CMD_TOPIC = '~joint5_cmd_topic'
PARAM_NAME_JOINT6_CMD_TOPIC = '~joint6_cmd_topic'
PARAM_NAME_JOINT_STATE_TOPIC = '~joint_state_topic'
PARAM_NAME_BS_STATE_TOPIC = '~bs_state_topic'
PARAM_NAME_JOINTVEL_INPUT_TOPIC = '/mico_jointvel_input_topic'
PARAM_NAME_BS_INPUT_TOPIC = '/blackship_input_topic'
PARAM_NAME_ARM_JOINT_STATE_TOPIC = '/mico_jointstate_output_topic'
PARAM_NAME_MOBILE_WHEEL_RADIUS = '/mobile_wheel_radius'
PARAM_NAME_MOBILE_AXLE_TRACK = '/mobile_axle_track'


class MicoBlackshipGazeboInterface(object):
    def __init__(self):
        self._mobile_vel_cmd_publisher = rospy.Publisher(rospy.get_param(PARAM_NAME_MOBILE_VW_CMD_TOPIC,
                                                                         default=PARAM_NAME_MOBILE_VW_CMD_TOPIC[1:]),
                                                         TwistStamped, queue_size=1)
        self._joint_vel_cmd_publishers = []

        param_names = [PARAM_NAME_JOINT1_CMD_TOPIC, PARAM_NAME_JOINT2_CMD_TOPIC, PARAM_NAME_JOINT3_CMD_TOPIC,
                       PARAM_NAME_JOINT4_CMD_TOPIC, PARAM_NAME_JOINT5_CMD_TOPIC, PARAM_NAME_JOINT6_CMD_TOPIC]
        self._joint_vel_cmd_publishers = [rospy.Publisher(rospy.get_param(param_name, default=param_name[1:]),
                                                          Float64, queue_size=1) for param_name in param_names]
        self._js_publisher = rospy.Publisher(rospy.get_param(PARAM_NAME_ARM_JOINT_STATE_TOPIC),
                                             JointState, queue_size=1)
        self._joint_state = JointState()
        self.Rw = rospy.get_param(PARAM_NAME_MOBILE_WHEEL_RADIUS)
        self.T = rospy.get_param(PARAM_NAME_MOBILE_AXLE_TRACK)
        self._robot_vel_cur_container = DataContainer('RobotVelCur', data_class=list,
                                                      header_list=['j1', 'j2', 'j3', 'j4', 'j5', 'j6', 'v', 'w'])

    def activate(self):
        rospy.Subscriber(rospy.get_param(PARAM_NAME_BS_INPUT_TOPIC,
                                         default=PARAM_NAME_BS_INPUT_TOPIC[1:]),
                         TwistStamped, self._mobile_vel_callback)
        rospy.Subscriber(rospy.get_param(PARAM_NAME_JOINTVEL_INPUT_TOPIC,
                                         default=PARAM_NAME_JOINTVEL_INPUT_TOPIC[1:]),
                         JointVelocity, self._joint_vel_callback)
        rospy.Subscriber(rospy.get_param(PARAM_NAME_JOINT_STATE_TOPIC,
                                         default=PARAM_NAME_JOINT_STATE_TOPIC[1:]),
                         JointState, self._joint_state_callback)
        rospy.Subscriber(rospy.get_param(PARAM_NAME_BS_STATE_TOPIC,
                                         default=PARAM_NAME_BS_STATE_TOPIC[1:]),
                         JointState, self._bs_state_callback)
        return self

    def _mobile_vel_callback(self, twist_stamped):
        self._mobile_vel_cmd_publisher.publish(twist_stamped)

    def _joint_vel_callback(self, joint_vel):
        for attr, publisher in zip(JointVelocity.__slots__, self._joint_vel_cmd_publishers):
            publisher.publish(Float64(data=getattr(joint_vel, attr)))

    def _joint_state_callback(self, js):
        self._joint_state = js
        self._js_publisher.publish(js)

    def _bs_state_callback(self, js):
        thetal_dot = (js.velocity[0] + js.velocity[2]) * 0.5
        thetar_dot = (js.velocity[1] + js.velocity[3]) * 0.5
        v = (thetal_dot + thetar_dot) * self.Rw /2
        w = (thetal_dot - thetar_dot) * self.Rw / self.T
        self._robot_vel_cur_container.write(list(self._joint_state.velocity) + [v, w])

# --------------------------------------------
if __name__ == '__main__':
    rospy.init_node(REMAPPABLE_NODE_NAME, anonymous=True)
    gazebo_if = MicoBlackshipGazeboInterface().activate()
    rospy.spin()
