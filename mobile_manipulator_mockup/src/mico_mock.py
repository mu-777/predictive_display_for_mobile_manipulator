#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import numpy as np
import common

from sensor_msgs.msg import JointState
from jaco_msgs.msg import JointAngles, JointVelocity
from utils.loop_time_manager import LoopTimeManager
from utils.mico_utils import joint_angles_deg2rad

REMAPPABLE_NODE_NAME = 'mobile_manipulator_mock'
PARAM_NAME_MICO_JOINTVEL_INPUT_TOPIC = '/mico_jointvel_input_topic'
PARAM_NAME_MICO_JOINTSTATE_OUTPUT_TOPIC = '/mico_jointstate_output_topic'


class MICOMockup(object):
    """
    Subscribe
        JointVelocity(mico_jointvel_input_topic)
    Publish
        JointState(mico_jointstate_output_topic)
        JointAngles(mico_jointangles_output_topic)
    """

    def __init__(self):
        self._joint_ang = joint_angles_deg2rad(JointAngles(joint1=0.0, joint2=-90.0, joint3=0.0,
                                                           joint4=0.0, joint5=90.0, joint6=0.0))
        self._joint_vel = JointVelocity(joint1=0.0, joint2=0.0, joint3=0.0,
                                        joint4=0.0, joint5=0.0, joint6=0.0)
        self._joint_state = JointState(name=['mico_joint_' + suffix for suffix in ['1', '2', '3', '4', '5', '6',
                                                                                   'finger_1', 'finger_2']],
                                       position=[getattr(self._joint_ang, a) for a in JointAngles.__slots__] + [0, 0],
                                       velocity=[getattr(self._joint_vel, a) for a in JointVelocity.__slots__] + [0, 0])
        self._joint_state_publisher = rospy.Publisher(rospy.get_param(PARAM_NAME_MICO_JOINTSTATE_OUTPUT_TOPIC),
                                                      JointState, queue_size=1)
        self.timer = LoopTimeManager()

    def activate(self):
        self._joint_state_publisher.publish()
        rospy.Subscriber(rospy.get_param(PARAM_NAME_MICO_JOINTVEL_INPUT_TOPIC),
                         JointVelocity, self._joint_vel_callback)
        rospy.loginfo('mico_mock activated')
        return self

    # HACKED
    def _joint_vel_callback(self, joint_vel):
        dt = self.timer.dt_sec
        for attr in JointAngles.__slots__:
            j_ang, j_vel = getattr(self._joint_ang, attr), getattr(self._joint_vel, attr)
            setattr(self._joint_ang, attr, j_ang + j_vel * dt)

        self._joint_vel = joint_vel
        self.timer.update()

    def publish(self):
        for idx, attr in enumerate(JointAngles.__slots__):
            self._joint_state.position[idx] = getattr(self._joint_ang, attr)
        for idx, attr in enumerate(JointVelocity.__slots__):
            self._joint_state.velocity[idx] = getattr(self._joint_vel, attr)
        self._joint_state_publisher.publish(self._joint_state)

# --------------------------------------------
if __name__ == '__main__':
    rospy.init_node(REMAPPABLE_NODE_NAME, anonymous=True)
    rate_mgr = rospy.Rate(100)  # Hz

    mico_mock = MICOMockup().activate()

    while not rospy.is_shutdown():
        mico_mock.publish()
        rate_mgr.sleep()
