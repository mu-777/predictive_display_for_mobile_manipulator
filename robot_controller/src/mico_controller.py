#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import common

from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped, TwistStamped
from common.msg import ManipulationReference
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import PoseStamped, Transform, Quaternion, TransformStamped

from robot_controller_abstract import RobotControllerAbstract
from utils.util_functions import transform2pose, init_tf_stamped, update_tf, wait_for_tf, diff_poses

PARAM_NAME_MICO_INPUT_TOPIC = '/mico_input_topic'
PARAM_NAME_MANI_REF_TOPIC = '/manipulation_reference_topic'
PARAM_NAME_MICO_EEPOSE_INPUT_ACTIONTOPIC = '/mico_eepose_input_actiontopic'

PARAM_NAME_SYS_ORIGIN_FRAME_ID = '/system_origin_frame_id'
PARAM_NAME_ARM_CTRL_ORIGIN_FRAME_ID = '/arm_base_frame_id'
PARAM_NAME_HAND_FRAME_ID = '/hand_frame_id'
PARAM_NAME_MICO_CONTROL_METHOD_EEPOS = '/mico_control_method_eepos'


class MicoCtrl_EEPose(RobotControllerAbstract):
    """
    Action is used in jaco-ros to control with end-effector's pose.
    But, Action is just a wrapper of some usual topic publishers and subscribers
    Here, topic publishers and subscribers are used instead of Action
    because some functions in Action are not necessary fot this and just for easy and unified implementation
    """

    def __init__(self, registered_tf_buffer=None):
        self._namespace = rospy.get_param(PARAM_NAME_MICO_EEPOSE_INPUT_ACTIONTOPIC)
        super(MicoCtrl_EEPose, self).__init__(PoseStamped, self._namespace + '/goal',
                                              registered_tf_buffer)

        self.cancel_publisher = rospy.Publisher(self._namespace + '/cancel',
                                                GoalID,
                                                queue_size=1)
        self.ref_pose_from_origin_frame = Pose(orientation=Quaternion(w=1.0))
        self.arm_base_tf = init_tf_stamped(rospy.get_param(PARAM_NAME_SYS_ORIGIN_FRAME_ID),
                                           rospy.get_param(PARAM_NAME_ARM_CTRL_ORIGIN_FRAME_ID))

    def activate_controller(self):
        self.arm_base_tf = wait_for_tf(self._tf_buffer,
                                       self.arm_base_tf.header.frame_id,
                                       self.arm_base_tf.child_frame_id)
        self._feedback_subscriber = rospy.Subscriber(self._namespace + '/feedback',
                                                     PoseStamped,
                                                     self._feedback_callback)
        self._mani_ref_subscriber = rospy.Subscriber(rospy.get_param(PARAM_NAME_MANI_REF_TOPIC),
                                                     ManipulationReference,
                                                     self._mani_ref_callback)
        return True

    def update_input(self):
        self.arm_base_tf = update_tf(self._tf_buffer, self.arm_base_tf)
        # TODO: diff_poses may be wrong
        ee_pose_from_arm_base = diff_poses(self.ref_pose_from_origin_frame, self.arm_base_tf)
        return PoseStamped(header=Header(stamp=rospy.Time.now()),
                           pose=ee_pose_from_arm_base)

    @property
    def is_reachable(self):
        # TODO
        return False

    def _mani_ref_callback(self, mani_ref):
        self.ref_pose_from_origin_frame = mani_ref.ee_pose

    def cancel_all_goals(self):
        self.cancel_publisher.publish(GoalID(stamp=rospy.Time.from_sec(0.0), id=""))
        return self

    def _feedback_callback(self, pose_stamped):
        # TODO
        pass


class MicoController(object):
    ctrl_method_map = {rospy.get_param(PARAM_NAME_MICO_CONTROL_METHOD_EEPOS): MicoCtrl_EEPose}

    def __init__(self, method, registered_tf_buffer=None):
        if method in self.ctrl_method_map.keys():
            self._controller = self.ctrl_method_map[method](registered_tf_buffer)
        else:
            rospy.logerr('Invalid MICOControlMethod is selected!')

    @property
    def controller(self):
        return self._controller

