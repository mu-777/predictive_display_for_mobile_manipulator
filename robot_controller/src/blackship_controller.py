#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import numpy as np
import common
from collections import namedtuple

from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Twist, TwistStamped, TransformStamped, Quaternion
from common.msg import NavigationReference, RobotState
from utils.util_functions import wait_for_tf, update_tf, transform2pose, pose2transform, euler_from_rosquaternion, \
    twist_from_vw
from utils.timeline_data import TimeLineData
from robot_controller_abstract import RobotControllerAbstract

REMAPPABLE_NODE_NAME = 'sample_to_use_nav_cmd_gen'

PARAM_NAME_OPE_STATE_TOPIC = '/operator_state_topic'
PARAM_NAME_NAVI_REF_TOPIC = '/navigation_reference_topic'
PARAM_NAME_BS_INPUT_TOPIC = '/blackship_input_topic'
PARAM_NAME_ROBOT_STATE_TOPIC = '/robot_state_topic'
PARAM_NAME_MANI_METHOD = '/mani_method'
PARAM_NAME_SYS_ORIGIN_FRAME_ID = '/system_origin_frame_id'
PARAM_NAME_AVATAR_BASE_FRAME_ID = '/avatar_base_frame_id'
PARAM_NAME_BS_CONTROL_METHOD_TARGETFOLLOWING = '/blackship_control_method_targetfollowing'


class BSCtrl_TargetFollowing(RobotControllerAbstract):
    def __init__(self, registered_tf_buffer=None):
        super(BSCtrl_TargetFollowing, self).__init__(TwistStamped,
                                                     rospy.get_param(PARAM_NAME_BS_INPUT_TOPIC),
                                                     registered_tf_buffer)

        self.base_pose_ref = Pose(orientation=Quaternion(w=1.0))
        self.base_pose_cur = Pose(orientation=Quaternion(w=1.0))
        self.base_twist_ref = Twist()
        self.base_twist_cur = Twist()
        self.theta_timeline = TimeLineData().initialize_values(length=10)

    def activate_controller(self):
        self._navi_ref_subscriber = rospy.Subscriber(rospy.get_param(PARAM_NAME_NAVI_REF_TOPIC),
                                                     NavigationReference,
                                                     self._navi_ref_callback)
        self._robot_state_subscriber = rospy.Subscriber(rospy.get_param(PARAM_NAME_ROBOT_STATE_TOPIC),
                                                        RobotState,
                                                        self._robot_state_callback)
        return True

    def update_input(self):
        v, w = self._calc_pd_input()
        return TwistStamped(header=Header(stamp=rospy.Time.now()),
                            twist=twist_from_vw(v, w))

    def _calc_pd_input(self):
        Kp, Kd = 1, 1
        theta = euler_from_rosquaternion(self.base_pose_cur.orientation)[0]
        R = np.matrix([[np.cos(theta), np.sin(theta)],
                       [-np.sin(theta), np.cos(theta)]])
        Xd_dot = np.matrix([[self.base_twist_ref.linear.x],
                            [self.base_twist_ref.linear.y]])

        Xd = np.matrix([[self.base_pose_ref.position.x],
                        [self.base_pose_ref.position.y]])

        X = np.matrix([[self.base_pose_cur.position.x],
                       [self.base_pose_cur.position.y]])

        u = R * (Kd * Xd_dot - Kp * (X - Xd))
        v, w = float(u[0]), float(u[1])
        return v, w


    def _calc_target_tracking_input(self):
        def dist_bw_points(pos1, pos2):
            return ((pos1.x - pos2.x) ** 2 + (pos1.y - pos2.y) ** 2 + (pos1.z - pos2.z) ** 2) ** 0.5

        def yaw_bw_orientations(ori1, ori2):
            return euler_from_rosquaternion(ori1)[0] - euler_from_rosquaternion(ori2)[0]

        Kv, Kt, KtD = 0.1, 1, 1
        self.theta_timeline.append(yaw_bw_orientations(self.base_pose_cur.orientation,
                                                       self.base_pose_ref.orientation))
        v = Kv * dist_bw_points(self.base_pose_ref.position, self.base_pose_cur.position)
        w = Kt * self.theta_timeline.dval + KtD * self.theta_timeline.differentiation
        print(v, w)
        return v, w


    def _navi_ref_callback(self, navi_ref):
        self.base_pose_ref = navi_ref.base_pose
        self.base_twist_ref = navi_ref.base_twist

    def _robot_state_callback(self, robot_state):
        self.base_pose_cur = robot_state.base_pose
        self.base_twist_cur = robot_state.base_twist


class BlackshipController(object):
    ctrl_method_map = {rospy.get_param(PARAM_NAME_BS_CONTROL_METHOD_TARGETFOLLOWING): BSCtrl_TargetFollowing}

    def __init__(self, method, registered_tf_buffer=None):
        if method in self.ctrl_method_map.keys():
            self._controller = self.ctrl_method_map[method](registered_tf_buffer)
        else:
            rospy.logerr('Invalid BlackShip Control Method selected!')

    @property
    def controller(self):
        return self._controller


