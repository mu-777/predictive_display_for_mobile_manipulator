#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import common
import numpy as np

from jaco_msgs.msg import JointVelocity
from robot_controller_abstract import RobotControllerAbstract
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped, Twist, TwistStamped, Quaternion
from common.msg import ManipulationReference, RobotState
from utils.util_functions import transform2pose, update_tf, wait_for_tf, diff_poses, rosmsg2nparray, twist_from_vw, \
    euler_from_rosquaternion, transform2pose, diff_poses
from utils.data_container import DataContainer
from calculator.get_jacobi_matrix_mm import get_jacobi_matrix
from calculator.get_vec_redundancy import get_vec_redundancy

PARAM_NAME_MANI_REF_TOPIC = '/manipulation_reference_topic'
PARAM_NAME_BS_INPUT_TOPIC = '/blackship_input_topic'
PARAM_NAME_MICO_JOINTVEL_INPUT_TOPIC = '/mico_jointvel_input_topic'
PARAM_NAME_ROBOT_STATE_TOPIC = '/robot_state_topic'
PARAM_NAME_CTRL_PARAM_N = '/ctrlparam_N'
PARAM_NAME_CTRL_PARAM_M = '/ctrlparam_M'
PARAM_NAME_CTRL_PARAM_KPGAIN = '/ctrlparam_Kp'
PARAM_NAME_CTRL_PARAM_KREDUNDANCY = '/ctrlparam_Kredundancy'
PARAM_NAME_MICOBS_CTRL_METHOD_MAX_MANIPULABILITY = '/micobs_control_method_max_manipulability'
PARAM_NAME_MICOBS_CTRL_METHOD_FORCE_STOP = '/micobs_control_method_force_stop'
PARAM_NAME_MOBILE_WHEEL_RADIUS = '/mobile_wheel_radius'
PARAM_NAME_MOBILE_AXLE_TRACK = '/mobile_axle_track'
PARAM_NAME_MOBILE_BASE_FRAME_ID_FOR_CALC = '/mobile_base_frame_id_for_calc'
PARAM_NAME_MICO_EE_FRAME_ID_FOR_CALC = '/end_effector_frame_id_for_calc'


class ParameterManager():
    def __init__(self):
        self.N = rospy.get_param(PARAM_NAME_CTRL_PARAM_N)
        self.M = rospy.get_param(PARAM_NAME_CTRL_PARAM_M)
        self.Kpgain = rospy.get_param(PARAM_NAME_CTRL_PARAM_KPGAIN)
        self.Kredundancy = rospy.get_param(PARAM_NAME_CTRL_PARAM_KREDUNDANCY)
        self.Rw = rospy.get_param(PARAM_NAME_MOBILE_WHEEL_RADIUS)
        self.T = rospy.get_param(PARAM_NAME_MOBILE_AXLE_TRACK)


class MicoBSCtrl_ForceStopping(RobotControllerAbstract):
    def __init__(self, registered_tf_buffer=None):
        super(MicoBSCtrl_ForceStopping, self).__init__([JointVelocity, TwistStamped],
                                                       [rospy.get_param(PARAM_NAME_MICO_JOINTVEL_INPUT_TOPIC),
                                                        rospy.get_param(PARAM_NAME_BS_INPUT_TOPIC)],
                                                       registered_tf_buffer)

    def activate_controller(self):
        return True

    def update_input(self):
        return [JointVelocity(), TwistStamped(header=Header(stamp=rospy.Time.now()))]


class MicoBSCtrl_MaxManipulability(RobotControllerAbstract):
    def __init__(self, registered_tf_buffer=None):
        super(MicoBSCtrl_MaxManipulability, self).__init__([JointVelocity, TwistStamped],
                                                           [rospy.get_param(PARAM_NAME_MICO_JOINTVEL_INPUT_TOPIC),
                                                            rospy.get_param(PARAM_NAME_BS_INPUT_TOPIC)],
                                                           registered_tf_buffer)
        self._param = ParameterManager()

        self._robot_state_cur = RobotState()
        self._mani_ref = ManipulationReference()
        self._is_initialized_robot_state_cur = False
        self._is_initialized_mani_ref = False
        self._decimals = 4
        self._interval_sec = 3.0

        self._ee_state_err_container = DataContainer('eeStateErr', data_class=list,
                                                     header_list=['x', 'y', 'z', 'roll', 'pitch', 'yaw'])
        self._ee_state_ref_container = DataContainer('eeStateRef', data_class=list,
                                                     header_list=['x', 'y', 'z', 'roll', 'pitch', 'yaw'])
        self._ee_state_cur_container = DataContainer('eeStateCur', data_class=list,
                                                     header_list=['x', 'y', 'z', 'roll', 'pitch', 'yaw'])
        self._robot_vel_input_container = DataContainer('RobotVelInput', data_class=list,
                                                        header_list=['j1', 'j2', 'j3', 'j4', 'j5', 'j6', 'v', 'w'])

    def activate_controller(self):
        rospy.Subscriber(rospy.get_param(PARAM_NAME_MANI_REF_TOPIC),
                         ManipulationReference, self._mani_ref_callback)
        rospy.Subscriber(rospy.get_param(PARAM_NAME_ROBOT_STATE_TOPIC),
                         RobotState, self._robot_state_callback)
        while (self._is_initialized_robot_state_cur is False):
            rospy.loginfo('wait for initialing robot_state_cur...')
            rospy.sleep(self._interval_sec)
        return True

    def update_input(self):
        m2mm = np.matrix(np.diag(np.array([1000, 1000, 1000, 1, 1, 1])))
        mm2m = np.matrix(np.diag(np.array([0.001, 0.001, 0.001, 1, 1, 1])))
        jacobi = get_jacobi_matrix(self._robot_state_cur.arm_joint_angles,
                                   euler_from_rosquaternion(self._robot_state_cur.base_pose.orientation)[0])

        # pinv_jacobi = np.linalg.pinv(jacobi)
        pinv_jacobi = jacobi.T.dot(np.linalg.inv(jacobi.dot(jacobi.T)))

        # vec_redundancy = get_vec_redundancy(self._robot_state_cur, jacobi)
        Kpgain = np.matrix(np.diag(np.array([2, 2, 2, 0, 0, 0])))
        Kredundancy = np.matrix(np.diag(np.array([1, 1, 1, 1, 1, 1, 1, 1])))

        # rospy.loginfo('\n' + '\n'.join(['EEStateErr:\t' + ', '.join([str(d[0]) for d in self._ee_state_err.tolist()]),
        #                                 'EEVelRef:\t' + ', '.join([str(d[0]) for d in self._ee_vel_ref.tolist()])]))

        u = pinv_jacobi * m2mm * (self._ee_vel_ref - Kpgain * self._ee_state_err)
        # + (np.identity(8) - pinv_jacobi * jacobi) * Kredundancy * vec_redundancy
        u = u.round(decimals=self._decimals)

        v = (u[6, 0] + u[7, 0]) * self._param.Rw / 2
        w = (u[6, 0] - u[7, 0]) * self._param.Rw / self._param.T

        self._ee_state_err_container.write(self._ee_state_err.T.tolist()[0])
        self._ee_state_ref_container.write(self._ee_state_ref.T.tolist()[0])
        self._ee_state_cur_container.write(self._ee_state_cur.T.tolist()[0])
        self._ee_state_cur_container.write(self._ee_state_cur.T.tolist()[0])
        self._robot_vel_input_container.write(u.T.tolist()[0][:6] + [v, w])

        # FIXME u[0, 0]
        return [JointVelocity(joint1=-u[0, 0], joint2=u[1, 0], joint3=u[2, 0],
                              joint4=u[3, 0], joint5=u[4, 0], joint6=u[5, 0]),
                TwistStamped(header=Header(stamp=rospy.Time.now()),
                             twist=twist_from_vw(v, w))]

    def _mani_ref_callback(self, mani_ref):
        self._mani_ref = mani_ref
        if self._is_initialized_mani_ref is False:
            self._is_initialized_mani_ref = True

    def _robot_state_callback(self, robot_state):
        self._robot_state_cur = robot_state
        if self._is_initialized_robot_state_cur is False:
            self._is_initialized_robot_state_cur = True

    @property
    def _ee_state_ref(self):
        pose_ref = self._mani_ref.ee_pose if self._is_initialized_mani_ref else self._robot_state_cur.ee_pose
        return np.matrix(rosmsg2nparray(pose_ref)).T.round(decimals=self._decimals)

    @property
    def _ee_state_cur(self):
        return np.matrix(rosmsg2nparray(self._robot_state_cur.ee_pose)).T.round(decimals=self._decimals)

    @property
    def _ee_vel_ref(self):
        twist_ref = self._mani_ref.ee_twist if self._is_initialized_mani_ref else Twist()
        return np.matrix(rosmsg2nparray(twist_ref)).T.round(decimals=self._decimals)

    @property
    def _ee_vel_cur(self):
        return np.matrix(rosmsg2nparray(self._robot_state_cur.ee_twist)).T.round(decimals=self._decimals)

    @property
    def _ee_state_err(self):
        pose_ref = self._mani_ref.ee_pose if self._is_initialized_mani_ref else self._robot_state_cur.ee_pose
        pose_err = diff_poses(self._robot_state_cur.ee_pose, pose_ref)
        return np.matrix(rosmsg2nparray(pose_err)).T.round(decimals=self._decimals)


class MicoBSController(object):
    ctrl_method_map = {rospy.get_param(PARAM_NAME_MICOBS_CTRL_METHOD_FORCE_STOP): MicoBSCtrl_ForceStopping,
                       rospy.get_param(PARAM_NAME_MICOBS_CTRL_METHOD_MAX_MANIPULABILITY): MicoBSCtrl_MaxManipulability}

    def __init__(self, method, registered_tf_buffer=None):
        if method in self.ctrl_method_map.keys():
            self._controller = self.ctrl_method_map[method](registered_tf_buffer)
        else:
            rospy.logerr('Invalid MICOBSControlMethod is selected!')

    @property
    def controller(self):
        return self._controller
