#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import numpy as np
import common
from common.msg import OperatorState
from blackship_controller import BlackshipController
from mico_controller import MicoController
from mico_bs_controller import MicoBSController

REMAPPABLE_NODE_NAME = 'sample_to_use_nav_cmd_gen'

PARAM_NAME_OPE_STATE_TOPIC = '/operator_state_topic'
PARAM_NAME_MICO_CONTROL_METHOD = '/mico_control_method'
PARAM_NAME_BS_CONTROL_METHOD = '/blackship_control_method'
PARAM_NAME_MICOBS_CONTROL_METHOD = '/micobs_control_method'
PARAM_NAME_MICOBS_CTRL_METHOD_FORCE_STOP = '/micobs_control_method_force_stop'


class MobileManipulatorControlManager(object):
    def __init__(self):
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._ope_state = OperatorState()
        self._ope_state_subscriber = rospy.Subscriber(rospy.get_param(PARAM_NAME_OPE_STATE_TOPIC),
                                                      OperatorState,
                                                      self._ope_state_callback)

        self._bs_controller = BlackshipController(rospy.get_param(PARAM_NAME_BS_CONTROL_METHOD),
                                                  self._tf_buffer).controller
        self._mico_controller = MicoController(rospy.get_param(PARAM_NAME_MICO_CONTROL_METHOD),
                                               self._tf_buffer).controller
        self._mico_bs_controller = MicoBSController(rospy.get_param(PARAM_NAME_MICOBS_CONTROL_METHOD),
                                                    self._tf_buffer).controller
        self._mico_bs_stop_controller = MicoBSController(rospy.get_param(PARAM_NAME_MICOBS_CTRL_METHOD_FORCE_STOP),
                                                         self._tf_buffer).controller
        self.is_activated = False

    def _ope_state_callback(self, ope_state):
        self._ope_state = ope_state

    def activate(self):
        is_successes = [self._bs_controller.activate().is_activated,
                        self._mico_controller.activate().is_activated,
                        self._mico_bs_controller.activate().is_activated]
        self.is_activated = all(is_successes)
        rospy.loginfo('activated')
        return self

    def publish_input(self):
        if not self.is_activated:
            self.activate()

        if self._ope_state.is_manipulating:
            if self._mico_controller.is_reachable:
                self._mico_controller.update().publish_input()
            else:
                self._mico_bs_controller.update().publish_input()
        if self._ope_state.is_walking:
            self._bs_controller.update().publish_input()

        if self._ope_state.is_stopping:
            self._mico_bs_stop_controller.update().publish_input()
            #TODO アームは単体で動かして周り見回す

        return self


# --------------------------------------------
if __name__ == '__main__':
    rospy.init_node(REMAPPABLE_NODE_NAME, anonymous=True)
    rate_mgr = rospy.Rate(100)  # Hz

    mm_controller = MobileManipulatorControlManager().activate()

    while not rospy.is_shutdown():
        mm_controller.publish_input()
        rate_mgr.sleep()
