#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import tf2_ros
import common

from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TwistStamped, Vector3, Quaternion
from utils.util_functions import init_tf_stamped, wait_for_tf, update_tf, euler_from_rosquaternion, \
    rosquaternion_from_euler, add_vectors
from utils.my_quaternion import multiply_ros_quaternion
from utils.loop_time_manager import LoopTimeManager


REMAPPABLE_NODE_NAME = 'mobile_manipulator_mock'
PARAM_NAME_BS_INPUT_TOPIC = '/blackship_input_topic'
PARAM_NAME_SYS_ORIGIN_FRAME_ID = '/system_origin_frame_id'
PARAM_NAME_MOBILE_BASE_FRAME_ID_FOR_CALC = '/mobile_base_frame_id_for_calc'
PARAM_NAME_MICO_EE_FRAME_ID_FOR_CALC = '/end_effector_frame_id_for_calc'
PARAM_NAME_LOCALIZED_POINT_ORIGIN_FRAME_ID = '/localized_point_frame_origin_id'
PARAM_NAME_LOCALIZED_POINT_ORIGIN_FRAME_ID_FOR_CALC = '/localized_point_frame_origin_id_for_calc'
PARAM_NAME_LOCALIZED_POINT_FRAME_ID = '/localized_point_frame_id'
PARAM_NAME_LOCALIZED_POINT_FRAME_ID_FOR_CALC = '/localized_point_frame_id_for_calc'


class SLAMMockup(object):
    """
    Subscribe
        TwistStamped(blackship_input)
        tf(mobile_base_frame_id_for_calc -> localized_point_frame_id_for_calc)
    Publish
        tf(localized_point_origin -> localized_point_frame_id)
    """

    def __init__(self):
        self._tf_broadcaster = tf2_ros.TransformBroadcaster()
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self._localized_point_tf_from_localized_origin = init_tf_stamped(
            rospy.get_param(PARAM_NAME_LOCALIZED_POINT_ORIGIN_FRAME_ID),
            rospy.get_param(PARAM_NAME_LOCALIZED_POINT_FRAME_ID))

        self._mobile_base_tf_for_calc_from_localized_origin = init_tf_stamped(
            rospy.get_param(PARAM_NAME_LOCALIZED_POINT_ORIGIN_FRAME_ID_FOR_CALC),
            rospy.get_param(PARAM_NAME_MOBILE_BASE_FRAME_ID_FOR_CALC))

        self._localized_point_tf_for_calc_from_mobile_base = init_tf_stamped(
            rospy.get_param(PARAM_NAME_MOBILE_BASE_FRAME_ID_FOR_CALC),
            rospy.get_param(PARAM_NAME_LOCALIZED_POINT_FRAME_ID_FOR_CALC))

        self._input = TwistStamped()
        self.timer = LoopTimeManager()

    def activate(self):
        rospy.Subscriber(rospy.get_param(PARAM_NAME_BS_INPUT_TOPIC),
                         TwistStamped, self._bs_input_callback)

        origin_to_localized_origin = wait_for_tf(self._tf_buffer,
                                                 rospy.get_param(PARAM_NAME_SYS_ORIGIN_FRAME_ID),
                                                 rospy.get_param(PARAM_NAME_LOCALIZED_POINT_ORIGIN_FRAME_ID))
        origin_to_localized_origin.child_frame_id = rospy.get_param(PARAM_NAME_LOCALIZED_POINT_ORIGIN_FRAME_ID_FOR_CALC)
        rospy.Publisher('/tf_static', TFMessage,
                        queue_size=100, latch=True).publish([origin_to_localized_origin])

        self._localized_point_tf_for_calc_from_mobile_base = wait_for_tf(self._tf_buffer,
                                                                         self._localized_point_tf_for_calc_from_mobile_base.header.frame_id,
                                                                         self._localized_point_tf_for_calc_from_mobile_base.child_frame_id,
                                                                         interval_sec=5.0)
        rospy.loginfo('slam_mock activated')
        return self

    # 厳密にはこの更新はループ的によろしくない．入ってきたvelで今まで動いていました，となっているので．
    # ↑直した．けどあまりいい方法とはいえない．．．
    def _bs_input_callback(self, bs_input):
        dt = self.timer.dt_sec
        vdt = self._input.twist.linear.x * dt
        wdt = self._input.twist.angular.z * dt
        theta = euler_from_rosquaternion(self._mobile_base_tf_for_calc_from_localized_origin.transform.rotation)[0]
        translation = add_vectors([self._mobile_base_tf_for_calc_from_localized_origin.transform.translation,
                                   Vector3(x=vdt * np.cos(theta), y=vdt * np.sin(theta))])
        rotation = multiply_ros_quaternion([self._mobile_base_tf_for_calc_from_localized_origin.transform.rotation,
                                            Quaternion(z=np.sin(wdt / 2.0), w=np.cos(wdt / 2.0))])

        self._mobile_base_tf_for_calc_from_localized_origin.header.stamp = rospy.Time.now()
        self._mobile_base_tf_for_calc_from_localized_origin.transform.translation = translation
        self._mobile_base_tf_for_calc_from_localized_origin.transform.rotation = rotation

        self._localized_point_tf_for_calc_from_mobile_base = update_tf(self._tf_buffer,
                                                                       self._localized_point_tf_for_calc_from_mobile_base)
        self._input = bs_input
        self.timer.update()

    def publish(self):
        stamp = self._mobile_base_tf_for_calc_from_localized_origin.header.stamp
        translation = add_vectors([self._mobile_base_tf_for_calc_from_localized_origin.transform.translation,
                                   self._localized_point_tf_for_calc_from_mobile_base.transform.translation])
        rotation = multiply_ros_quaternion([self._mobile_base_tf_for_calc_from_localized_origin.transform.rotation,
                                            self._localized_point_tf_for_calc_from_mobile_base.transform.rotation])

        self._localized_point_tf_from_localized_origin.header.stamp = rospy.Time.now()
        self._localized_point_tf_from_localized_origin.transform.translation = translation
        self._localized_point_tf_from_localized_origin.transform.rotation = rotation

        self._tf_broadcaster.sendTransform([self._localized_point_tf_from_localized_origin,
                                            self._mobile_base_tf_for_calc_from_localized_origin])


# --------------------------------------------
if __name__ == '__main__':
    rospy.init_node(REMAPPABLE_NODE_NAME, anonymous=True)
    rate_mgr = rospy.Rate(100)  # Hz

    slam_mock = SLAMMockup().activate()

    while not rospy.is_shutdown():
        slam_mock.publish()
        rate_mgr.sleep()
