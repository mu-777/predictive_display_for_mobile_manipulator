#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import common
from numpy import sin, cos, pi, matrix, deg2rad

from std_msgs.msg import Header
from geometry_msgs.msg import Transform, TransformStamped, Vector3, Quaternion
from sensor_msgs.msg import JointState
from jaco_msgs.msg import JointAngles
from tf.transformations import quaternion_from_matrix
from utils.mico_utils import dh_to_mico


class TransMatManager(object):
    def __init__(self):
        self.update(0, 0, 0, 0, 0, 0, 0)

    def update(self, th1, th2, th3, th4, th5, th6, thb):
        self.trans_mat_list = self._update_mat_list(th1, th2, th3, th4, th5, th6, thb)
        return self

    def _update_mat_list(self, th1, th2, th3, th4, th5, th6, thb):
        xb, yb, lb, hb = 0, 0, 0, 0
        # xb, yb, lb, hb = 0, 0, 0.15, (0.16 + 0.15 / 2.0 + 0.06 + 0.003 / 2.0)
        # mico_arm.launch
        d0, d1, d2, d3_offset, d3, d4, d5, d6 = 0.1544, -0.1181, 0.2900, -0.0070, 0.1233, 0.0741, 0.0741, -0.1600
        # mico_arm.xacro
        # d0, d1, d2, d3_offset, d3, d4, d5, d6 = 0.1535, -0.1185, 0.2900, -0.00845, 0.123, 0.0741, 0.0741, 0.1600
        j5_bend, j6_bend = deg2rad(-60), deg2rad(60)

        th1, th2, th3, th4, th5, th6 = dh_to_mico(th1, th2, th3, th4, th5, th6)
        return [matrix([[cos(thb), -sin(thb), 0, xb + lb * cos(thb)],
                        [sin(thb), cos(thb), 0, yb + lb * sin(thb)],
                        [0, 0, 1, hb],
                        [0, 0, 0, 1]]),

                matrix([[cos(th1), -sin(th1), 0, 0],
                        [-sin(th1), -cos(th1), 0, 0],
                        [0, 0, -1, d0],
                        [0, 0, 0, 1]]),

                matrix([[sin(th2), cos(th2), 0, 0],
                        [0, 0, 1, 0],
                        [cos(th2), -sin(th2), 0, d1],
                        [0, 0, 0, 1]]),

                matrix([[-cos(th3), sin(th3), 0, d2],
                        [sin(th3), cos(th3), 0, 0],
                        [0, 0, -1, 0],
                        [0, 0, 0, 1]]),

                matrix([[1, 0, 0, 0],
                        [0, 1, 0, 0],
                        [0, 0, 1, d3_offset],
                        [0, 0, 0, 1]]),

                matrix([[0, 0, -1, d3],
                        [sin(th4), cos(th4), 0, 0],
                        [cos(th4), -sin(th4), 0, 0],
                        [0, 0, 0, 1]]),

                matrix([[cos(j5_bend) * cos(th5), cos(j5_bend) * -sin(th5), sin(j5_bend), cos(-j5_bend) * d4],
                        [sin(th5), cos(th5), 0, 0],
                        [-sin(j5_bend) * cos(th5), sin(j5_bend) * sin(th5), cos(j5_bend), -sin(-j5_bend) * d4],
                        [0, 0, 0, 1]]),

                matrix([[cos(j6_bend) * cos(th6), cos(j6_bend) * -sin(th6), sin(j6_bend), -cos(j6_bend) * d5],
                        [sin(th6), cos(th6), 0, 0],
                        [-sin(j6_bend) * cos(th6), sin(j6_bend) * sin(th6), cos(j6_bend), -sin(j6_bend) * d5],
                        [0, 0, 0, 1]]),

                matrix([[-1, 0, 0, 0],
                        [0, 1, 0, 0],
                        [0, 0, -1, d6],
                        [0, 0, 0, 1]])]


def tf_from_transmat(mat):
    q = quaternion_from_matrix(mat)
    return Transform(translation=Vector3(x=mat[0, 3], y=mat[1, 3], z=mat[2, 3]),
                     rotation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))


class TransformChecker(object):
    def __init__(self):
        self._tf_broadcaster = tf2_ros.TransformBroadcaster()
        self._subscriber = rospy.Subscriber('joint_states', JointState, self._callback)
        self.trans_mgr = TransMatManager()

    def _callback(self, joint_states):
        names = joint_states.name
        angles = joint_states.position
        mat_list = self.trans_mgr.update(angles[names.index('mico_joint_1')],
                                         angles[names.index('mico_joint_2')],
                                         angles[names.index('mico_joint_3')],
                                         angles[names.index('mico_joint_4')],
                                         angles[names.index('mico_joint_5')],
                                         angles[names.index('mico_joint_6')], 0.0).trans_mat_list

        tf_list = [
            # TransformStamped(header=Header(frame_id='blackship_base_for_calc',
            # stamp=rospy.Time.now()),
            # child_frame_id='_mico_api_origin_',
            # transform=tf_from_transmat(mat_list[0])),
            TransformStamped(header=Header(frame_id='_mico_root_',
                                           stamp=rospy.Time.now()),
                             child_frame_id='_mico_link_1_',
                             transform=tf_from_transmat(mat_list[1])),
            TransformStamped(header=Header(frame_id='_mico_link_1_',
                                           stamp=rospy.Time.now()),
                             child_frame_id='_mico_link_2_',
                             transform=tf_from_transmat(mat_list[2])),
            TransformStamped(header=Header(frame_id='_mico_link_2_',
                                           stamp=rospy.Time.now()),
                             child_frame_id='_mico_link_3_',
                             transform=tf_from_transmat(mat_list[3])),
            TransformStamped(header=Header(frame_id='_mico_link_3_',
                                           stamp=rospy.Time.now()),
                             child_frame_id='_mico_link_4_',
                             transform=tf_from_transmat(mat_list[4] * mat_list[5])),
            TransformStamped(header=Header(frame_id='_mico_link_4_',
                                           stamp=rospy.Time.now()),
                             child_frame_id='_mico_link_5_',
                             transform=tf_from_transmat(mat_list[6])),
            TransformStamped(header=Header(frame_id='_mico_link_5_',
                                           stamp=rospy.Time.now()),
                             child_frame_id='_mico_link_hand_',
                             transform=tf_from_transmat(mat_list[7])),
            TransformStamped(header=Header(frame_id='_mico_link_hand_',
                                           stamp=rospy.Time.now()),
                             child_frame_id='_mico_end_effector_',
                             transform=tf_from_transmat(mat_list[8]))]

        self._tf_broadcaster.sendTransform(tf_list)

# --------------------------------------------
if __name__ == '__main__':
    rospy.init_node('tf_test', anonymous=True)
    t = TransformChecker()
    rospy.spin()
