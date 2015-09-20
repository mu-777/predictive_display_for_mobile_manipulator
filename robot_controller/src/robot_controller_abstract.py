#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import common

from abc import ABCMeta, abstractmethod


class RobotControllerAbstract(object):
    __metaclass__ = ABCMeta
    """
    制御入力をpublishするための上位クラス．
    これを親クラスにすることで，activate, update, publish_inputの実装が強制されるので，実行するところを統一できる．

    """

    def __init__(self, InputClasses, input_topic_names, registered_tf_buffer=None):
        if registered_tf_buffer is not None:
            self._tf_buffer = registered_tf_buffer
        else:
            self._tf_buffer = tf2_ros.Buffer()
            tf2_ros.TransformListener(self._tf_buffer)

        self._inputs, self._input_publishers = [], []
        data_classes = InputClasses if isinstance(InputClasses, list) else [InputClasses]
        topic_names = input_topic_names if isinstance(input_topic_names, list) else [input_topic_names]
        for data_class, topic_name in zip(data_classes, topic_names):
            self._inputs.append(data_class())
            self._input_publishers.append(rospy.Publisher(topic_name, data_class, queue_size=1))
        self.is_activated = False

    @abstractmethod
    def activate_controller(self):
        return True

    @abstractmethod
    def update_input(self):
        updated_inputs = [input.__class__() for input in self._inputs]
        return updated_inputs

    def activate(self):
        self.is_activated = self.activate_controller()
        return self

    def update(self):
        updated_input = self.update_input()
        self._inputs = updated_input if isinstance(updated_input, list) else [updated_input]
        return self

    def publish_input(self):
        for input, publisher in zip(self._inputs, self._input_publishers):
            publisher.publish(input)
        return self
