#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
    PHM Ros Sensor Class
"""

import rospy
from threading import Thread

import roslib.message
import roslib.names


class ROSSensor:
    """
        Ros Sensor Class
    """
    def __init__(self, topic):
        self.name = topic
        self.error = None
        self.data = None
        self.topic_message_type = None

        self.thread = Thread(target=self.main_func, args=(self.name,))
        self.thread.start()


    def main_func(self, topic):
        """
            Ros Sensor Main Function
        """
        try:
            topic_type, real_topic, fields = self.get_topic_type(topic)

            if topic_type is not None:
                data_class = roslib.message.get_message_class(topic_type)
                self.topic_message_type = str(data_class)

                rospy.Subscriber(real_topic, data_class, self._ros_cb)
            else:
                print("Can not resolve topic type of %s" % topic)

        except ValueError as err:
            print("ROSSensor Class Error!\n")
            print(err)


    def _ros_cb(self, msg):
        """
            Ros Sensor Callback Function
        """
        try:
            if "sensor_msgs.msg._Temperature.Temperature" in self.topic_message_type:
                self.data = msg.temperature

            elif "agv_msgs.msg._CurrentData.CurrentData" in self.topic_message_type:
                self.data = msg.current_data

            elif "agv_msgs.msg._VoltageData.VoltageData" in self.topic_message_type:
                self.data = msg.voltage_data

            elif "agv_msgs.msg._PowerData.PowerData" in self.topic_message_type:
                self.data = msg.power_data

            elif "std_msgs" in self.topic_message_type:
                self.data = msg.data

            else:
                self.data = None

        except AttributeError as err:
            print("Invalid topic spec [%s]: %s" % (self.name, str(err)))


    def stop_thread_func(self):
        """
            Ros Sensor Stop Thread Function
        """
        self.thread.join()
        self.thread.should_abort_immediately = True

    @classmethod
    def _get_topic_type(cls, topic):
        """
            Private Get Topic Type Function
        """
        try:
            val = rospy.get_published_topics()
        except Exception as err:
            print("unable to get list of topics from master")
            print(err)

        matches = [(tpc, t_type) for tpc, t_type in val if tpc == topic or topic.startswith(tpc + '/')]

        if matches:
            tpc, t_type = matches[0]
            if t_type == roslib.names.ANYTYPE:
                return None, None, None
            if t_type == topic:
                return t_type, None
            return t_type, tpc, topic[len(tpc):]
        else:
            return None, None, None


    def get_topic_type(self, topic):
        """
            Get Topic Type Function
        """
        try:
            topic_type, real_topic, rest = self._get_topic_type(topic)

            if topic_type:
                return topic_type, real_topic, rest
            else:
                return None, None, None

        except Exception as err:
            print("get_topic_type Error!")
            print(err)
