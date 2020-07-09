#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""

    PHM POTC Subscriber Class

"""

import rospy
from std_msgs.msg import String


class POTCSubscriber:
    """
        POTC Subscriber Class

        Subscriber:
            Topic name = /task_time
            Message type = String

            Topic name = /task_position
            Message type = String

            Topic name = /robot_task_list
            Message type = String
    """
    def __init__(self):
        self.task_time = 1.0
        self.task_position = list()
        self.robot_task_list = list()

        self.task_time_control = False
        self.task_position_control = False
        self.robot_task_list_control = False


    def main_func(self):
        """
            POTC Subscriber Class Main Function
        """
        rospy.Subscriber('/task_time', String, self.task_time_clbk)
        rospy.Subscriber('/task_position', String, self.task_position_clbk)
        rospy.Subscriber('/robot_task_list', String, self.robot_task_list_clbk)


    def task_time_clbk(self, msg):
        """
            Callback function of /task_time
        """
        self.task_time = list(eval(msg.data))
        self.task_time_control = True


    def task_position_clbk(self, msg):
        """
            Callback function of /task_position
        """
        self.task_position = list(eval(msg.data))
        self.task_position_control = True


    def robot_task_list_clbk(self, msg):
        """
            Callback function of /robot_task_list
        """
        self.robot_task_list = list(eval(msg.data))
        self.robot_task_list_control = True
