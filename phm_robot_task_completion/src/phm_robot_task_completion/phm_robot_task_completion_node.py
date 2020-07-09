#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""

    PHM Robot Task Completion Node

"""


import rospy
from std_msgs.msg import String
from phm_msgs.msg import Potc


class RobotTaskCompletionNode:
    """
        Robot Task Completion Node

        Subscriber:
            Topic name = /gui_actual_potc
            Message type = String

            Topic name = /gui_predict_potc
            Message type = String

        Publishler:
            Topic nane = /phm_potc
            Message type = Potc

        Rate = 1
    """
    def __init__(self):
        self.phm_gui_control = False
        self.phm_gui_actual_potc = ""
        self.phm_gui_predict_potc = ""


    def main_func(self):
        """
            Robot Task Completion Node Main Function
        """
        rospy.Subscriber('/gui_actual_potc', String, self.phm_gui_actual_potc_callback_func)
        rospy.Subscriber('/gui_predict_potc', String, self.phm_gui_predict_potc_callback_func)
        publisher_phm_potc = rospy.Publisher('/phm_potc', Potc, queue_size=10)

        rate = rospy.Rate(1)

        phm_potc = Potc()

        while not rospy.is_shutdown():
            if self.phm_gui_control:
                ros_time = rospy.get_rostime()
                phm_potc = self.set_potc_func(self.phm_gui_actual_potc, self.phm_gui_predict_potc, ros_time)
                publisher_phm_potc.publish(phm_potc)

            rate.sleep()

    def phm_gui_actual_potc_callback_func(self, phm_msg):
        """
            Callback function of /gui_actual_potc
        """
        self.phm_gui_actual_potc = dict(eval(phm_msg.data))
        self.phm_gui_control = True


    def phm_gui_predict_potc_callback_func(self, phm_msg):
        """
            Callback function of /gui_predict_potc
        """
        self.phm_gui_predict_potc = dict(eval(phm_msg.data))
        self.phm_gui_control = True

    @classmethod
    def set_potc_func(cls, phm_gui_actual_potc, phm_gui_predict_potc, ros_time):
        """
            Message function of the /phm_potc to be published
        """
        phm_potc = Potc()

        phm_potc.stamp = ros_time

        # Actual POTC
        if phm_gui_actual_potc != "":
            phm_potc.actual_potc.potc_nominal_value = float(phm_gui_actual_potc["Nominal"]["POTC"])
            phm_potc.actual_potc.potc_sensor_based_value = float(phm_gui_actual_potc["Sensor Based"]["POTC"])
            phm_potc.actual_potc.potc_time = float(phm_gui_actual_potc["Nominal"]["Time"])
            phm_potc.actual_potc.potc_distance = float(phm_gui_actual_potc["Nominal"]["Distance"])

        else:
            phm_potc.actual_potc.potc_nominal_value = float(1.0)
            phm_potc.actual_potc.potc_sensor_based_value = float(1.0)
            phm_potc.actual_potc.potc_time = float(0.0)
            phm_potc.actual_potc.potc_distance = float(0.0)

        # Predict POTC
        if phm_gui_predict_potc != "":
            phm_potc.predict_potc.potc_nominal_value = float(phm_gui_predict_potc["Nominal"]["POTC"])
            phm_potc.predict_potc.potc_sensor_based_value = float(phm_gui_predict_potc["Sensor Based"]["POTC"])
            phm_potc.predict_potc.potc_time = float(phm_gui_predict_potc["Nominal"]["Time"])
            phm_potc.predict_potc.potc_distance = float(phm_gui_predict_potc["Nominal"]["Distance"])

        else:
            phm_potc.predict_potc.potc_nominal_value = float(1.0)
            phm_potc.predict_potc.potc_sensor_based_value = float(1.0)
            phm_potc.predict_potc.potc_time = float(0.0)
            phm_potc.predict_potc.potc_distance = float(0.0)

        return phm_potc


if __name__ == '__main__':
    rospy.init_node('phm_robot_task_completion_node')
    POTC_CLASS = RobotTaskCompletionNode()
    POTC_CLASS.main_func()
