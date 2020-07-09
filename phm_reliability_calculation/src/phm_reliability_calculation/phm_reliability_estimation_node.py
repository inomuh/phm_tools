#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""

    PHM Reliability Estimation Node

"""

import rospy
from std_msgs.msg import String
from phm_msgs.msg import Reliability


class ReliabilityEstimationNode:
    """
        Reliability Estimation Node

        Subscriber:
            Topic name = /gui_reliability
            Message type = String

        Publishler:
            Topic nane = /phm_reliability
            Message type = Reliability

        Rate = 1
    """
    def __init__(self):
        self.phm_gui_control = False
        self.phm_gui_reliability = ""


    def main_func(self):
        """
            Reliability Estimation Node Main Function
        """
        rospy.Subscriber('/gui_reliability', String, self.phm_gui_reliability_callback_func)
        publisher_phm_reliability = rospy.Publisher('/phm_reliability', Reliability, queue_size=10)

        rate = rospy.Rate(1)

        phm_reliability = Reliability()

        while not rospy.is_shutdown():
            if self.phm_gui_control:
                ros_time = rospy.get_rostime()
                phm_reliability = self.set_reliability_func(self.phm_gui_reliability, ros_time)
                publisher_phm_reliability.publish(phm_reliability)

            rate.sleep()


    def phm_gui_reliability_callback_func(self, phm_msg):
        """
            Callback function of /gui_reliability
        """
        self.phm_gui_reliability = dict(eval(phm_msg.data))
        self.phm_gui_control = True

    @classmethod
    def set_reliability_func(cls, reliability_dict, ros_time):
        """
            Message function of the /phm_reliability to be published
        """
        phm_reliability = Reliability()

        module_names = list(reliability_dict["System"].keys())
        module_names.remove("Reliability")

        system_reliability_keys = list(reliability_dict["System"]["Reliability"].keys())
        system_reliability_keys.remove("Nominal")

        phm_reliability.stamp = ros_time
        phm_reliability.system_value = float(reliability_dict["System"]["Reliability"]["Nominal"])

        if system_reliability_keys:
            phm_reliability.system_sensor_based_value = float(reliability_dict["System"]["Reliability"]["Sensor Based"])

        if module_names:
            module_values = list()
            module_sensor_based_values = list()
            phm_reliability.module_names = list(module_names)

            for module in module_names:
                module_reliability_keys = list(reliability_dict["System"][str(module)]["Reliability"].keys())
                module_reliability_keys.remove("Nominal")

                current_module_value = float(reliability_dict["System"][str(module)]["Reliability"]["Nominal"])
                module_values.append(current_module_value)

                if module_reliability_keys:
                    current_module_sensor_based_value = float(reliability_dict["System"][str(module)]["Reliability"]["Sensor Based"])
                    module_sensor_based_values.append(current_module_sensor_based_value)

                else:
                    module_sensor_based_values.append(float(0.0))

            phm_reliability.module_values = module_values
            phm_reliability.module_sensor_based_values = module_sensor_based_values

        return phm_reliability


if __name__ == '__main__':
    rospy.init_node('phm_reliability_estimation_node')

    RELIABILITY_CLASS = ReliabilityEstimationNode()
    RELIABILITY_CLASS.main_func()
