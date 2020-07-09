#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""

    PHM Hazard Rate Calculation Node

"""

import rospy
from std_msgs.msg import String
from phm_msgs.msg import HazardRate


class HazardRateCalculationNode:
    """
        Hazard Rate Calculation Node

        Subscriber:
            Topic name = /gui_hazard_rate
            Message type = String

        Publishler:
            Topic nane = /phm_hazard_rate
            Message type = HazardRate

        Rate = 1
    """
    def __init__(self):
        self.phm_gui_control = False
        self.phm_gui_hazard_rate = ""


    def main_func(self):
        """
            Hazard Rate Calculation Node Main Function
        """
        rospy.Subscriber('/gui_hazard_rate', String, self.phm_gui_hazard_rate_callback_func)
        publisher_phm_hazard_rate = rospy.Publisher('/phm_hazard_rate', HazardRate, queue_size=10)

        rate = rospy.Rate(1)

        phm_hazard_rate = HazardRate()

        while not rospy.is_shutdown():
            if self.phm_gui_control:
                ros_time = rospy.get_rostime()
                phm_hazard_rate = self.set_hazard_rate_func(self.phm_gui_hazard_rate, ros_time)
                publisher_phm_hazard_rate.publish(phm_hazard_rate)

            rate.sleep()


    def phm_gui_hazard_rate_callback_func(self, phm_msg):
        """
            Callback function of /gui_hazard_rate
        """
        self.phm_gui_hazard_rate = dict(eval(phm_msg.data))
        self.phm_gui_control = True

    @classmethod
    def set_hazard_rate_func(cls, hazard_rate_dict, ros_time):
        """
            Message function of the /phm_hazard_rate to be published
        """
        phm_hazard_rate = HazardRate()

        module_names = list(hazard_rate_dict["System"].keys())
        module_names.remove("Failure Rate")

        system_hazard_rate_keys = list(hazard_rate_dict["System"]["Failure Rate"].keys())
        system_hazard_rate_keys.remove("Nominal")

        phm_hazard_rate.stamp = ros_time
        phm_hazard_rate.system_value = float(hazard_rate_dict["System"]["Failure Rate"]["Nominal"])

        if system_hazard_rate_keys:
            phm_hazard_rate.system_sensor_based_value = float(hazard_rate_dict["System"]["Failure Rate"]["Sensor Based"])

        if module_names:
            module_values = list()
            module_sensor_based_values = list()
            phm_hazard_rate.module_names = list(module_names)

            for module in module_names:
                module_hazard_rate_keys = list(hazard_rate_dict["System"][str(module)]["Failure Rate"].keys())
                module_hazard_rate_keys.remove("Nominal")

                current_module_value = float(hazard_rate_dict["System"][str(module)]["Failure Rate"]["Nominal"])
                module_values.append(current_module_value)

                if module_hazard_rate_keys:
                    current_module_sensor_based_value = float(hazard_rate_dict["System"][str(module)]["Failure Rate"]["Sensor Based"])
                    module_sensor_based_values.append(current_module_sensor_based_value)

                else:
                    module_sensor_based_values.append(float(0.0))

            phm_hazard_rate.module_values = module_values
            phm_hazard_rate.module_sensor_based_values = module_sensor_based_values

        return phm_hazard_rate


if __name__ == '__main__':
    rospy.init_node('phm_hazard_rate_calculation_node')

    HR_CLASS = HazardRateCalculationNode()
    HR_CLASS.main_func()
