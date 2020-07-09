#!/usr/bin/env python
# -*- coding: utf-8 -*-

import unittest
import math
import rosunit
from phm_robot_task_completion.phm_robot_task_completion_node import RobotTaskCompletionNode
from phm_msgs.msg import Potc

PKG = 'phm_robot_task_completion'
NAME = 'test_phm_robot_task_completion_node'


class TestRobotTaskCompletionNode(unittest.TestCase):
    rtcn = RobotTaskCompletionNode()
    actual_potc_dict = dict({'Nominal': {'Distance': 0.019216928763702394, 'POTC': 0.9999999622234921, 'Time': 0.03545456144544813}, 'Sensor Based': {'Distance': 0.019216928763702394, 'POTC': 0.9999999622234921, 'Time': 0.03545456144544813}})
    predict_potc_dict = dict({'Nominal': {'Distance': 0.018785978314649166, 'POTC': 0.9999999655204828, 'Time': 0.01687624037496195}, 'Sensor Based': {'Distance': 0.018785978314649166, 'POTC': 0.9999999655204828, 'Time': 0.01687624037496195}})

    def test_1_set_reliability_func(self):
        phm_potc = Potc()
        phm_potc = self.rtcn.set_potc_func(self.actual_potc_dict, self.predict_potc_dict, 5)

        ap_potc_nominal_value = 0.999999962223
        ap_potc_sensor_based_value = 0.999999962223
        ap_potc_time = 0.0354545614454
        ap_potc_distance = 0.0192169287637

        pp_potc_nominal_value = 0.99999996552
        pp_potc_sensor_based_value = 0.99999996552
        pp_potc_time = 0.016876240375
        pp_potc_distance = 0.0187859783146


        self.assertAlmostEqual(phm_potc.actual_potc.potc_nominal_value, ap_potc_nominal_value, 5)
        self.assertAlmostEqual(phm_potc.actual_potc.potc_sensor_based_value, ap_potc_sensor_based_value, 5)
        self.assertAlmostEqual(phm_potc.actual_potc.potc_time, ap_potc_time, 5)
        self.assertAlmostEqual(phm_potc.actual_potc.potc_distance, ap_potc_distance, 5)

        self.assertAlmostEqual(phm_potc.predict_potc.potc_nominal_value, pp_potc_nominal_value, 5)
        self.assertAlmostEqual(phm_potc.predict_potc.potc_sensor_based_value, pp_potc_sensor_based_value, 5)
        self.assertAlmostEqual(phm_potc.predict_potc.potc_time, pp_potc_time, 5)
        self.assertAlmostEqual(phm_potc.predict_potc.potc_distance, pp_potc_distance, 5)


    def test_2_set_reliability_func(self):
        phm_potc = Potc()
        empty_dict = ""
        phm_potc = self.rtcn.set_potc_func(empty_dict, empty_dict, 5)

        ap_potc_nominal_value = 1.0
        ap_potc_sensor_based_value = 1.0
        ap_potc_time = 0.0
        ap_potc_distance = 0.0

        pp_potc_nominal_value = 1.0
        pp_potc_sensor_based_value = 1.0
        pp_potc_time = 0.0
        pp_potc_distance = 0.0


        self.assertAlmostEqual(phm_potc.actual_potc.potc_nominal_value, ap_potc_nominal_value, 5)
        self.assertAlmostEqual(phm_potc.actual_potc.potc_sensor_based_value, ap_potc_sensor_based_value, 5)
        self.assertAlmostEqual(phm_potc.actual_potc.potc_time, ap_potc_time, 5)
        self.assertAlmostEqual(phm_potc.actual_potc.potc_distance, ap_potc_distance, 5)

        self.assertAlmostEqual(phm_potc.predict_potc.potc_nominal_value, pp_potc_nominal_value, 5)
        self.assertAlmostEqual(phm_potc.predict_potc.potc_sensor_based_value, pp_potc_sensor_based_value, 5)
        self.assertAlmostEqual(phm_potc.predict_potc.potc_time, pp_potc_time, 5)
        self.assertAlmostEqual(phm_potc.predict_potc.potc_distance, pp_potc_distance, 5)


if __name__ == '__main__':
    rosunit.unitrun(PKG, NAME, TestRobotTaskCompletionNode, sysargs = "--cov", coverage_packages=[str(PKG)])
   