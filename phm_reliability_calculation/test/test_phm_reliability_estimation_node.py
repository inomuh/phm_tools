#!/usr/bin/env python
# -*- coding: utf-8 -*-

import unittest
import math
import rosunit
from phm_reliability_calculation.phm_reliability_estimation_node import ReliabilityEstimationNode
from phm_msgs.msg import Reliability

PKG = 'phm_reliability_calculation'
NAME = 'test_phm_reliability_estimation_node'


class TestReliabilityEstimationNode(unittest.TestCase):
    ren = ReliabilityEstimationNode()
    reliability_dict = dict({'System': {'Power': {'Reliability': {'Nominal': 0.9999992289110848}}, 'Mobility': {'Reliability': {'Nominal': 0.9999999213314239}}, 'Communication': {'Reliability': {'Nominal': 0.9999999822923761}}, 'Reliability': {'Nominal': 0.999999118612577, 'Sensor Based': 0.999999118612577}, 'Computation': {'Reliability': {'Nominal': 0.9999999998083333}}, 'Sensing': {'Reliability': {'Nominal': 0.9999999862692709}}}})


    def test_set_reliability_func(self):
        phm_reliability = Reliability()
        phm_reliability = self.ren.set_reliability_func(self.reliability_dict, 5)

        system_value = 0.999999118613
        system_sensor_based_value = 0.999999118613
        module_names = ["Power", "Mobility", "Communication", "Computation", "Sensing"]
        module_values = [0.9999992289110848, 0.9999999213314239, 0.9999999822923761, 0.9999999998083333, 0.9999999862692709]
        module_sensor_based_values = [0.0, 0.0, 0.0, 0.0, 0.0]


        self.assertAlmostEqual(phm_reliability.system_value, system_value, 5)
        self.assertAlmostEqual(phm_reliability.system_sensor_based_value, system_sensor_based_value, 5)
        self.assertListEqual(phm_reliability.module_names, module_names)
        self.assertListEqual(phm_reliability.module_values, module_values)
        self.assertListEqual(phm_reliability.module_sensor_based_values, module_sensor_based_values)


if __name__ == '__main__':
    rosunit.unitrun(PKG, NAME, TestReliabilityEstimationNode, sysargs = "--cov", coverage_packages=[str(PKG)])
   