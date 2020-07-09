#!/usr/bin/env python
# -*- coding: utf-8 -*-

import unittest
import math
import rosunit
from phm_hazard_rate_calculation.phm_hazard_rate_calculation_node import HazardRateCalculationNode
from phm_msgs.msg import HazardRate

PKG = 'phm_hazard_rate_calculation'
NAME = 'test_phm_hazard_rate_calculation_node'


class TestHazardRateCalculationNode(unittest.TestCase):
    hrcn = HazardRateCalculationNode()
    hazard_rate_dict = dict({'System': {'Failure Rate': {'Nominal': 0.0005288326869051733, 'Sensor Based': 0.0005288326869051733}, 'Power': {'Failure Rate': {'Nominal': 0.000462653527447025}}, 'Mobility': {'Failure Rate': {'Nominal': 4.7201147504000006e-05}}, 'Communication': {'Failure Rate': {'Nominal': 1.06245744208149e-05}}, 'Computation': {'Failure Rate': {'Nominal': 1.15e-07}}, 'Sensing': {'Failure Rate': {'Nominal': 8.238437533333333e-06}}}})

    
    def test_set_hazard_rate_func(self):
        phm_hazard_rate = HazardRate()
        phm_hazard_rate = self.hrcn.set_hazard_rate_func(self.hazard_rate_dict, 5)

        system_value = 0.000528832686905
        system_sensor_based_value = 0.000528832686905
        module_names = ["Power", "Mobility", "Communication", "Computation", "Sensing"]
        module_values = [0.000462653527447025, 4.7201147504000006e-05, 1.06245744208149e-05, 1.15e-07, 8.238437533333333e-06]
        module_sensor_based_values = [0.0, 0.0, 0.0, 0.0, 0.0]


        self.assertAlmostEqual(phm_hazard_rate.system_value, system_value, 5)
        self.assertAlmostEqual(phm_hazard_rate.system_sensor_based_value, system_sensor_based_value, 5)
        self.assertListEqual(phm_hazard_rate.module_names, module_names)
        self.assertListEqual(phm_hazard_rate.module_values, module_values)
        self.assertListEqual(phm_hazard_rate.module_sensor_based_values, module_sensor_based_values)


if __name__ == '__main__':
	rosunit.unitrun(PKG, NAME, TestHazardRateCalculationNode, sysargs = "--cov", coverage_packages=[str(PKG)])
   