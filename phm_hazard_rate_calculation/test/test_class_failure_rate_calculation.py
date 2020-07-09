#!/usr/bin/env python
# -*- coding: utf-8 -*-

import unittest
import math
import rosunit
from phm_hazard_rate_calculation.class_failure_rate_calculation import FailureRateCalculation

PKG = 'phm_hazard_rate_calculation'
NAME = 'test_class_failure_rate_calculation'


class TestFailureRateCalculation(unittest.TestCase):
    frc = FailureRateCalculation()

    def test_1_failure_rate_calculation_using_operating_load_func(self):
        result = self.frc.failure_rate_calculation_using_operating_load_func(5, 1, 1)
        test_result = 5

        self.assertEqual(result, test_result)


    def test_2_failure_rate_calculation_using_temperature_func(self):
        result = self.frc.failure_rate_calculation_using_temperature_func(5, 11, 1)
        test_result = 10

        self.assertEqual(result, test_result)


    def test_3_failure_rate_calculation_using_operating_load_and_temperature_func(self):
        result = self.frc.failure_rate_calculation_using_operating_load_and_temperature_func(5, 1, 1, 11, 1)
        test_result = 10

        self.assertEqual(result, test_result)


    def test_4_component_serial_failure_rate_calculation(self):
        serial_list = [5, 10, 15]
        result = self.frc.component_serial_failure_rate_calculation(serial_list)
        test_result = 30

        self.assertEqual(result, test_result)


    def test_5_component_parallel_failure_rate_calculation(self):
        parallel_list = [5, 10, 15, 20, 25]
        result = self.frc.component_parallel_failure_rate_calculation(parallel_list)
        test_result = (0.43795620 * 75)

        self.assertAlmostEqual(result, test_result, 5)

    
    def test_6_parallel_count_calculate_func(self):        
        result = self.frc.parallel_count_calculate_func(5)
        test_result = 0.43795620

        self.assertAlmostEqual(result, test_result, 5)



if __name__ == '__main__':
	rosunit.unitrun(PKG, NAME, TestFailureRateCalculation, sysargs = "--cov", coverage_packages=[str(PKG)])
   