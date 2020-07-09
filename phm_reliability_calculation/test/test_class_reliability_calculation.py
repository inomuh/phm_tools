#!/usr/bin/env python
# -*- coding: utf-8 -*-

import unittest
import math
import rosunit
from phm_reliability_calculation.class_reliability_calculation import ReliabilityCalculation

PKG = 'phm_reliability_calculation'
NAME = 'test_class_reliability_calculation'


class TestReliabilityCalculation(unittest.TestCase):
    rc = ReliabilityCalculation()

    usage_time = 100000
    failure_rate = 0.5e-6


    def test_1_reliability_calculate_func(self):
        selected_reliability_model = "Exponential Distribution"
        selected_reliability_unit = False
        shape_parameter = 1

        result = self.rc.reliability_calculate_func(self.usage_time, self.failure_rate, selected_reliability_model, selected_reliability_unit, shape_parameter)
        test_result = 0.951229424500714

        # result = math.exp(float(-1) * float(100000) * float(0.5e-6))

        self.assertAlmostEqual(result, test_result, 5)


    def test_2_reliability_calculate_func(self):
        selected_reliability_model = "Exponential Distribution"
        selected_reliability_unit = True
        shape_parameter = 1

        result = self.rc.reliability_calculate_func(self.usage_time, self.failure_rate, selected_reliability_model, selected_reliability_unit, shape_parameter)
        test_result = 0.9999861112075613

        # result = math.exp(float(-1) * float(100000/3600) * float(0.5e-6))

        self.assertAlmostEqual(result, test_result, 5)


    def test_3_reliability_calculate_func(self):
        selected_reliability_model = "Rayleigh Distribution"
        selected_reliability_unit = False
        shape_parameter = 1

        result = self.rc.reliability_calculate_func(self.usage_time, self.failure_rate, selected_reliability_model, selected_reliability_unit, shape_parameter)
        test_result = 0.0

        # result = math.exp(float(-1) * pow((float(100000) / float(0.5e-6)), 2))

        self.assertAlmostEqual(result, test_result, 5)


    def test_4_reliability_calculate_func(self):
        selected_reliability_model = "Rayleigh Distribution"
        selected_reliability_unit = True
        shape_parameter = 1

        result = self.rc.reliability_calculate_func(self.usage_time, self.failure_rate, selected_reliability_model, selected_reliability_unit, shape_parameter)
        test_result = 0.0

        # result = math.exp(float(-1) * pow((float(100000/3600) / float(0.5e-6)), 2))

        self.assertAlmostEqual(result, test_result, 5)


    def test_5_reliability_calculate_func(self):
        selected_reliability_model = "Weibull Distribution"
        selected_reliability_unit = False
        shape_parameter = 1.5

        result = self.rc.reliability_calculate_func(self.usage_time, self.failure_rate, selected_reliability_model, selected_reliability_unit, shape_parameter)
        test_result = 0.0

        # result = math.exp(float(-1) * pow((float(100000) / float(0.5e-6)), 1.5))

        self.assertAlmostEqual(result, test_result, 5)


    def test_6_reliability_calculate_func(self):
        selected_reliability_model = "Weibull Distribution"
        selected_reliability_unit = True
        shape_parameter = 1.5

        result = self.rc.reliability_calculate_func(self.usage_time, self.failure_rate, selected_reliability_model, selected_reliability_unit, shape_parameter)
        test_result = 0.0

        # result = math.exp(float(-1) * pow((float(100000/3600) / float(0.5e-6)), 1.5))

        self.assertAlmostEqual(result, test_result, 5)


    def test_7_reliability_calculate_func(self):
        selected_reliability_model = "Curve Distribution"
        selected_reliability_unit = False
        shape_parameter = 1.5

        result = self.rc.reliability_calculate_func(self.usage_time, self.failure_rate, selected_reliability_model, selected_reliability_unit, shape_parameter)
        test_result = 0.9888198936843544

        # result = math.exp(float(-1) * (float(math.exp(pow((float(100000) * float(0.5e-6)), 1.5))) - float(1)))

        self.assertAlmostEqual(result, test_result, 5)


    def test_8_reliability_calculate_func(self):
        selected_reliability_model = "Curve Distribution"
        selected_reliability_unit = True
        shape_parameter = 1.5

        result = self.rc.reliability_calculate_func(self.usage_time, self.failure_rate, selected_reliability_model, selected_reliability_unit, shape_parameter)
        test_result = 0.9999999482391673

        # result = math.exp(float(-1) * (float(math.exp(pow((float(100000/3600) * float(0.5e-6)), 1.5))) - float(1)))

        self.assertAlmostEqual(result, test_result, 5)


    def test_9_reliability_calculate_func(self):
        selected_reliability_model = "None"
        selected_reliability_unit = False
        shape_parameter = 1.5

        result = self.rc.reliability_calculate_func(self.usage_time, self.failure_rate, selected_reliability_model, selected_reliability_unit, shape_parameter)
        test_result = 1.0

        self.assertAlmostEqual(result, test_result, 5)


if __name__ == '__main__':
    rosunit.unitrun(PKG, NAME, TestReliabilityCalculation, sysargs = "--cov", coverage_packages=[str(PKG)])
   