#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""

    PHM Gui Reliability Calculation Class

"""

import math


class ReliabilityCalculation:
    """
        Reliability Calculation Class
    """
    def reliability_calculate_func(self, usage_time, failure_rate, selected_reliability_model, selected_reliability_unit, shape_parameter):
        """
            Reliability Calculation Function

            Reliability Types = Exponential, Rayleigh, Weibull, Curve Distribution
        """
        reliability = 1.0
        unit = 1

        if selected_reliability_unit:
            unit = 3600

        reliability_time = float(usage_time) / float(unit)

        if selected_reliability_model == "Exponential Distribution":
            reliability = self.reliability_exponential_func(reliability_time, failure_rate)

        elif selected_reliability_model == "Rayleigh Distribution":
            reliability = self.reliability_rayleigh_func(reliability_time, failure_rate)

        elif selected_reliability_model == "Weibull Distribution":
            reliability = self.reliability_weibull_func(reliability_time, failure_rate, shape_parameter)

        elif selected_reliability_model == "Curve Distribution":
            reliability = self.reliability_curve_func(reliability_time, failure_rate, shape_parameter)

        else:
            reliability = 1.0

        return float(reliability)

    @classmethod
    def reliability_exponential_func(cls, reliability_time, failure_rate):
        """
            Reliability Model = Exponential Distribution

            R = e ^ -(λt)
        """
        return float(math.exp(float(-1) * float(reliability_time) * float(failure_rate)))

    @classmethod
    def reliability_rayleigh_func(cls, reliability_time, failure_rate):
        """
            Reliability Model = Rayleigh Distribution

            R = e ^ -( (t/λ) ^ 2 )
        """
        return float(math.exp(float(-1) * pow((float(reliability_time) / float(failure_rate)), 2)))

    @classmethod
    def reliability_weibull_func(cls, reliability_time, failure_rate, shape_parameter):
        """
            Reliability Model = Weibull Distribution

            R = e ^ -( (t/λ) ^ c )
        """
        return float(math.exp(float(-1) * pow((float(reliability_time) / float(failure_rate)), shape_parameter)))

    @classmethod
    def reliability_curve_func(cls, reliability_time, failure_rate, shape_parameter):
        """
            Reliability Model = Curve Distribution

            R = e ^ -( e ^ ( (λt) ^ c ) - 1 )
        """
        mini_formula = float(math.exp(pow((float(reliability_time) * float(failure_rate)), shape_parameter))) - float(1)
        return float(math.exp(float(-1) * mini_formula))
