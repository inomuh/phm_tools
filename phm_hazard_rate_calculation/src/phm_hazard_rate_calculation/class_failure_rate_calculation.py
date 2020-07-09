#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""

    PHM Gui Failure Rate Calculation Class

"""

class FailureRateCalculation:
    """
        Failure Rate Calculation Class
    """
    @classmethod
    def failure_rate_calculation_using_operating_load_func(cls, failure_rate, p_value, p_0):
        """
            λ = λ0 * (P / P0) ^ 3
        """
        result = float(failure_rate * pow((p_value / p_0), 3))

        return result

    @classmethod
    def failure_rate_calculation_using_temperature_func(cls, failure_rate, time_value, t_0):
        """
            λ = λ0 * (2 ^ ((T - T0) / 10))
        """
        result = float(float(failure_rate) * pow(2, float(float(time_value - t_0) / 10)))

        return result

    @classmethod
    def failure_rate_calculation_using_operating_load_and_temperature_func(cls, failure_rate, p_value, p_0, time_value, t_0):
        """
            λ = λ0 * ((P / P0) ^ 3) * (2 ^ ((T - T0) / 10))
        """
        result = float(failure_rate * pow((p_value / p_0), 3) * pow(2, ((time_value - t_0) / 10)))

        return result

    @classmethod
    def component_serial_failure_rate_calculation(cls, serial_fr_list):
        """
            Calculation of serial failure rate list
        """
        failure_rate_of_serial_value = 0.0

        if serial_fr_list:
            for item in serial_fr_list:
                if float(item) != 0.0 and str(item) != "None":
                    failure_rate_of_serial_value += float(item)

        return float(failure_rate_of_serial_value)


    def component_parallel_failure_rate_calculation(self, parallel_fr_list):
        """
            Calculation of parallel failure rate list
        """
        failure_rate_of_parallel_value = float(0.0)
        result = 0.0

        if parallel_fr_list:
            for item in parallel_fr_list:
                if float(item) != 0.0 and str(item) != "None":
                    failure_rate_of_parallel_value += float(item)

            result = float(failure_rate_of_parallel_value) * float(self.parallel_count_calculate_func(len(parallel_fr_list)))

        else:
            result = float(failure_rate_of_parallel_value)

        return float(result)

    @classmethod
    def parallel_count_calculate_func(cls, parallel_count):
        """
            Calculation of parallel element count
        """
        if parallel_count == 0:
            result = 1

        else:
            count = float(0.0)

            for number in range(int(parallel_count)):
                count += float(1 / (float(number) + 1))

            result = pow(count, (-1))

        return result
