#!/usr/bin/env python
# -*- coding: utf-8 -*-

import unittest
import math
import rosunit
from phm_robot_task_completion.class_robot_task_completion import RobotTaskCompletion, SimulationRobotTaskCompletion

PKG = 'phm_robot_task_completion'
NAME = 'test_class_robot_task_completion'


class TestClassPOTC(unittest.TestCase):
    usage_time = 100000
    failure_rate = 0.5e-6
    reliability = 0.754321
    selected_reliability_model = "Exponential Distribution"
    selected_reliability_unit = False
    shape_parameter = 1


# RobotTaskCompletion Class - START -
    def test_rtc_probability_of_task_completion_formula(self):
        rtc = RobotTaskCompletion(self.selected_reliability_model, self.selected_reliability_unit, self.shape_parameter)

        result = rtc.probability_of_task_completion_formula(self.reliability, 5.678)
        test_result = 0.2017270301914359

        # result = float(pow(float(0.754321), float(5.678)))

        self.assertAlmostEqual(result, test_result, 5)


    def test_rtc_distance_calculate(self):
        rtc = RobotTaskCompletion(self.selected_reliability_model, self.selected_reliability_unit, self.shape_parameter)

        start_position = [4.173425111935546, 0.6996212725439405]
        finish_position = [4.252327779615457, 1.7887780478495028]
        result = rtc.distance_calculate(start_position, finish_position)
        test_result = 1.0920110403109566

        # result = float(math.sqrt(abs(pow((float(4.252327779615457) - float(4.173425111935546)), 2) + pow((float(1.7887780478495028) - float(0.6996212725439405)), 2))))

        self.assertAlmostEqual(result, test_result, 5)


    def test_rtc_path_calculate(self):
        rtc = RobotTaskCompletion(self.selected_reliability_model, self.selected_reliability_unit, self.shape_parameter)

        task_position = [[4.173425111935546, 0.6996212725439405], [4.252327779615457, 1.7887780478495028], [4.254796821506879, 2.986813819580745], [4.254796821506879, 2.986813819580745], [4.075966735712705, 2.45786878651297], [5.296776265140758, 3.2377047247386685], [7.186286384259671, 3.4575905505176574], [7.186286384259671, 3.4575905505176574], [10.02707803637877, 3.834368378077772], [10.713219522235965, 3.6906226762430725], [11.321258808297841, 3.0485739135240486], [11.451096448575916, 2.271822575810481], [11.191486774449103, 1.4076532146026228], [8.491930981852743, 0.7301303113520736], [6.714700980072448, 0.7608179284539074], [4.405262763785259, 0.5861917110532081]]
        result = rtc.path_calculate(task_position)
        test_result = [1.0920110403109566, 1.198038315963031, 0.0, 0.5583574550341579, 1.448626935304842, 1.9022613560263684, 0.0, 2.865668986831954, 0.7010370642200527, 0.884272801291358, 0.7875280651969145, 0.902322485451087, 2.7832784197362677, 1.7774949251886425, 2.316030913146763]

        self.assertListEqual(result, test_result)


    def test_rtc_probability_of_task_completion_calculate_func(self):
        rtc = RobotTaskCompletion(self.selected_reliability_model, self.selected_reliability_unit, self.shape_parameter)

        time_list = [7.070728063583374, 7.325403928756714, 3.0024330615997314, 5.1106040477752686, 10.97532606124878, 12.948495864868164, 3.0029280185699463, 17.01675295829773, 3.674638032913208, 4.787661075592041, 4.183042049407959, 4.751224040985107, 16.489573001861572, 10.011996984481812, 17.285614013671875]
        task_position = [[4.173425111935546, 0.6996212725439405], [4.252327779615457, 1.7887780478495028], [4.254796821506879, 2.986813819580745], [4.254796821506879, 2.986813819580745], [4.075966735712705, 2.45786878651297], [5.296776265140758, 3.2377047247386685], [7.186286384259671, 3.4575905505176574], [7.186286384259671, 3.4575905505176574], [10.02707803637877, 3.834368378077772], [10.713219522235965, 3.6906226762430725], [11.321258808297841, 3.0485739135240486], [11.451096448575916, 2.271822575810481], [11.191486774449103, 1.4076532146026228], [8.491930981852743, 0.7301303113520736], [6.714700980072448, 0.7608179284539074], [4.405262763785259, 0.5861917110532081]]
        result = rtc.probability_of_task_completion_calculate_func(time_list, task_position, self.failure_rate, self.reliability)
        test_result = 0.5204178446022534

        # result = float(math.sqrt(abs(pow((float(4.252327779615457) - float(4.173425111935546)), 2) + pow((float(1.7887780478495028) - float(0.6996212725439405)), 2))))

        self.assertAlmostEqual(result, test_result, 5)

# RobotTaskCompletion Class - END -

# SimulationRobotTaskCompletion Class - START -

    def test_srtc_calculate_time_func(self):
        srtc = SimulationRobotTaskCompletion(self.failure_rate, self.reliability, self.selected_reliability_model, self.selected_reliability_unit, self.shape_parameter)

        result = srtc.calculate_time_func(100, 20)
        test_result = 5.0

        # result = float(100 / 50)

        self.assertAlmostEqual(result, test_result, 2)


    def test_srtc_path_calculate(self):
        srtc = SimulationRobotTaskCompletion(self.failure_rate, self.reliability, self.selected_reliability_model, self.selected_reliability_unit, self.shape_parameter)

        task_position = [[4.25, 2.18], [4.25, 3.1], [4.25, 2.18], [5.55, 3.54], [7.29, 3.53], [10.34, 3.91], [11.05, 3.67], [11.42, 2.76], [11.42, 1.93], [10.97, 1.08], [8.16, 0.82], [6.35, 0.83], [4.26, 0.68]]
        result = srtc.path_calculate(task_position)
        test_result = [0.9199999999999999, 0.9199999999999999, 1.8813824704190265, 1.7400287353949075, 3.0735809733924366, 0.7494664769020702, 0.9823441352194249, 0.8299999999999998, 0.9617692030835668, 2.822002834867464, 1.8100276240985942, 2.0953758612716715]

        self.assertListEqual(result, test_result)


    def test_srtc_calculate_time_list_func(self):
        srtc = SimulationRobotTaskCompletion(self.failure_rate, self.reliability, self.selected_reliability_model, self.selected_reliability_unit, self.shape_parameter)

        path_list = [0.9199999999999999, 0.9199999999999999, 1.8813824704190265, 1.7400287353949075, 3.0735809733924366, 0.7494664769020702, 0.9823441352194249, 0.8299999999999998, 0.9617692030835668, 2.822002834867464, 1.8100276240985942, 2.0953758612716715]
        speed_list = [0.305032286643936, 0.34244299963151925, 0.3130261708222558, 0.25665934871261936, 0.3099463716242755, 0.33396332918153804, 0.34579751666395014, 0.34335229915257853, 0.32025931883120473, 0.2732565117783081, 0.3424904966267803, 0.3364506314969207, 0.30230026016025063]
        result = srtc.calculate_time_list_func(path_list, speed_list)
        test_result = [3.0160741674992435, 2.6865784991661457, 6.010304076100152, 6.779526029824115, 9.916492834825974, 2.244158000037993, 2.840807373912051, 2.417342193567678, 3.0030951373829504, 10.32730315007806, 5.284898827633814, 6.227885059834845]

        self.assertListEqual(result, test_result)


    def test_srtc_split_robot_task_list_func(self):
        srtc = SimulationRobotTaskCompletion(self.failure_rate, self.reliability, self.selected_reliability_model, self.selected_reliability_unit, self.shape_parameter)

        robot_task_list = [[4.25, 2.18, 0.305032286643936], [4.25, 3.1, 0.34244299963151925], [4.25, 2.18, 0.3130261708222558], [5.55, 3.54, 0.25665934871261936], [7.29, 3.53, 0.3099463716242755], [10.34, 3.91, 0.33396332918153804], [11.05, 3.67, 0.34579751666395014], [11.42, 2.76, 0.34335229915257853], [11.42, 1.93, 0.32025931883120473], [10.97, 1.08, 0.2732565117783081], [8.16, 0.82, 0.3424904966267803], [6.35, 0.83, 0.3364506314969207], [4.26, 0.68, 0.30230026016025063]]
        position_result, time_result = srtc.split_robot_task_list_func(robot_task_list)
        test_position_result = [[4.25, 2.18], [4.25, 3.1], [4.25, 2.18], [5.55, 3.54], [7.29, 3.53], [10.34, 3.91], [11.05, 3.67], [11.42, 2.76], [11.42, 1.93], [10.97, 1.08], [8.16, 0.82], [6.35, 0.83], [4.26, 0.68]]
        test_time_result = [3.0160741674992435, 2.6865784991661457, 6.010304076100152, 6.779526029824115, 9.916492834825974, 2.244158000037993, 2.840807373912051, 2.417342193567678, 3.0030951373829504, 10.32730315007806, 5.284898827633814, 6.227885059834845]

        self.assertListEqual(position_result, test_position_result)
        self.assertListEqual(time_result, test_time_result)


    def test_srtc_probability_of_task_completion_formula(self):
        srtc = SimulationRobotTaskCompletion(self.failure_rate, self.reliability, self.selected_reliability_model, self.selected_reliability_unit, self.shape_parameter)

        result = srtc.probability_of_task_completion_formula(self.reliability, 5.678)
        test_result = 0.2017270301914359

        # result = float(pow(float(0.754321), float(5.678)))

        self.assertAlmostEqual(result, test_result, 5)


    def test_srtc_distance_calculate(self):
        srtc = SimulationRobotTaskCompletion(self.failure_rate, self.reliability, self.selected_reliability_model, self.selected_reliability_unit, self.shape_parameter)

        start_position = [4.173425111935546, 0.6996212725439405]
        finish_position = [4.252327779615457, 1.7887780478495028]
        result = srtc.distance_calculate(start_position, finish_position)
        test_result = 1.0920110403109566

        # result = float(math.sqrt(abs(pow((float(4.252327779615457) - float(4.173425111935546)), 2) + pow((float(1.7887780478495028) - float(0.6996212725439405)), 2))))

        self.assertAlmostEqual(result, test_result, 5)


    def test_srtc_calculate_potc_reliability_func(self):
        srtc = SimulationRobotTaskCompletion(self.failure_rate, self.reliability, self.selected_reliability_model, self.selected_reliability_unit, self.shape_parameter)

        result = srtc.calculate_potc_reliability_func(3.0160741674992435)
        test_result = {'Reliability': 0.7543198624568167, 'Total Time': 3.0160741674992435}

        self.assertDictEqual(result, test_result)


    def test_srtc_calculate_potc_func(self):
        srtc = SimulationRobotTaskCompletion(self.failure_rate, self.reliability, self.selected_reliability_model, self.selected_reliability_unit, self.shape_parameter)

        time_list = [3.0160741674992435, 2.6865784991661457, 6.010304076100152, 6.779526029824115, 9.916492834825974, 2.244158000037993, 2.840807373912051, 2.417342193567678, 3.0030951373829504, 10.32730315007806, 5.284898827633814, 6.227885059834845]
        task_position_list = [0.9199999999999999, 0.9199999999999999, 1.8813824704190265, 1.7400287353949075, 3.0735809733924366, 0.7494664769020702, 0.9823441352194249, 0.8299999999999998, 0.9617692030835668, 2.822002834867464, 1.8100276240985942, 2.0953758612716715]
        result = srtc.calculate_potc_func(time_list, task_position_list)
        test_result = 0.5538683775401194

        self.assertAlmostEqual(result, test_result, 5)


    def test_srtc_prognostic_calculate_last_potc_func(self):
        srtc = SimulationRobotTaskCompletion(self.failure_rate, self.reliability, self.selected_reliability_model, self.selected_reliability_unit, self.shape_parameter)

        time_list = [3.0160741674992435, 2.6865784991661457, 6.010304076100152, 6.779526029824115, 9.916492834825974, 2.244158000037993, 2.840807373912051, 2.417342193567678, 3.0030951373829504, 10.32730315007806, 5.284898827633814, 6.227885059834845]
        task_position_list = [[4.25, 2.18], [4.25, 3.1], [4.25, 2.18], [5.55, 3.54], [7.29, 3.53], [10.34, 3.91], [11.05, 3.67], [11.42, 2.76], [11.42, 1.93], [10.97, 1.08], [8.16, 0.82], [6.35, 0.83], [4.26, 0.68]]
        result = srtc.prognostic_calculate_last_potc_func(time_list, task_position_list)
        test_result = 0.5538683775401194

        self.assertAlmostEqual(result, test_result, 5)


    def test_srtc_prognostic_calculate_potc_func(self):
        srtc = SimulationRobotTaskCompletion(self.failure_rate, self.reliability, self.selected_reliability_model, self.selected_reliability_unit, self.shape_parameter)

        simulation_count = 10
        time_list = [3.0160741674992435, 2.6865784991661457, 6.010304076100152, 6.779526029824115, 9.916492834825974, 2.244158000037993, 2.840807373912051, 2.417342193567678, 3.0030951373829504, 10.32730315007806, 5.284898827633814, 6.227885059834845]
        task_position_list = [[4.25, 2.18], [4.25, 3.1], [4.25, 2.18], [5.55, 3.54], [7.29, 3.53], [10.34, 3.91], [11.05, 3.67], [11.42, 2.76], [11.42, 1.93], [10.97, 1.08], [8.16, 0.82], [6.35, 0.83], [4.26, 0.68]]
        result = srtc.prognostic_calculate_potc_func(simulation_count, time_list, task_position_list)
        test_result = [[0, 1], [1, 0.5538683775401194], [2, 0.5538331239871783], [3, 0.553797872678115], [4, 0.5537626236127864], [5, 0.55372737679105], [6, 0.5536921322127634], [7, 0.5536568898777836], [8, 0.5536216497859677], [9, 0.5535864119371724], [10, 0.5535511763312557]]

        self.assertListEqual(result, test_result)


# SimulationRobotTaskCompletion Class - END -


if __name__ == '__main__':
    rosunit.unitrun(PKG, NAME, TestClassPOTC, sysargs = "--cov", coverage_packages=[str(PKG)])
   