#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""

    PHM Robot Task Completion Class

"""

import math
from phm_reliability_calculation.class_reliability_calculation import ReliabilityCalculation


class RobotTaskCompletion:
    """
        Actual POTC Calculation Class
    """
    def __init__(self, selected_reliability_model, selected_reliability_unit, shape_parameter):
        self.r_calculation_class = ReliabilityCalculation()
        self.selected_reliability_model = selected_reliability_model
        self.selected_reliability_unit = selected_reliability_unit
        self.shape_parameter = shape_parameter

        self.main_reliability = float(1)
        self.total_time = float(0)
        self.total_distance = float(0)
        self.last_potc = float(0)

        self.current_potc_time = float(0)
        self.current_potc_distance = float(0)

    @classmethod
    def probability_of_task_completion_formula(cls, reliability, distance):
        """
            POTC = R ^ d

            Calculation POTC Function
        """
        potc_result = float(pow(float(reliability), float(distance)))

        return potc_result

    @classmethod
    def distance_calculate(cls, start_position, finish_position):
        """
            Distance calculation function between 2 points
        """
        distance_result = float(math.sqrt(abs(pow((float(finish_position[0]) - float(start_position[0])), 2) + pow((float(finish_position[1]) - float(start_position[1])), 2))))

        return distance_result


    def path_calculate(self, task_position):
        """
            Function of calculating distances in a mission
        """
        goals = task_position
        distance_list = list()

        for goal in range(len(goals) - 1):
            distance_list.append(self.distance_calculate(goals[goal], goals[goal+1]))

        return distance_list


    def probability_of_task_completion_calculate_func(self, time_list, task_position, hazard_rate, reliability):
        """
            POTC Value Calculation Function
        """
        path_calculate_list = self.path_calculate(task_position)
        self.current_potc_time = 0
        self.current_potc_distance = 0
        self.main_reliability = reliability

        potc_calculate_list = list()
        potc_calculate_list.append([0, 1])
        counter = 0

        for path in path_calculate_list:
            temp_task_time = time_list[counter]
            self.total_time += temp_task_time
            self.current_potc_time += temp_task_time
            new_reliability = self.r_calculation_class.reliability_calculate_func(temp_task_time, hazard_rate, self.selected_reliability_model, self.selected_reliability_unit, self.shape_parameter)

            self.main_reliability = self.main_reliability * new_reliability

            calculate_distance = path

            self.total_distance += calculate_distance
            self.current_potc_distance += calculate_distance
            potc = self.probability_of_task_completion_formula(self.main_reliability, calculate_distance)
            potc_calculate_list.append([counter+1, potc])
            counter += 1
            self.last_potc = potc

        return self.last_potc


class SimulationRobotTaskCompletion:
    """
        Predict POTC Calculation Class
    """
    def __init__(self, hazard_rate, reliability, selected_reliability_model, selected_reliability_unit, shape_parameter):
        self.r_calculation_class = ReliabilityCalculation()
        self.hazard_rate = hazard_rate
        self.main_reliability = reliability
        self.selected_reliability_model = selected_reliability_model
        self.selected_reliability_unit = selected_reliability_unit
        self.shape_parameter = shape_parameter

        self.main_dict = dict()

        self.simulation_potc = float(0)
        self.simulation_time = float(0)
        self.simulation_distance = float(0)

    @classmethod
    def calculate_time_func(cls, distance, speed):
        """
            Time Calculation Function

        """
        time = float(distance / speed)

        return time


    def calculate_time_list_func(self, path_list, speed_list):
        """
            Function of calculating times in a mission
        """
        time_list = list()
        counter = 0

        for path in path_list:
            calculated_time = self.calculate_time_func(path, speed_list[counter])
            time_list.append(calculated_time)
            counter += 1

        return time_list


    def split_robot_task_list_func(self, robot_task_list):
        """
            Split incoming robot tasks into position list and time list
        """
        position_list = list()
        speed_list = list()
        time_list = list()

        for item in robot_task_list:
            position_list.append([item[0], item[1]])
            speed_list.append(item[2])

        path_list = self.path_calculate(position_list)
        time_list = self.calculate_time_list_func(path_list, speed_list)

        return position_list, time_list

    @classmethod
    def probability_of_task_completion_formula(cls, reliability, distance):
        """
            POTC = R ^ d

            Calculation POTC Function
        """
        potc_result = float(pow(float(reliability), float(distance)))

        return potc_result

    @classmethod
    def distance_calculate(cls, start_position, finish_position):
        """
            Distance calculation function between 2 points
        """
        distance_result = float(math.sqrt(abs(pow((float(finish_position[0]) - float(start_position[0])), 2) + pow((float(finish_position[1]) - float(start_position[1])), 2))))

        return distance_result


    def path_calculate(self, task_position_list):
        """
            Function of calculating distances in a mission
        """
        goals = task_position_list
        distance_list = list()

        for goal in range(len(goals) - 1):
            distance_list.append(self.distance_calculate(goals[goal], goals[goal+1]))

        return distance_list


    def calculate_potc_reliability_func(self, time):
        """
            POTC Reliability Value Calculation Function
        """
        self.simulation_time += time
        new_reliability = self.r_calculation_class.reliability_calculate_func(time, self.hazard_rate, self.selected_reliability_model, self.selected_reliability_unit, self.shape_parameter)
        self.main_reliability = self.main_reliability * new_reliability
        self.main_dict[str(float(self.simulation_time))] = {"new_reliability": float(new_reliability), "main_reliability": float(self.main_reliability), "potc": float(0.0)}
        temp_dict = {"Reliability": self.main_reliability, "Total Time": self.simulation_time}

        return temp_dict


    def calculate_potc_func(self, time_list, task_position_list):
        """
            POTC Value Calculation Function
        """
        last_potc = float(0)
        counter = 0

        for task_position in task_position_list:
            calculate_reliability_dict = self.calculate_potc_reliability_func(time_list[counter])
            current_potc_result = self.probability_of_task_completion_formula(calculate_reliability_dict["Reliability"], task_position)
            self.simulation_potc = current_potc_result
            self.simulation_distance += task_position
            self.main_dict[str(float(calculate_reliability_dict["Total Time"]))]["potc"] = current_potc_result
            last_potc = current_potc_result
            counter += 1

        return last_potc


    def prognostic_calculate_last_potc_func(self, time_list, task_position_list):
        """
            Get POTC Value
        """
        path_list = self.path_calculate(task_position_list)
        potc = float(1.0)

        potc = self.calculate_potc_func(time_list, path_list)

        return potc


    def prognostic_calculate_potc_func(self, simulation_count, time_list, task_position_list):
        """
            Get POTC values as many as the number of simulations

            Potc list = [Simulation Count, POTC Value]
        """
        path_list = self.path_calculate(task_position_list)

        potc_calculate_list = list()
        potc_calculate_list.append([0, 1])

        for count in range(int(simulation_count)):
            potc = self.calculate_potc_func(time_list, path_list)
            potc_calculate_list.append([count+1, potc])

        return potc_calculate_list
