#!/usr/bin/env python

import unittest
import math
import phm_reliability.reliability_node as rn

PKG = 'phm_reliability'
NAME = 'test_reliability_node'

class TestReliabilityNode(unittest.TestCase):
    def info_parameters(self):
        info_dict = dict({'D2*': {'T2': {'ObjectCount': 10, 'ObjectFailureRate': 10.0}, 'T3': {'ObjectCount': 3, 'ObjectFailureRate': 55555.0}}, 'D3*': {'T4': {'ObjectCount': 5, 'ObjectFailureRate': 2.0}}, 'D1*': {'T1': {'ObjectCount': 5, 'ObjectFailureRate': 22222.0}}})
        
        return info_dict
    
    def type_parameters(self):
        type_dict = dict({
                            'D2*': {
                                        'T2': {'type': 'Serial'},
                                        'T3': {'type': 'Parallel'},
                                        'unrelated': ['T2', 'T3'],
                                        'relation': {
                                                        'T2': {'Serial': [], 'Parallel': []},
                                                        'T3': {'Serial': [], 'Parallel': []}},
                                        'type*': 'Parallel'},
                            'D3*': {
                                        'T4': {'type': 'Parallel'},
                                        'unrelated': ['T4'],
                                        'relation': {
                                                        'T4': {'Serial': [], 'Parallel': []}},
                                        'type*': 'Serial'},
                            'D1*': {    
                                        'T1': {'type': 'Serial'},
                                        'unrelated': ['T1'],
                                        'relation': {
                                                        'T1': {'Serial': [], 'Parallel': []}},                                        
                                        'type*': 'Serial'}})

        return type_dict


    def system_graph_dict(self):
        temp_dict = dict ({'System': {'ModuleValue': '', 'Reliability': 0.0, 'Type': '', 'FailureRate': ''}, 'D2': {'ModuleValue': 1, 'Reliability': 0.0, 'Type': 'Parallel', 'FailureRate': 30402.727272727276}, 'D3': {'ModuleValue': 3, 'Reliability': 7.224174, 'Type': 'Serial', 'FailureRate': 0.8759124087591241}, 'D1': {'ModuleValue': 4, 'Reliability': 0.0, 'Type': 'Serial', 'FailureRate': 111110.0}})

        return temp_dict

    
    def module_graph_dict(self):
        temp_dict = dict({'D2': {'ObjectCount': '', 'FailureRate': 30402.727272727276, 'Type': 'Parallel', 'ObjectFailureRate': ''}, 'T2': {'ObjectCount': 10, 'FailureRate': 100.0, 'Type': 'Serial', 'ObjectFailureRate': 10.0}, 'T3': {'ObjectCount': 3, 'FailureRate': 30302.727272727276, 'Type': 'Parallel', 'ObjectFailureRate': 55555.0}})

        return temp_dict


    def module_array(self):
        temp_list = ['D2', 'D3', 'D1']


        return temp_list
        
    def component_array(self):
        temp_list = [['T2', 'T3'], ['T4'], ['T1']]


        return temp_list


    def module_usage_during_task(self):
        module_usage_during_task_list = [1, 3, 4]

        return module_usage_during_task_list


    def failure_rate_for_modules(self):
        module_failure_rate = [30402.727272727276, 0.8759124087591241, 111110.0]

        return module_failure_rate
        

# -------------------------------------------------------------------------------------------------
   

    def test_get_select_parameter_dict_1(self):
        info_dict = self.info_parameters()
        reliability_module = "D2"
        component_list = "T3"
        test_temp = rn.get_select_parameter_dict(info_dict, reliability_module, component_list)
        test_result = dict({'ObjectCount': 3, 'ObjectFailureRate': 55555.0})

        self.assertDictEqual(test_temp, test_result)


    def test_get_select_parameter_dict_2(self):
        type_dict = self.type_parameters()
        reliability_module = "D2"
        component_list = "T3"
        test_temp = rn.get_select_parameter_dict(type_dict, reliability_module, component_list)
        test_result = dict({'type': 'Parallel'})

        self.assertDictEqual(test_temp, test_result)
        
# -------------------------------------------------------------------------------------------------
    def test_get_select_module_type(self):
        type_dict = self.type_parameters()
        reliability_module = "D2"
        test_temp = rn.get_select_module_type(type_dict, reliability_module)
        test_result = 'Parallel'

        self.assertEqual(test_temp, test_result)

# -------------------------------------------------------------------------------------------------
    def test_object_failure_rate_calculation_1(self):
        temp_dict = dict({'ObjectCount': 3, 'ObjectFailureRate': 55555.0})
        temp_type = "Serial"
        test_temp = rn.object_failure_rate_calculation(temp_dict, temp_type)
        test_result = 166665.0

        self.assertEqual(test_temp, test_result)

    def test_object_failure_rate_calculation_2(self):
        temp_dict = dict({'ObjectCount': 3, 'ObjectFailureRate': 55555.0})
        temp_type = "Parallel"
        test_temp = rn.object_failure_rate_calculation(temp_dict, temp_type)
        test_result = 30302.727272727276

        self.assertEqual(test_temp, test_result)

# -------------------------------------------------------------------------------------------------

    def test_object_parallel_failure_rate_calculation(self):
        temp_dict = dict({'ObjectCount': 3, 'ObjectFailureRate': 55555.0})        
        test_temp = rn.object_parallel_failure_rate_calculation(temp_dict)
        test_result = 30302.727272727276

        self.assertEqual(test_temp, test_result)


    def test_object_serial_failure_rate_calculation(self):
        temp_dict = dict({'ObjectCount': 3, 'ObjectFailureRate': 55555.0})        
        test_temp = rn.object_serial_failure_rate_calculation(temp_dict)
        test_result = 166665.0

        self.assertEqual(test_temp, test_result)

# -------------------------------------------------------------------------------------------------

    def test_component_parallel_failure_rate_calculation_1(self):
        temp_list = [1.00, 2.00, 3.00, 4.00, 5.00]        
        test_temp = rn.component_parallel_failure_rate_calculation(temp_list)
        test_result = 6.569343065693431

        self.assertEqual(test_temp, test_result)

    def test_component_parallel_failure_rate_calculation_2(self):
        temp_list = list()        
        test_temp = rn.component_parallel_failure_rate_calculation(temp_list)
        test_result = 0.0

        self.assertEqual(test_temp, test_result)

    def test_component_serial_failure_rate_calculation_1(self):
        temp_list = [1.00, 2.00, 3.00, 4.00, 5.00]        
        test_temp = rn.component_serial_failure_rate_calculation(temp_list)
        test_result = 15.0

        self.assertEqual(test_temp, test_result)

    def test_component_serial_failure_rate_calculation_2(self):
        temp_list = list()        
        test_temp = rn.component_serial_failure_rate_calculation(temp_list)
        test_result = 0.0

        self.assertEqual(test_temp, test_result)
    
# -------------------------------------------------------------------------------------------------

    def test_reliability_parallel_formula_1(self):
        test_temp = rn.reliability_parallel_formula(1)
        test_result = 0

        self.assertEqual(test_temp, test_result)

    def test_reliability_parallel_formula_2(self):
        test_temp = rn.reliability_parallel_formula(2)
        test_result = -1.0

        self.assertEqual(test_temp, test_result)

    def test_reliability_parallel_calculation_1(self):
        temp_list = list()
        test_temp = rn.reliability_parallel_calculation(temp_list)
        test_result = 1.0

        self.assertEqual(test_temp, test_result)

    def test_reliability_parallel_calculation_2(self):
        temp_list = [1.0]
        test_temp = rn.reliability_parallel_calculation(temp_list)
        test_result = 1.0

        self.assertEqual(test_temp, test_result)

    def test_reliability_serial_calculation_1(self):
        temp_list = list()
        test_temp = rn.reliability_serial_calculation(temp_list)
        test_result = 1.0

        self.assertEqual(test_temp, test_result)

    def test_reliability_serial_calculation_2(self):
        temp_list = [1.0]
        test_temp = rn.reliability_serial_calculation(temp_list)
        test_result = 1.0

        self.assertEqual(test_temp, test_result)

# -------------------------------------------------------------------------------------------------

    def test_failure_rate_for_modules(self):
        reliability_module = self.module_array()
        component_list = self.component_array()
        info_dict = self.info_parameters()
        type_dict = self.type_parameters()
        graph_control = False
        test_temp = rn.failure_rate_for_modules(reliability_module, component_list, info_dict, type_dict, graph_control)
        test_result = [30402.727272727276, 0.8759124087591241, 111110.0]

        self.assertEqual(test_temp, test_result)
    
# -------------------------------------------------------------------------------------------------

    def test_nominal_reliability_calculate(self):
        test_temp = rn.nominal_reliability_calculate(1, 1)
        test_result = 0.36787944117144233

        self.assertEqual(test_temp, test_result)

# -------------------------------------------------------------------------------------------------

    def test_parallel_count_calculate_1(self):
        test_temp = rn.parallel_count_calculate(0)
        test_result = 1

        self.assertEqual(test_temp, test_result)

    def test_parallel_count_calculate_2(self):
        test_temp = rn.parallel_count_calculate(3)
        test_result = 0.545454545454545501
        
        self.assertEqual(test_temp, test_result)

# -------------------------------------------------------------------------------------------------

    def test_percentage_calculated(self):
        test_temp = rn.percentage_calculated(80)
        test_result = 8000

        self.assertEqual(test_temp, test_result)

# -------------------------------------------------------------------------------------------------

    def test_reliability_calculation_function(self):
        reliability_module = self.module_array()
        type_dict = self.type_parameters()
        module_usage_during_task_list = self.module_usage_during_task()
        module_failure_rate = self.failure_rate_for_modules()
        want_a_string = False
        graph_control = False
        test_temp = rn.reliability_calculation_function(type_dict, reliability_module, module_usage_during_task_list, module_failure_rate, want_a_string, graph_control)
        test_result = 0.0

        self.assertEqual(test_temp, test_result)
    
    
# -------------------------------------------------------------------------------------------------

    def test_create_system_graph(self):
        module_name = "System"
        graph_dict = self.system_graph_dict()
        test_temp = rn.create_system_graph(module_name, graph_dict)
        test_result = True

        self.assertEqual(test_temp, test_result)

    def test_create_module_graph(self):
        module_name = "D2"
        graph_dict = self.module_graph_dict()
        test_temp = rn.create_module_graph(module_name, graph_dict)
        test_result = True

        self.assertEqual(test_temp, test_result)
# -------------------------------------------------------------------------------------------------

    def test_monitoring(self):
        rn.show_value_float = 1.0
        test_temp = rn.monitoring(False)
        test_result = 1.0

        self.assertEqual(test_temp, test_result)

# -------------------------------------------------------------------------------------------------

if __name__ == '__main__':
	import rosunit
	rosunit.unitrun(PKG, NAME, TestReliabilityNode, sysargs = "--cov", coverage_packages=[str(PKG)])
    #import rostest
    #rostest.rosrun(PKG, NAME, TestReliabilityNode)


	
    