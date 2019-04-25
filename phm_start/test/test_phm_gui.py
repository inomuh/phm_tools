#!/usr/bin/env python

import unittest
import math
import phm_start.phm_gui as phm_gui
#from indoor_localization.localization_node import CalcPos2D3AItle

PKG = 'phm_start'
NAME = 'test_phm_gui'

class TestStartGuiNode(unittest.TestCase):
	
    def test_module_array(self):
        test_temp = phm_gui.Ui_mainWindow().module_array()
        test_result = ['D2', 'D3', 'D1']

        self.assertEqual(test_temp, test_result)

    def test_component_array(self):
        test_temp = phm_gui.Ui_mainWindow().component_array()
        test_result = [['T2', 'T3'], ['T4'], ['T1']]

        self.assertEqual(test_temp, test_result)

# -------------------------------------------------------------------------------------------------

    def test_addition_object_read_system_function(self):
        test_temp = phm_gui.Ui_mainWindow().addition_object_read_system_function()
        test_result = {'D2': ['T2', 'T3'], 'D3': ['T4'], 'D1': ['T1']}

        self.assertEqual(test_temp, test_result)

# -------------------------------------------------------------------------------------------------

    def test_addition_object_read_info_function(self):
        test_temp = phm_gui.Ui_mainWindow().addition_object_read_info_function()
        test_result = "\n    D3*:\n        T4: {'ObjectCount': 5, 'ObjectFailureRate': 2.0}\n\n    D2*:\n        T3: {'ObjectCount': 3, 'ObjectFailureRate': 55555.0}\n        T2: {'ObjectCount': 10, 'ObjectFailureRate': 10.0}\n\n    D1*:\n        T1: {'ObjectCount': 5, 'ObjectFailureRate': 22222.0}\n"

        self.assertEqual(test_temp, test_result)

# -------------------------------------------------------------------------------------------------

    def test_addition_object_read_type_function(self):
        test_temp = phm_gui.Ui_mainWindow().addition_object_read_type_function()
        test_result = "\n    D3*:\n        T4:\n            type: Parallel\n        unrelated: ['T4']\n        relation: {'T4': {'Serial': [], 'Parallel': []}}\n        type*: Serial\n\n    D2*:\n        T3:\n            type: Parallel\n        T2:\n            type: Serial\n        unrelated: ['T2', 'T3']\n        relation: {'T2': {'Serial': [], 'Parallel': []},\n                   'T3': {'Serial': [], 'Parallel': []}}\n        type*: Parallel\n\n    D1*:\n        T1:\n            type: Serial\n        unrelated: ['T1']\n        relation: {'T1': {'Serial': [], 'Parallel': []}}\n        type*: Serial\n"

        self.assertEqual(test_temp, test_result)

# -------------------------------------------------------------------------------------------------
    """
    def test_select_module_component_value(self):
        phm_gui.Ui_mainWindow().combo_box_module.currentIndexChanged(1)
        test_temp = phm_gui.Ui_mainWindow().select_module_component_value()
        test_result = "\n    D3*:\n        T4:\n            type: Parallel\n        unrelated: ['T4']\n        relation: {'T4': {'Serial': [], 'Parallel': []}}\n        type*: Serial\n\n    D2*:\n        T3:\n            type: Parallel\n        T2:\n            type: Serial\n        unrelated: ['T2', 'T3']\n        relation: {'T2': {'Serial': [], 'Parallel': []},\n                   'T3': {'Serial': [], 'Parallel': []}}\n        type*: Parallel\n\n    D1*:\n        T1:\n            type: Serial\n        unrelated: ['T1']\n        relation: {'T1': {'Serial': [], 'Parallel': []}}\n        type*: Serial\n"

        self.assertEqual(test_temp, test_result)
    """
# -------------------------------------------------------------------------------------------------

    def test_read_parameter_field(self):
        test_temp = phm_gui.Ui_mainWindow().read_parameter_field()
        test_result = "---\nInfo**:\n    D3*:\n        T4: {'ObjectCount': 5, 'ObjectFailureRate': 2.0}\n\n    D2*:\n        T3: {'ObjectCount': 3, 'ObjectFailureRate': 55555.0}\n        T2: {'ObjectCount': 10, 'ObjectFailureRate': 10.0}\n\n    D1*:\n        T1: {'ObjectCount': 5, 'ObjectFailureRate': 22222.0}\n"

        self.assertEqual(test_temp, test_result)

# -------------------------------------------------------------------------------------------------

    def test_read_task_field(self):
        test_temp = phm_gui.Ui_mainWindow().read_task_field()
        test_result = 'D1: 4\nD2: 1\nD3: 3'

        self.assertEqual(test_temp, test_result)

# -------------------------------------------------------------------------------------------------

    def test_read_variable_field(self):
        test_temp = phm_gui.Ui_mainWindow().read_variable_field()
        test_result = 'Variable: {\ndegisken1: 10,\ndegisken2: 20,\ndegisken3: degisken1+ degisken2,\nsayi1: 10\n}'

        self.assertEqual(test_temp, test_result)

# -------------------------------------------------------------------------------------------------

    def test_read_formula_field(self):
        test_temp = phm_gui.Ui_mainWindow().read_formula_field()
        test_result = 'Formula: (ozel_fonksiyon(degisken1,degisken2) + degisken3) / sayi1'

        self.assertEqual(test_temp, test_result)

    
if __name__ == '__main__':
	import rosunit
	rosunit.unitrun(PKG, NAME, TestStartGuiNode, sysargs = "--cov", coverage_packages=[str(PKG)])
