#!/usr/bin/env python

import unittest
import math
#import phm_reliability.reliability_node as rn
#from indoor_localization.localization_node import CalcPos2D3AItle

PKG = 'phm_task_plan'
NAME = 'test_task_plan_node'

class TestTaskPlanNode(unittest.TestCase):
	
    def test_1(self):
        self.assertEquals(1, 1)

    
if __name__ == '__main__':
	import rosunit
	rosunit.unitrun(PKG, NAME, TestTaskPlanNode, sysargs = "--cov", coverage_packages=[str(PKG)])
