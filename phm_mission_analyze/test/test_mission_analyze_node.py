#!/usr/bin/env python

import unittest
import math
#import phm_reliability.reliability_node as rn

PKG = 'phm_mission_analyze'
NAME = 'test_mission_analyze_node'

class TestMissionAnalyzeNode(unittest.TestCase):
	
    def test_1(self):
        self.assertEquals(1, 1)
    """
    def test_2(self):
        test_deneme = rn.percentage_calculated(80)
        self.assertEqual(test_deneme, 8000)
    """

if __name__ == '__main__':
	import rosunit
	rosunit.unitrun(PKG, NAME, TestMissionAnalyzeNode, sysargs = "--cov", coverage_packages=[str(PKG)])
