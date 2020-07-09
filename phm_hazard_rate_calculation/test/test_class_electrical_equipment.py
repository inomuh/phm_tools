#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import unittest
import math
import rosunit
from phm_hazard_rate_calculation.class_electrical_equipment import ElectricalEquipment

PKG = 'phm_hazard_rate_calculation'
NAME = 'test_class_electrical_equipment'


class TestElectricalEquipment(unittest.TestCase):
    e_eq = ElectricalEquipment()

# QUARTZ CRYSTALS FORMULAS - START-

    def test_qrtz_lambda_b_func(self):
        self.e_eq.qrtz_lambda_b_func(5)
        result = self.e_eq.get_qrtz_lambda_b()
        test_result = 0.018823763306560406  
        # result = float(0.013 * pow(5, 0.23))

        self.assertAlmostEqual(result, test_result, 5)

# QUARTZ CRYSTALS FORMULAS - END-

# CONNECTORS, SOCKETS FORMULAS - START -

    def test_con_sock_pi_p_func(self):
        self.e_eq.con_sock_pi_p_func(5)
        result = self.e_eq.get_con_sock_pi_p()
        test_result = 2.012796909336014
        # result = math.exp(pow(((5-1) / 10), 0.39))

        self.assertAlmostEqual(result, test_result, 5)

# CONNECTORS, SOCKETS FORMULAS - END -

# CONNECTORS, GENERAL FORMULAS - START -

    def test_con_gen_pi_t_func(self):
        self.e_eq.con_gen_pi_t_func(35)
        result = self.e_eq.get_con_gen_pi_t()
        test_result = 1.1936465129494536
        # result = float(math.exp((-0.14 / (8.617 * (pow(10, -5)))) * ((1 / (35 + 273)) - (1 / 298))))

        self.assertAlmostEqual(result, test_result, 5)

# CONNECTORS, GENERAL FORMULAS - END -

# RELAYS FORMULAS - START -

    def test_rel_lambda_b_func_1(self):
        self.e_eq.rel_lambda_b_func(35, 85)
        result = self.e_eq.get_rel_lambda_b()
        test_result = 0.007502108907236194
        # result = float(0.0059 * float(math.exp((-0.19 / (8.617 * pow(10, -5))) * ((1 / (35 + 273)) - (1 / 298)))))

        self.assertAlmostEqual(result, test_result, 5)


    def test_rel_lambda_b_func_2(self):
        self.e_eq.rel_lambda_b_func(35, 125)
        result = self.e_eq.get_rel_lambda_b()
        test_result = 0.007314777440225106
        # result = float(0.0059 * float(math.exp((-0.17 / (8.617 * pow(10, -5))) * ((1 / (35 + 273)) - (1 / 298)))))

        self.assertAlmostEqual(result, test_result, 5)


    def test_rel_pi_l_func_1(self):
        self.e_eq.rel_pi_l_func(10, 1)
        result = self.e_eq.get_rel_pi_l()
        test_result = 157.25
        # result = float(pow((10 / 0.8), 2) + 1)

        self.assertAlmostEqual(result, test_result, 2)


    def test_rel_pi_l_func_2(self):
        self.e_eq.rel_pi_l_func(10, 2)
        result = self.e_eq.get_rel_pi_l()
        test_result = 626.00
        # result = float(pow((10 / 0.4), 2) + 1)

        self.assertAlmostEqual(result, test_result, 2)


    def test_rel_pi_l_func_3(self):
        self.e_eq.rel_pi_l_func(10, 3)
        result = self.e_eq.get_rel_pi_l()
        test_result = 2501.0
        # result = float(pow((10 / 0.2), 2) + 1)

        self.assertAlmostEqual(result, test_result, 2)


    def test_rel_pi_cyc_func_1(self):
        self.e_eq.rel_pi_cyc_func(10, 0)
        result = self.e_eq.get_rel_pi_cyc()
        test_result = 1.0

        self.assertAlmostEqual(result, test_result, 2)


    def test_rel_pi_cyc_func_2(self):
        self.e_eq.rel_pi_cyc_func(0.1, 0)
        result = self.e_eq.get_rel_pi_cyc()
        test_result = 0.1

        self.assertAlmostEqual(result, test_result, 2)


    def test_rel_pi_cyc_func_3(self):
        self.e_eq.rel_pi_cyc_func(1500, 1)
        result = self.e_eq.get_rel_pi_cyc()
        test_result = 225.0
        # result = float(pow(1500/100, 2))

        self.assertAlmostEqual(result, test_result, 2)


    def test_rel_pi_cyc_func_4(self):
        self.e_eq.rel_pi_cyc_func(150, 1)
        result = self.e_eq.get_rel_pi_cyc()
        test_result = 15.0
        # result = float(150/10)

        self.assertAlmostEqual(result, test_result, 2)


    def test_rel_pi_cyc_func_5(self):
        self.e_eq.rel_pi_cyc_func(5, 1)
        result = self.e_eq.get_rel_pi_cyc()
        test_result = 1.0

        self.assertAlmostEqual(result, test_result, 2)

# RELAYS FORMULAS - END -

# ROTATING DEVICES, MOTORS FORMULAS - START -

    def test_rd_alpha_b_func(self):
        self.e_eq.rd_alpha_b_func(35)
        result = self.e_eq.get_rd_alpha_b()
        test_result = 85595.978589058
        # result = float(pow(float(pow(10, float(2.534 - float(2357 / (35 + 273)))) + (1 / float(pow(10, float(20 - (4500 / float(35 + 273)))) + 300))), -1))

        self.assertAlmostEqual(result, test_result, 5)


    def test_rd_alpha_w_func(self):
        self.e_eq.rd_alpha_w_func(35)
        result = self.e_eq.get_rd_alpha_w()
        test_result = 664656.7239197979
        # result = float(pow(10, float(float(2357 / (35 + 273)) - 1.83)))

        self.assertAlmostEqual(result, test_result, 5)

# ROTATING DEVICES, MOTORS FORMULAS - END -

# RESISTOR FORMULAS - START -

    def test_res_pi_t_func_1(self):
        self.e_eq.res_pi_t_func(35, 1)
        result = self.e_eq.get_res_pi_t()
        test_result = 1.2877230710513001
        # result = float(math.exp(((-1 * 0.2) / (8.617 * pow(10, -5))) * ((1 / (35 + 273)) - (1 / 298))))

        self.assertAlmostEqual(result, test_result, 5)


    def test_res_pi_t_func_2(self):
        self.e_eq.res_pi_t_func(35, 2)
        result = self.e_eq.get_res_pi_t()
        test_result = 1.1064428601975629

        # result = float(math.exp(((-1 * 0.08) / (8.617 * pow(10, -5))) * ((1 / (35 + 273)) - (1 / 298))))

        self.assertAlmostEqual(result, test_result, 5)


    def test_res_pi_t_func_3(self):
        self.e_eq.res_pi_t_func(35, 3)
        result = self.e_eq.get_res_pi_t()
        test_result = 1.0

        self.assertAlmostEqual(result, test_result, 5)


    def test_calculate_res_pi_t_func(self):
        result = self.e_eq.calculate_res_pi_t_func(35, 1)
        test_result = 3.5408895754801635

        # result = float(math.exp(((-1 * 1) / (8.617 * pow(10, -5))) * ((1 / (35 + 273)) - (1 / 298))))

        self.assertAlmostEqual(result, test_result, 5)


    def test_res_pi_p_func(self):
        self.e_eq.res_pi_p_func(35)
        result = self.e_eq.get_res_pi_p()
        test_result = 4.001165701268361

        # result = float(pow(35, 0.39))

        self.assertAlmostEqual(result, test_result, 5)


    def test_res_pi_s_func_1(self):
        self.e_eq.res_pi_s_func(0.35, 1)
        result = self.e_eq.get_res_pi_s()
        test_result = 1.0434261682232124

        # result = 0.71 * float(math.exp(1.1 * 0.35))

        self.assertAlmostEqual(result, test_result, 5)


    def test_res_pi_s_func_2(self):
        self.e_eq.res_pi_s_func(0.35, 2)
        result = self.e_eq.get_res_pi_s()
        test_result = 1.1027574993576743

        # result = 0.54 * float(math.exp(2.04 * 0.35))

        self.assertAlmostEqual(result, test_result, 5)


    def test_res_pi_s_func_3(self):
        self.e_eq.res_pi_s_func(0.35, 3)
        result = self.e_eq.get_res_pi_s()
        test_result = 1.0

        self.assertAlmostEqual(result, test_result, 5)


# RESISTOR FORMULAS - END -

# CAPACITOR FORMULAS - START -

    def test_cc_pi_t_func_1(self):
        self.e_eq.cc_pi_t_func(35, 0)
        result = self.e_eq.get_cc_pi_t()
        test_result = 35.0

        self.assertAlmostEqual(result, test_result, 5)


    def test_cc_pi_t_func_2(self):
        self.e_eq.cc_pi_t_func(35, 1)
        result = self.e_eq.get_cc_pi_t()
        test_result = 1.2088345313598179

        # result = float(math.exp(((-1 * 0.15) / (8.617 * pow(10, -5))) * ((1 / (35 + 273)) - (1 / 298))))

        self.assertAlmostEqual(result, test_result, 5)


    def test_cc_pi_t_func_3(self):
        self.e_eq.cc_pi_t_func(35, 2)
        result = self.e_eq.get_cc_pi_t()
        test_result = 1.5566441151155237

        # result = float(math.exp(((-1 * 0.35) / (8.617 * pow(10, -5))) * ((1 / (35 + 273)) - (1 / 298))))

        self.assertAlmostEqual(result, test_result, 5)


    def test_cc_pi_t_func_4(self):
        self.e_eq.cc_pi_t_func(35, 3)
        result = self.e_eq.get_cc_pi_t()
        test_result = 1.0

        self.assertAlmostEqual(result, test_result, 5)


    def test_calculate_cc_pi_t_func(self):
        result = self.e_eq.calculate_cc_pi_t_func(35, 1)
        test_result = 3.5408895754801635

        # result = float(math.exp(((-1 * 1) / (8.617 * pow(10, -5))) * ((1 / (35 + 273)) - (1 / 298))))

        self.assertAlmostEqual(result, test_result, 5)


    def test_cc_pi_c_func_1(self):
        self.e_eq.cc_pi_c_func(35, 0)
        result = self.e_eq.get_cc_pi_c()
        test_result = 35.0

        self.assertAlmostEqual(result, test_result, 5)


    def test_cc_pi_c_func_2(self):
        self.e_eq.cc_pi_c_func(35, 1)
        result = self.e_eq.get_cc_pi_c()
        test_result = 1.3771020474505347

        # result = float(pow(35, 0.09))

        self.assertAlmostEqual(result, test_result, 5)


    def test_cc_pi_c_func_3(self):
        self.e_eq.cc_pi_c_func(35, 2)
        result = self.e_eq.get_cc_pi_c()
        test_result = 2.265351770958606

        # result = float(pow(35, 0.23))

        self.assertAlmostEqual(result, test_result, 5)


    def test_cc_pi_c_func_4(self):
        self.e_eq.cc_pi_c_func(35, 3)
        result = self.e_eq.get_cc_pi_c()
        test_result = 1.0

        self.assertAlmostEqual(result, test_result, 5)


    def test_cc_pi_v_func_1(self):
        self.e_eq.cc_pi_v_func(0.35, 0)
        result = self.e_eq.get_cc_pi_v()
        test_result = 0.35

        self.assertAlmostEqual(result, test_result, 5)


    def test_cc_pi_v_func_2(self):
        self.e_eq.cc_pi_v_func(0.35, 1)
        result = self.e_eq.get_cc_pi_v()
        test_result = 1.0675435635288066

        # result = float(pow((0.35 / 0.6), 5) + 1)

        self.assertAlmostEqual(result, test_result, 5)


    def test_cc_pi_v_func_3(self):
        self.e_eq.cc_pi_v_func(0.35, 2)
        result = self.e_eq.get_cc_pi_v()
        test_result = 1.0045621329741699

        # result = float(pow((0.35 / 0.6), 10) + 1)

        self.assertAlmostEqual(result, test_result, 5)


    def test_cc_pi_v_func_4(self):
        self.e_eq.cc_pi_v_func(0.35, 3)
        result = self.e_eq.get_cc_pi_v()
        test_result = 1.1984953703703705

        # result = float(pow((0.35 / 0.6), 3) + 1)

        self.assertAlmostEqual(result, test_result, 5)


    def test_cc_pi_v_func_5(self):
        self.e_eq.cc_pi_v_func(0.35, 4)
        result = self.e_eq.get_cc_pi_v()
        test_result = 1.0001048541194446

        # result = float(pow((0.35 / 0.6), 17) + 1)

        self.assertAlmostEqual(result, test_result, 5)


    def test_cc_pi_v_func_6(self):
        self.e_eq.cc_pi_v_func(0.35, 5)
        result = self.e_eq.get_cc_pi_v()
        test_result = 1.1984953703703705

        # result = float(pow((0.35 / 0.6), 3) + 1)

        self.assertAlmostEqual(result, test_result, 5)


    def test_cc_pi_v_func_7(self):
        self.e_eq.cc_pi_v_func(0.35, 6)
        result = self.e_eq.get_cc_pi_v()
        test_result = 1.0

        self.assertAlmostEqual(result, test_result, 5)


    def test_cc_pi_sr_func_1(self):
        self.e_eq.cc_pi_sr_func("None")
        result = self.e_eq.get_cc_pi_sr()
        test_result = 1.0

        self.assertAlmostEqual(result, test_result, 2)


    def test_cc_pi_sr_func_2(self):
        self.e_eq.cc_pi_sr_func("cr_g_0_8")
        result = self.e_eq.get_cc_pi_sr()
        test_result = 0.66

        self.assertAlmostEqual(result, test_result, 2)


    def test_cc_pi_sr_func_3(self):
        self.e_eq.cc_pi_sr_func("cr_g_0_6_to_0_8")
        result = self.e_eq.get_cc_pi_sr()
        test_result = 1.0

        self.assertAlmostEqual(result, test_result, 2)


    def test_cc_pi_sr_func_4(self):
        self.e_eq.cc_pi_sr_func("cr_g_0_4_to_0_6")
        result = self.e_eq.get_cc_pi_sr()
        test_result = 1.3

        self.assertAlmostEqual(result, test_result, 2)


    def test_cc_pi_sr_func_5(self):
        self.e_eq.cc_pi_sr_func("cr_g_0_2_to_0_4")
        result = self.e_eq.get_cc_pi_sr()
        test_result = 2.0

        self.assertAlmostEqual(result, test_result, 2)


    def test_cc_pi_sr_func_6(self):
        self.e_eq.cc_pi_sr_func("cr_g_0_1_to_0_2")
        result = self.e_eq.get_cc_pi_sr()
        test_result = 2.7

        self.assertAlmostEqual(result, test_result, 2)


    def test_cc_pi_sr_func_7(self):
        self.e_eq.cc_pi_sr_func("cr_0_to_0_1")
        result = self.e_eq.get_cc_pi_sr()
        test_result = 3.3

        self.assertAlmostEqual(result, test_result, 2)


    def test_cc_pi_sr_func_8(self):
        self.e_eq.cc_pi_sr_func(35)
        result = self.e_eq.get_cc_pi_sr()
        test_result = 35.0

        self.assertAlmostEqual(result, test_result, 2)


# CAPACITOR FORMULAS - END -

# DIODE FORMULAS - START -

    def test_dd_pi_t_func_1(self):
        self.e_eq.dd_pi_t_func(35, 0)
        result = self.e_eq.get_dd_pi_t()
        test_result = 35.0

        self.assertAlmostEqual(result, test_result, 2)


    def test_dd_pi_t_func_2(self):
        self.e_eq.dd_pi_t_func(35, 1)
        result = self.e_eq.get_dd_pi_t()
        test_result = 1.2333469156667014

        # result = float(math.exp((-1 * 1925) * ((1 / (35 + 273)) - (1 / 298))))

        self.assertAlmostEqual(result, test_result, 5)


    def test_dd_pi_t_func_3(self):
        self.e_eq.dd_pi_t_func(35, 2)
        result = self.e_eq.get_dd_pi_t()
        test_result = 1.4004154404247757

        # result = float(math.exp((-1 * 3091) * ((1 / (35 + 273)) - (1 / 298))))

        self.assertAlmostEqual(result, test_result, 5)


    def test_dd_pi_t_func_4(self):
        self.e_eq.dd_pi_t_func(35, 3)
        result = self.e_eq.get_dd_pi_t()
        test_result = 1.0

        self.assertAlmostEqual(result, test_result, 2)


    def test_calculate_dd_pi_t_func(self):
        result = self.e_eq.calculate_dd_pi_t_func(35, 1925)
        test_result = 1.2333469156667014

        # result = float(math.exp((-1 * 1925) * ((1 / (35 + 273)) - (1 / 298))))

        self.assertAlmostEqual(result, test_result, 5)


    def test_dd_pi_s_func_1(self):
        self.e_eq.dd_pi_s_func(0.25, 1)
        result = self.e_eq.get_dd_pi_s()
        test_result = 0.054

        self.assertAlmostEqual(result, test_result, 3)


    def test_dd_pi_s_func_2(self):
        self.e_eq.dd_pi_s_func(0.35, 1)
        result = self.e_eq.get_dd_pi_s()
        test_result = 0.07799833781904994

        # result = float(pow(0.35, 2.43))

        self.assertAlmostEqual(result, test_result, 5)


    def test_dd_pi_s_func_3(self):
        self.e_eq.dd_pi_s_func(35, 2)
        result = self.e_eq.get_dd_pi_s()
        test_result = 35.0

        self.assertAlmostEqual(result, test_result, 2)


    def test_dd_pi_s_func_4(self):
        self.e_eq.dd_pi_s_func(35, 3)
        result = self.e_eq.get_dd_pi_s()
        test_result = 1.0

        self.assertAlmostEqual(result, test_result, 2)

# DIODE FORMULAS - END -


# INDUCTOR FORMULAS - START -

    def test_id_pi_t_func_1(self):
        self.e_eq.id_pi_t_func(35, 0)
        result = self.e_eq.get_id_pi_t()
        test_result = 35.0

        self.assertAlmostEqual(result, test_result, 2)


    def test_id_pi_t_func_2(self):
        self.e_eq.id_pi_t_func(35, 1)
        result = self.e_eq.get_id_pi_t()
        test_result = 1.1492178478655128

        # result = float(math.exp(((-1 * 0.11) / (8.617 * pow(10, -5))) * ((1 / (35 + 273)) - (1 / 298))))
        

        self.assertAlmostEqual(result, test_result, 5)


    def test_id_pi_t_func_3(self):
        self.e_eq.id_pi_t_func(35, 2)
        result = self.e_eq.get_id_pi_t()
        test_result = 1.0

        self.assertAlmostEqual(result, test_result, 2)

# INDUCTOR FORMULAS - END -

# TRANSISTOR FORMULAS - START -

    def test_ts_pi_t_func_1(self):
        self.e_eq.ts_pi_t_func(35, 0)
        result = self.e_eq.get_ts_pi_t()
        test_result = 35.0

        self.assertAlmostEqual(result, test_result, 2)


    def test_ts_pi_t_func_2(self):
        self.e_eq.ts_pi_t_func(35, 1)
        result = self.e_eq.get_ts_pi_t()
        test_result = 1.2333469156667014

        # result = float(math.exp((-1 * 1925) * ((1 / (35 + 273)) - (1 / 298))))

        self.assertAlmostEqual(result, test_result, 5)


    def test_ts_pi_t_func_3(self):
        self.e_eq.ts_pi_t_func(35, 2)
        result = self.e_eq.get_ts_pi_t()
        test_result = 1.0

        self.assertAlmostEqual(result, test_result, 2)

# TRANSISTOR FORMULAS - END -

if __name__ == '__main__':
	rosunit.unitrun(PKG, NAME, TestElectricalEquipment, sysargs = "--cov", coverage_packages=[str(PKG)])
   