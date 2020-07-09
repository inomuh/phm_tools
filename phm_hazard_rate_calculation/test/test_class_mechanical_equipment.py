#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import unittest
import math
import rosunit
from phm_hazard_rate_calculation.class_mechanical_equipment import MechanicalEquipment

PKG = 'phm_hazard_rate_calculation'
NAME = 'test_class_mechanical_equipment'


class TestMechanicalEquipment(unittest.TestCase):
    m_eq = MechanicalEquipment()

# Get FUNCTIONS - START -

    def test_get_function_1(self):
        result = self.m_eq.get_sp_lambda()
        test_result = 0.0

        self.assertAlmostEqual(result, test_result, 5)

    def test_get_function_2(self):
        result = self.m_eq.get_gr_lambda()
        test_result = 0.0

        self.assertAlmostEqual(result, test_result, 5)

    def test_get_function_3(self):
        result = self.m_eq.get_be_lambda()
        test_result = 0.0

        self.assertAlmostEqual(result, test_result, 5)


    def test_get_function_4(self):
        result = self.m_eq.get_ac_lambda()
        test_result = 0.0

        self.assertAlmostEqual(result, test_result, 5)


    def test_get_function_5(self):
        result = self.m_eq.get_sh_lambda()
        test_result = 0.0

        self.assertAlmostEqual(result, test_result, 5)


    def test_get_function_6(self):
        result = self.m_eq.get_em_lambda()
        test_result = 1.0

        self.assertAlmostEqual(result, test_result, 5)


    def test_get_function_7(self):
        result = self.m_eq.get_cp_lambda()
        test_result = 0.0

        self.assertAlmostEqual(result, test_result, 5)


    def test_get_function_8(self):
        result = self.m_eq.get_bat_lambda()
        test_result = 0.0

        self.assertAlmostEqual(result, test_result, 5)


# ATTRIBUTES - END -

# SPRING FORMULAS - START -

    def test_sp_c_dw_func(self):
        self.m_eq.sp_c_dw_func(0.5)
        result = self.m_eq.get_sp_c_dw()
        test_result = 203.54162426216158

        # result = float(pow((0.5 / 0.085), 3))

        self.assertAlmostEqual(result, test_result, 5)


    def test_sp_c_dc_func(self):
        self.m_eq.sp_c_dc_func(0.5)
        result = self.m_eq.get_sp_c_dc()
        test_result = 2.436396322815999

        # result = float(pow(0.58 / 0.5, 6))

        self.assertAlmostEqual(result, test_result, 5)


    def test_sp_c_n_func(self):
        self.m_eq.sp_c_n_func(5)
        result = self.m_eq.get_sp_c_n()
        test_result = 21.951999999999995

        # result = float(pow(14 / 5, 3))

        self.assertAlmostEqual(result, test_result, 5)


    def test_sp_c_l_func(self):
        self.m_eq.sp_c_l_func(7, 5)
        result = self.m_eq.get_sp_c_l()
        test_result = 6.530383015126816

        # result = float(pow((7 - 5) / 1.07, 3))

        self.assertAlmostEqual(result, test_result, 5)


    def test_sp_c_k_func(self):
        self.m_eq.sp_c_k_func(9, 5)
        result = self.m_eq.get_sp_c_k()
        test_result = 0.7314342657282276

        # r = float(9 / 5)
        # k_w = float(((4 * r - 1) / (4 * r + 1)) + (0.616 / r))
        # result = float(pow(k_w / 1.219, 3))

        self.assertAlmostEqual(result, test_result, 5)


    def test_sp_c_cs_func_1(self):
        self.m_eq.sp_c_cs_func(25)
        result = self.m_eq.get_sp_c_cs()
        test_result = 0.1

        self.assertAlmostEqual(result, test_result, 2)


    def test_sp_c_cs_func_2(self):
        self.m_eq.sp_c_cs_func(250)
        result = self.m_eq.get_sp_c_cs()
        test_result = 0.8333333333333334

        # result = float(250 / 300)

        self.assertAlmostEqual(result, test_result, 5)


    def test_sp_c_cs_func_3(self):
        self.m_eq.sp_c_cs_func(350)
        result = self.m_eq.get_sp_c_cs()
        test_result = 1.5879629629629632

        # result = float(pow(350 / 300, 3))

        self.assertAlmostEqual(result, test_result, 5)


# SPRING FORMULAS - END -

# BEARING FORMULAS - START -
    def test_bearing_l_10_func(self):
        result = self.m_eq.bearing_l_10_func(2, 30, 6, 5)
        test_result = 83333.33333333334

        # result = float((pow(10, 6) / (60 * 5)) * pow((30 / 6), 2))

        self.assertAlmostEqual(result, test_result, 5)


    def test_be_lambda_be_b_func_1(self):
        self.m_eq.be_lambda_be_b_func(2, 30, 6, 5, 10)
        result = self.m_eq.get_be_lambda_be_b()
        test_result = 1.1999999999999997e-06

        # l_10 = float((pow(10, 6) / (60 * 5)) * pow((30 / 6), 2))
        # result = float(1 / (l_10 * 10))

        self.assertAlmostEqual(result, test_result, 5)


    def test_be_lambda_be_b_func_2(self):
        self.m_eq.be_lambda_be_b_func(2, 30, 6, None, 10)
        result = self.m_eq.get_be_lambda_be_b()
        test_result = 1.0

        self.assertAlmostEqual(result, test_result, 5)


    def test_be_c_y_func(self):
        self.m_eq.be_c_y_func(2, 30, 6)
        result = self.m_eq.get_be_c_y()
        test_result = 25.0

        # result = float(pow((30 / 6), 2))

        self.assertAlmostEqual(result, test_result, 5)


    def test_be_c_r_func_1(self):
        self.m_eq.be_c_r_func(5, True)
        result = self.m_eq.get_be_c_r()
        test_result = 0.10730901231734431

        # result = float(float(0.223) / float(pow(math.log(float(100) / float(5)), (2/3))))

        self.assertAlmostEqual(result, test_result, 5)


    def test_be_c_r_func_2(self):
        self.m_eq.be_c_r_func(5, False)
        result = self.m_eq.get_be_c_r()
        test_result = 1.0

        self.assertAlmostEqual(result, test_result, 2)


    def test_be_c_v_func_1(self):
        self.m_eq.be_c_v_func(30, 5, True)
        result = self.m_eq.get_be_c_v()
        test_result = 2.6314897044644194

        # result = float(pow((30 / 5), (0.54)))

        self.assertAlmostEqual(result, test_result, 5)


    def test_be_c_v_func_2(self):
        self.m_eq.be_c_v_func(30, 5, False)
        result = self.m_eq.get_be_c_v()
        test_result = 1.0

        self.assertAlmostEqual(result, test_result, 2)


    def test_be_c_cw_func_1(self):
        self.m_eq.be_c_cw_func(0.9, True)
        result = self.m_eq.get_be_c_cw()
        test_result = 11.0

        self.assertAlmostEqual(result, test_result, 2)


    def test_be_c_cw_func_2(self):
        self.m_eq.be_c_cw_func(0.5, True)
        result = self.m_eq.get_be_c_cw()
        test_result = 9.6875

        # result = float(1.0 + (25.50 * 0.5) - (16.25 * pow(0.5, 2)))

        self.assertAlmostEqual(result, test_result, 5)


    def test_be_c_cw_func_3(self):
        self.m_eq.be_c_cw_func(0.9, False)
        result = self.m_eq.get_be_c_cw()
        test_result = 1.0

        self.assertAlmostEqual(result, test_result, 2)


    def test_be_c_t_func_1(self):
        self.m_eq.be_c_t_func(150)
        result = self.m_eq.get_be_c_t()
        test_result = 1.0

        self.assertAlmostEqual(result, test_result, 2)


    def test_be_c_t_func_2(self):
        self.m_eq.be_c_t_func(200)
        result = self.m_eq.get_be_c_t()
        test_result = 1.3053792885584978

        # result = float(pow((200 / 183), 3))

        self.assertAlmostEqual(result, test_result, 5)

# BEARING FORMULAS - END -

# GEAR FORMULAS - START -

    def test_gr_lambda_gr_b_func_1(self):
        self.m_eq.gr_lambda_gr_b_func(100, 10, True)
        result = self.m_eq.get_gr_lambda_gr_b()
        test_result = 600.0

        # result = float(100 * 60 * (1 / revolutions))

        self.assertAlmostEqual(result, test_result, 5)


    def test_gr_lambda_gr_b_func_2(self):
        self.m_eq.gr_lambda_gr_b_func(1, 1, False)
        result = self.m_eq.get_gr_lambda_gr_b()
        test_result = 1.0

        self.assertAlmostEqual(result, test_result, 5)


    def test_c_gs_func_1(self):
        self.m_eq.c_gs_func(100, 10, True)
        result = self.m_eq.get_gr_c_gs()
        test_result = 6.011872336272722

        # result = float(1.0 + pow((100 / 10), 0.7))

        self.assertAlmostEqual(result, test_result, 5)


    def test_c_gs_func_2(self):
        self.m_eq.c_gs_func(100, 10, False)
        result = self.m_eq.get_gr_c_gs()
        test_result = 1.0

        self.assertAlmostEqual(result, test_result, 5)


    def test_c_gp_func_1(self):
        self.m_eq.c_gp_func(100, 10, True)
        result = self.m_eq.get_gr_c_gp()
        test_result = 1264243.85515232

        # result = float(0.5 + pow(((100 / 10) / 0.5), 4.69))

        self.assertAlmostEqual(result, test_result, 5)


    def test_c_gp_func_2(self):
        self.m_eq.c_gp_func(100, 10, False)
        result = self.m_eq.get_gr_c_gp()
        test_result = 1.0

        self.assertAlmostEqual(result, test_result, 5)


    def test_c_ga_func_1(self):
        self.m_eq.c_ga_func(1, True)
        result = self.m_eq.get_gr_c_ga()
        test_result = 175211.67808855648

        # result = float(pow((1 / 0.006), 2.36))

        self.assertAlmostEqual(result, test_result, 5)


    def test_c_ga_func_2(self):
        self.m_eq.c_ga_func(1, False)
        result = self.m_eq.get_gr_c_ga()
        test_result = 1.0

        self.assertAlmostEqual(result, test_result, 5)


    def test_c_gl_func_1(self):
        self.m_eq.c_gl_func(100, 10, True)
        result = self.m_eq.get_gr_c_gl()
        test_result = 4.467368504525316

        # result = float(1.0 + pow((100 / 10), 0.54))

        self.assertAlmostEqual(result, test_result, 5)


    def test_c_gl_func_2(self):
        self.m_eq.c_gl_func(100, 10, False)
        result = self.m_eq.get_gr_c_gl()
        test_result = 1.0

        self.assertAlmostEqual(result, test_result, 5)


    def test_c_gt_func_1(self):
        self.m_eq.c_gt_func(150)
        result = self.m_eq.get_gr_c_gt()
        test_result = 1.0

        self.assertAlmostEqual(result, test_result, 5)


    def test_c_gt_func_2(self):
        self.m_eq.c_gt_func(200)
        result = self.m_eq.get_gr_c_gt()
        test_result = 1.064516129032258

        # result = float((460 + 200) / 620)

        self.assertAlmostEqual(result, test_result, 5)

# GEAR FORMULAS - END -

# ACTUATOR FORMULAS - START -

    def test_ac_n_o_func(self):
        self.m_eq.ac_n_o_func(0.5, 0.1, 0.5, 0.1, 0.5, 0.4, 0.2)
        result = self.m_eq.ac_n_o
        test_result = 2.320515320283925

        self.assertAlmostEqual(result, test_result, 5)


    def test_ac_lambda_ac_b_func(self):
        self.m_eq.ac_n_o = 2
        self.m_eq.ac_lambda_ac_b_func()
        result = self.m_eq.get_ac_lambda_ac_b()
        test_result = 500000.0

        self.assertAlmostEqual(result, test_result, 5)


    def test_ac_c_h_func_1(self):
        self.m_eq.ac_c_h_func(10, 5, True)
        result = self.m_eq.get_ac_c_h()
        test_result = 2.0

        # result = float(10 / 5)

        self.assertAlmostEqual(result, test_result, 5)


    def test_ac_c_h_func_2(self):
        self.m_eq.ac_c_h_func(10, 5, False)
        result = self.m_eq.get_ac_c_h()
        test_result = 1.0

        self.assertAlmostEqual(result, test_result, 5)


    def test_ac_c_s_func(self):
        self.m_eq.ac_c_s_func(150)
        result = self.m_eq.ac_c_s
        test_result = 15.0

        # result = float(150 / 10)

        self.assertAlmostEqual(result, test_result, 5)


    def test_ac_c_cp_func_1(self):
        self.m_eq.ac_c_h = 5
        self.m_eq.ac_c_s = 5
        self.m_eq.ac_c_n = 2
        self.m_eq.ac_c_cp_func(True)
        result = self.m_eq.get_ac_c_cp()
        test_result = 50.0

        # result = float(5 * 5 * 2)

        self.assertAlmostEqual(result, test_result, 5)


    def test_ac_c_cp_func_2(self):
        self.m_eq.ac_c_cp_func(False)
        result = self.m_eq.get_ac_c_cp()
        test_result = 1.0

        self.assertAlmostEqual(result, test_result, 5)


# ACTUATOR FORMULAS - END -

# ELECTRIC MOTORS FORMULAS - START -

    def test_elec_mot_lambda_wi_b_func(self):
        self.m_eq.elec_mot_lambda_wi_b_func(150)
        result = self.m_eq.get_em_lambda_wi_b()
        test_result = 6666.666666666667

        # result = float((1.0 * pow(10, 6)) / 150)

        self.assertAlmostEqual(result, test_result, 5)


    def test_elec_mot_c_t_func_1(self):
        self.m_eq.elec_mot_c_t_func(80)
        result = self.m_eq.get_em_c_t()
        test_result = 16.0

        # result = float(pow(2, ((float(80) - 40) / 10)))

        self.assertAlmostEqual(result, test_result, 5)


    def test_elec_mot_c_t_func_2(self):
        self.m_eq.elec_mot_c_t_func(None)
        result = self.m_eq.get_em_c_t()
        test_result = 1.0

        self.assertAlmostEqual(result, test_result, 5)


    def test_elec_mot_c_v_func_1(self):
        self.m_eq.em_motor_phase = 1
        self.m_eq.elec_mot_c_v_func(25, 5, 5, True)
        result = self.m_eq.get_em_c_v()
        test_result = 1125899906842624.0

        # result = pow(2, 10 * (float(25) / float(5)))

        self.assertAlmostEqual(result, test_result, 5)


    def test_elec_mot_c_v_func_2(self):
        self.m_eq.em_motor_phase = 3
        self.m_eq.elec_mot_c_v_func(25, 5, 5, True)
        result = self.m_eq.get_em_c_v()
        test_result = 6.656854249492381

        # result = float(1 + pow(0.40 * float(5), 2.5))

        self.assertAlmostEqual(result, test_result, 5)


    def test_elec_mot_c_v_func_3(self):
        self.m_eq.em_motor_phase = 0
        self.m_eq.elec_mot_c_v_func(25, 5, 5, True)
        result = self.m_eq.get_em_c_v()
        test_result = 1.0

        self.assertAlmostEqual(result, test_result, 5)


    def test_elec_mot_c_v_func_4(self):
        self.m_eq.elec_mot_c_v_func(25, 5, 5, False)
        result = self.m_eq.get_em_c_v()
        test_result = 1.0

        self.assertAlmostEqual(result, test_result, 5)


    def test_elec_mot_v_u_func_1(self):
        self.m_eq.elec_mot_v_u_func(25, 5, True)
        result = self.m_eq.get_em_v_u()
        test_result = 500.0

        # result = float(100 * (25 / 5))

        self.assertAlmostEqual(result, test_result, 5)


    def test_elec_mot_v_u_func_2(self):
        self.m_eq.elec_mot_v_u_func(25, 5, False)
        result = self.m_eq.get_em_v_u()
        test_result = 1.0

        self.assertAlmostEqual(result, test_result, 5)


    def test_elec_mot_alt_func_1(self):
        self.m_eq.elec_mot_alt_func(3000)
        result = self.m_eq.get_em_c_alt()
        test_result = 1.0

        self.assertAlmostEqual(result, test_result, 5)


    def test_elec_mot_alt_func_2(self):
        self.m_eq.elec_mot_alt_func(3750)
        result = self.m_eq.get_em_c_alt()
        test_result = 1.036

        # result = float(1.00 + (8 * pow(10, -5)) * (3750 - 3300))

        self.assertAlmostEqual(result, test_result, 5)


    def test_elec_mot_lambda_c_func(self):
        self.m_eq.elec_mot_lambda_c_func(35)
        result = self.m_eq.get_em_lambda_c()
        test_result = 35.0

        self.assertAlmostEqual(result, test_result, 5)

# ELECTRIC MOTORS FORMULAS - END -

# SHAFTS MOTORS FORMULAS - START -

    def test_sh_lambda_sh_b_func(self):
        self.m_eq.sh_lambda_sh_b_func(35)
        result = self.m_eq.get_sh_lambda_sh_b()
        test_result = 0.02857142857142857

        # result = float(1 / 35)

        self.assertAlmostEqual(result, test_result, 5)


    def test_sh_c_f_func_1(self):
        self.m_eq.sh_c_f_func(35, 0)
        result = self.m_eq.get_sh_c_f()
        test_result = 35.0

        self.assertAlmostEqual(result, test_result, 5)


    def test_sh_c_f_func_2(self):
        self.m_eq.sh_c_f_func(35, 1)
        result = self.m_eq.get_sh_c_f()
        test_result = 0.7892532499999999

        # result = float(0.94 - (0.0046 * 35) + 8.37 * pow(10, -6) * pow(35, 2))

        self.assertAlmostEqual(result, test_result, 5)


    def test_sh_c_f_func_3(self):
        self.m_eq.sh_c_f_func(35, 2)
        result = self.m_eq.get_sh_c_f()
        test_result = 0.9170418625000001

        # result = float(1.07 - 0.0051 * 35 + 2.21 * pow(10, -5) * pow(35, 2) - 3.57 * pow(10, -8) * pow(35, 3))

        self.assertAlmostEqual(result, test_result, 5)


    def test_sh_c_f_func_4(self):
        self.m_eq.sh_c_f_func(35, 3)
        result = self.m_eq.get_sh_c_f()
        test_result = 0.6171854999999999

        # result = float(0.75 - 4.06 * pow(10, -3) * 35 + 7.58 * pow(10, -6) * pow(35, 2))

        self.assertAlmostEqual(result, test_result, 5)


    def test_sh_c_f_func_5(self):
        self.m_eq.sh_c_f_func(35, 4)
        result = self.m_eq.get_sh_c_f()
        test_result = 1.0

        self.assertAlmostEqual(result, test_result, 5)


    def test_sh_c_f_hot_rolled(self):
        result = self.m_eq.sh_c_f_hot_rolled(35)
        test_result = 0.7892532499999999

        # result = float(0.94 - (0.0046 * 35) + 8.37 * pow(10, -6) * pow(35, 2))

        self.assertAlmostEqual(result, test_result, 5)


    def test_sh_c_f_macined(self):
        result = self.m_eq.sh_c_f_macined(35)
        test_result = 0.9170418625000001

        # result = float(1.07 - 0.0051 * 35 + 2.21 * pow(10, -5) * pow(35, 2) - 3.57 * pow(10, -8) * pow(35, 3))

        self.assertAlmostEqual(result, test_result, 5)


    def test_sh_c_f_forged(self):
        result = self.m_eq.sh_c_f_forged(35)
        test_result = 0.6171854999999999

        # result = float(0.75 - 4.06 * pow(10, -3) * 35 + 7.58 * pow(10, -6) * pow(35, 2))

        self.assertAlmostEqual(result, test_result, 5)


    def test_sh_c_t_func_1(self):
        self.m_eq.sh_c_t_func(150)
        result = self.m_eq.get_sh_c_t()
        test_result = 1.0

        self.assertAlmostEqual(result, test_result, 5)


    def test_sh_c_t_func_2(self):
        self.m_eq.sh_c_t_func(200)
        result = self.m_eq.get_sh_c_t()
        test_result = 1.064516129032258

        # result = float((460 + 200) / 620)

        self.assertAlmostEqual(result, test_result, 5)


    def test_sh_c_dy_func(self):
        section_dict = {"X": {'length': 3, 'I': 5}, "Y": {'length': 4, 'I': 3}}
        self.m_eq.sh_c_dy_func(3, 4, 5, section_dict)
        result = self.m_eq.get_sh_c_dy()
        test_result = 0.030654222222222225

        self.assertAlmostEqual(result, test_result, 5)


    def test_shaft_moment_of_inertia_func(self):
        result = self.m_eq.shaft_moment_of_inertia_func(3)
        test_result = 0.5890486225480862

        # result = float((math.pi * 3 * 4) / 64)

        self.assertAlmostEqual(result, test_result, 5)


    def test_sh_c_sc_r_func_1(self):
        self.m_eq.sh_c_sc_r_func(10, 5, 2, True)
        result = self.m_eq.get_sh_c_sc_r()
        test_result = 0.014583783731925789

        # result = float(pow((0.3) / (10 / 2), (0.2)) * pow((5 / 2), ((1.0) - (10 / 2))))

        self.assertAlmostEqual(result, test_result, 5)


    def test_sh_c_sc_r_func_2(self):
        self.m_eq.sh_c_sc_r_func(10, 5, 2, False)
        result = self.m_eq.get_sh_c_sc_r()
        test_result = 1.0

        self.assertAlmostEqual(result, test_result, 5)


    def test_sh_c_sc_func(self):
        self.m_eq.sh_c_sc_r_func(10, 5, 2, True)
        self.m_eq.sh_c_sc_g = 5
        self.m_eq.sh_c_sc_func()
        result = self.m_eq.get_sh_c_sc()
        test_result = 5.014583783731925789

        self.assertAlmostEqual(result, test_result, 5)

# SHAFTS MOTORS FORMULAS - END -

# MECHANICAL COUPLINGS MOTORS FORMULAS - START -

    def test_lambda_cp_func(self):
        self.m_eq.cp_lambda_cp_b = 5
        self.m_eq.cp_c_sf = 4
        self.m_eq.cp_lambda_h = 3
        self.m_eq.lambda_cp_func()
        result = self.m_eq.get_cp_lambda()
        test_result = 2.3e-05

        self.assertAlmostEqual(result, test_result, 5)

# MECHANICAL COUPLINGS MOTORS FORMULAS - END -

# BATTERY FORMULAS - START -

    def test_lambda_bat_func(self):
        self.m_eq.bat_lambda_0 = 5
        self.m_eq.lambda_bat_func()
        result = self.m_eq.get_bat_lambda()
        test_result = 5e-09

        # result = float(5 * pow(10, -9))

        self.assertAlmostEqual(result, test_result, 5)

# BATTERY FORMULAS - END -


if __name__ == '__main__':
    rosunit.unitrun(PKG, NAME, TestMechanicalEquipment, sysargs = "--cov", coverage_packages=[str(PKG)])
