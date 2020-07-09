#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""

    PHM Gui Mechanical Equipment Class

"""

import math

class MechanicalEquipment:
    """
        λM = (λM,B * CSF) + λWI + λBS + λST + λAS + λBE + λGR + λC

        λM = Total failure rate for the motor system, failures/million hours

        λM,B = Base failure rate of motor, failures/million hours

        CSF = Motor load service factor

        λWI = Failure rate of electric motor windings, failures/million hours

        λBS = Failure rate of brushes, 3.2 failures/million hours/brush

        λST = Failure rate of the stator housing, 0.001 failures/million hours

        λSH  = Failure rate of the armature shaft, failures/million hours

        λBE = Failure rate of bearings, failures/million hours

        λGR = Failure rate of gears, failures/million hours

        λC = Failure rate of capacitor, failures/million hours
    """

    def __init__(self):
       # Private System Attribute
        self.__lambda_sp = 0.0          # spring
        self.__lambda_gr = 0.0          # gears
        self.__lambda_be = 0.0          # bearing
        self.__lambda_ac = 0.0          # actuator
        self.__lambda_sh = 0.0          # shaft
        self.__electric_motor_system_failure_rate = 1.0
        self.__lambda_cp = 0.0          # mechanical coupling
        self.__lambda_bat = 0.0         # battery

        self.__em_lambda_c = 0.0        # capacitor

        # Public System Attribute
        self.lambda_m_b = 1.0
        self.c_sf = 1.0

# COMPONENTS' ATTRIBUTES - START -
     # SPRING ATTRIBUTES - START -
        self.sp_lambda_sp_b = 23.8
        self.sp_c_g = 1.0
        self.sp_c_y = 1.0
        self.sp_c_m = 1.0
        self.sp_c_r = 1.0

        self.sp_c_dw = 1.0
        self.sp_c_dc = 1.0
        self.sp_c_n = 1.0
        self.sp_c_l = 1.0
        self.sp_c_k = 1.0
        self.__sp_c_cs = 1.0
     # SPRING ATTRIBUTES - END -

     # ELECTRIC MOTORS ATTRIBUTES - START -
       # Private Attributes
        self.__em_lambda_m_b = 1.0
        self.__em_c_sf = 1.0
        self.__em_lambda_wi_b = 1.0
        self.__em_c_t = 1.0
        self.__em_c_v = 1.0
        self.__em_v_u = 0.0
        self.__em_c_alt = 1.0

        self.__em_lambda_bs = 3.2       # constant
        self.__em_lambda_st = 0.001     # constant
        self.__em_lambda_wi = 0.0

       # Public Attributes
        self.em_motor_phase = 0
        self.em_v_d = 0.0
        self.em_v_r = 1.0

     # ELECTRIC MOTORS ATTRIBUTES - END -

     # SHAFT ATTRIBUTES - START -
       # Private Shaft Attribute
        self.__sh_lambda_sh_b = 1.0
        self.__sh_c_t = 1.0
        self.__sh_c_dy = 1.0
        self.__sh_c_sc = 1.0
        self.__sh_c_sc_r = 0.0
        self.__sh_c_f = 1.0

       # Public Shaft Attribute
        self.sh_c_sc_g = 1.0
     # SHAFT ATTRIBUTES - END -

     # BEARING ATTRIBUTES - START -
       # Private Bearing Attribute
        self.__be_lambda_be_b = 1.0
        self.__be_c_y = 1.0
        self.__be_c_r = 1.0
        self.__be_c_v = 1.0
        self.__be_c_cw = 1.0
        self.__be_c_t = 1.0
       # Public Bearing Attribute
        self.be_c_sf = 1.0
        self.be_c_c = 1.0
     # BEARING ATTRIBUTES - END -

     # GEAR ATTRIBUTES - START -
      # Private Gear Attribute
        self.__gr_lambda_gr_b = 1.0
        self.__gr_c_gs = 1.0
        self.__gr_c_gp = 1.0
        self.__gr_c_ga = 1.0
        self.__gr_c_gl = 1.0
        self.__gr_c_gt = 1.0
      # Public Gear Attribute
        self.gr_c_gv = 1.0
     # GEAR ATTRIBUTES - END -

     # ACTUATOR ATTRIBUTES - START -
        self.__ac_lambda_ac_b = 1.0
        self.__ac_c_cp = 1.0
        self.__ac_c_t = 1.0

        self.ac_gama = 1.0
        self.ac_k_2 = float(17.7 * pow(10, 3))
        self.ac_f_y = 1.0
        self.ac_n_o = 1.0
        self.ac_c_h = 1.0

        self.ac_c_s = 1.0
        self.ac_c_n = 1.0

        self.ac_t_a = 25.2
     # ACTUATOR ATTRIBUTES - END -

     # MECHANICAL COUPLINGS ATTRIBUTES - START -
        self.cp_lambda_cp_b = 5.0
        self.__lambda_se = 0.0
        self.cp_c_sf = 1.0
        self.cp_lambda_h = 0.001
     # MECHANICAL COUPLINGS ATTRIBUTES - END -

     # BATTERY ATTRIBUTES - START -
        self.bat_lambda_0 = 1.0
     # BATTERY ATTRIBUTES - START -
# COMPONENTS' ATTRIBUTES - END -

    # Get Main Functions
    def get_sp_lambda(self):
        """
            get __lambda_sp attribute function
        """
        return self.__lambda_sp


    def get_gr_lambda(self):
        """
            get __lambda_gr attribute function
        """
        return self.__lambda_gr


    def get_be_lambda(self):
        """
            get __lambda_be attribute function
        """
        return self.__lambda_be


    def get_ac_lambda(self):
        """
            get __lambda_ac attribute function
        """
        return self.__lambda_ac


    def get_sh_lambda(self):
        """
            get __lambda_sh attribute function
        """
        return self.__lambda_sh


    def get_em_lambda(self):
        """
            get __electric_motor_system_failure_rate attribute function
        """
        return self.__electric_motor_system_failure_rate


    def get_cp_lambda(self):
        """
            get __lambda_cp attribute function
        """
        return self.__lambda_cp


    def get_bat_lambda(self):
        """
            get __lambda_bat attribute function
        """
        return self.__lambda_bat


# GET COMPONENTS' FUNCTIONS - START -

   # GET SPRING FUNCTIONS - START -
    def get_sp_c_dw(self):
        """
            get sp_c_dw attribute function
        """
        return self.sp_c_dw


    def get_sp_c_dc(self):
        """
            get sp_c_dc attribute function
        """
        return self.sp_c_dc


    def get_sp_c_n(self):
        """
            get sp_c_n attribute function
        """
        return self.sp_c_n


    def get_sp_c_l(self):
        """
            get sp_c_l attribute function
        """
        return self.sp_c_l


    def get_sp_c_k(self):
        """
            get sp_c_k attribute function
        """
        return self.sp_c_k


    def get_sp_c_cs(self):
        """
            get __sp_c_cs attribute function
        """
        return self.__sp_c_cs
   # GET SPRING FUNCTIONS - END -

   # GET GEAR FUNCTIONS - START -
    def get_gr_lambda_gr_b(self):
        """
            get __gr_lambda_gr_b attribute function
        """
        return self.__gr_lambda_gr_b


    def get_gr_c_gs(self):
        """
            get __gr_c_gs attribute function
        """
        return self.__gr_c_gs


    def get_gr_c_gp(self):
        """
            get __gr_c_gp attribute function
        """
        return self.__gr_c_gp


    def get_gr_c_ga(self):
        """
            get __gr_c_ga attribute function
        """
        return self.__gr_c_ga


    def get_gr_c_gl(self):
        """
            get __gr_c_gl attribute function
        """
        return self.__gr_c_gl


    def get_gr_c_gt(self):
        """
            get __gr_c_gt attribute function
        """
        return self.__gr_c_gt

   # GET GEAR FUNCTIONS - END -

   # GET ACTUATOR FUNCTIONS - START -

    def get_ac_c_h(self):
        """
            get ac_c_h attribute function
        """
        return self.ac_c_h


    def get_ac_lambda_ac_b(self):
        """
            get __ac_lambda_ac_b attribute function
        """
        return self.__ac_lambda_ac_b


    def get_ac_c_cp(self):
        """
            get __ac_c_cp attribute function
        """
        return self.__ac_c_cp
   # GET ACTUATOR FUNCTIONS - END -

   # GET BEARING FUNCTIONS - START -

    def get_be_lambda_be_b(self):
        """
            get __be_lambda_be_b attribute function
        """
        return self.__be_lambda_be_b


    def get_be_c_y(self):
        """
            get __be_c_y attribute function
        """
        return self.__be_c_y


    def get_be_c_r(self):
        """
            get __be_c_r attribute function
        """
        return self.__be_c_r


    def get_be_c_cw(self):
        """
            get __be_c_cw attribute function
        """
        return self.__be_c_cw


    def get_be_c_t(self):
        """
            get __be_c_t attribute function
        """
        return self.__be_c_t


    def get_be_c_v(self):
        """
            get __be_c_v attribute function
        """
        return self.__be_c_v

   # GET BEARING FUNCTIONS - END -

   # GET SHAFT FUNCTIONS - START -

    def get_sh_lambda_sh_b(self):
        """
            get __sh_lambda_sh_b attribute function
        """
        return self.__sh_lambda_sh_b


    def get_sh_c_t(self):
        """
            get __sh_c_t attribute function
        """
        return self.__sh_c_t


    def get_sh_c_sc_r(self):
        """
            get __sh_c_sc_r attribute function
        """
        return self.__sh_c_sc_r


    def get_sh_c_sc_g(self):
        """
            get sh_c_sc_g attribute function
        """
        return self.sh_c_sc_g


    def get_sh_c_sc(self):
        """
            get __sh_c_sc attribute function
        """
        return self.__sh_c_sc


    def get_sh_c_dy(self):
        """
            get __sh_c_dy attribute function
        """
        return self.__sh_c_dy

    def get_sh_c_f(self):
        """
            get __sh_c_f attribute function
        """
        return self.__sh_c_f

   # GET SHAFT FUNCTIONS - END -

   # GET ELECTRIC MOTORS FUNCTIONS - START -

    def get_em_lambda_wi_b(self):
        """
            get __em_lambda_wi_b attribute function
        """
        return self.__em_lambda_wi_b


    def get_em_c_t(self):
        """
            get __em_c_t attribute function
        """
        return self.__em_c_t


    def get_em_c_v(self):
        """
            get __em_c_v attribute function
        """
        return self.__em_c_v


    def get_em_v_u(self):
        """
            get __em_v_u attribute function
        """
        return self.__em_v_u


    def get_em_c_alt(self):
        """
            get __em_c_alt attribute function
        """
        return self.__em_c_alt


    def get_em_lambda_wi_func(self):
        """
            get __em_lambda_wi attribute function
        """
        return self.__em_lambda_wi


    def get_em_lambda_c(self):
        """
            get __em_lambda_c attribute function
        """
        return self.__em_lambda_c

   # GET ELECTRIC MOTORS FUNCTIONS - END -

# GET COMPONENTS' FUNCTIONS - END -


# COMPONENTS' FORMULAS - START -

  # SPRING FORMULAS - START -
    def lambda_sp_func(self):
        """
            calculate __lambda_sp attribute function
        """
        self.__lambda_sp = float(self.sp_lambda_sp_b * self.sp_c_g * self.sp_c_dw * self.sp_c_dc * self.sp_c_n * self.sp_c_y * self.sp_c_l * self.sp_c_k * self.__sp_c_cs * self.sp_c_r * self.sp_c_m) / float(pow(10, 6))


    def sp_c_dw_func(self, d_w):
        """
            calculate sp_c_dw attribute function
        """
        self.sp_c_dw = float(pow((d_w / 0.085), 3))


    def sp_c_dc_func(self, d_c):
        """
            calculate sp_c_dc attribute function
        """
        self.sp_c_dc = float(pow(0.58 / d_c, 6))


    def sp_c_n_func(self, n_a):
        """
            calculate sp_c_n attribute function
        """
        self.sp_c_n = float(pow(14 / n_a, 3))


    def sp_c_l_func(self, l_1, l_2):
        """
            calculate sp_c_l attribute function
        """
        self.sp_c_l = float(pow((l_1 - l_2) / 1.07, 3))


    def sp_c_k_func(self, d_c, d_w):
        """
            calculate sp_c_k attribute function
        """
        r_value = float(d_c / d_w)
        k_w = float(((4 * r_value - 1) / (4 * r_value + 1)) + (0.616 / r_value))
        self.sp_c_k = float(pow(k_w / 1.219, 3))


    def sp_c_cs_func(self, c_r):
        """
            calculate __sp_c_cs attribute function
        """
        if c_r <= 30:
            result = float(0.1)

        elif c_r <= 300:
            result = float(c_r / 300)

        else:
            result = float(pow(c_r / 300, 3))

        self.__sp_c_cs = result

  # SPRING FORMULAS - END -

  # BEARING FORMULAS - START -
    def lambda_be_func(self):
        """
            λBE = λBE,B * C_Y * C_R * C_V * C_CW * C_t * C_SF * C_C

            λBE = Failure rate of bearing, failures/million hours

            λBE,B = Base failure rate, failures/million hours

            C_Y = Multiplying factor applied load

            C_R = Life adjustment factor for reliability

            C_V = Multiplying factor for lubricant

            C_CW = Multiplying factor for water contaminant level

            C_t = Multiplying factor for operating temperature

            C_SF = Multiplying factor for operating service conditions

            C_C = Multiplying factor for lubrication contamination level

        """

        self.__lambda_be = float(self.__be_lambda_be_b * self.__be_c_y * self.__be_c_r * self.__be_c_v * self.__be_c_cw * self.__be_c_t * self.be_c_sf * self.be_c_c) / float(pow(10, 6))


    def be_lambda_be_b_func(self, y_value, l_s, l_a, n_value, h_value):
        """
            λBE,B = 1 / L_10_h

            L_10 = (10^6 / 60 * n)* (L_S / L_A)^y

            y:constant 3 for ball bearings, 3.3 for roller bearings,

            L_S dynamic load rating of bearing, ıbf,

            L_A equivalent radial load on bearing, ıbf, L_10 bearing life with reliability 90%,
            millions of revolutions,

            n: operation speed, revolutions/min.
        """

        if str(n_value) != "None":
            l_10 = self.bearing_l_10_func(y_value, l_s, l_a, n_value)

            result = float(1 / (l_10 * h_value))

        else:
            result = 1.0

        self.__be_lambda_be_b = result

    @classmethod
    def bearing_l_10_func(cls, y_value, l_s, l_a, n_value):
        """
            L_10 = (10^6 / 60 * n)* (L_S / L_A)^y
        """

        result = float((pow(10, 6) / (60 * n_value)) * pow((l_s / l_a), y_value))

        return result


    def be_c_y_func(self, y_value, l_a, l_s):
        """
            C_Y = (L_A / LS)^y

            y:constant 3 for ball bearings, 3.3 for roller bearings,

            L_S dynamic load rating of bearing, ıbf,

            L_A equivalent radial load on bearing, ıbf, L_10 bearing life with reliability 90%,
            millions of revolutions,
        """
        self.__be_c_y = float(pow((l_a / l_s), y_value))


    def be_c_r_func(self, r_value, select):
        """
            C_R = (0.223) / ln(100 / R)^(2/3)

            R = ?
        """

        if select:
            self.__be_c_r = float(float(0.223) / float(pow(math.log(float(100) / float(r_value)), (2/3))))

        else:
            self.__be_c_r = 1.0


    def be_c_v_func(self, v_o, v_l, select):
        """
            C_V = (V_O / V_L)^0.54

            V_O= Viscosity of specification fluid

            V_L= Viscosity of lubricant used
        """

        if select:
            self.__be_c_v = float(pow((v_o / v_l), (0.54)))

        else:
            self.__be_c_v = 1.0


    def be_c_cw_func(self, cw_value, select):
        """
            C_CW = 1.0 + 25.50CW - 16.25 CW^2

            CW = Percentage of water in the lubricant
        """
        if select:
            if cw_value > 0.8:
                result = 11.00

            else:
                result = float(1.0 + (25.50 * cw_value) - (16.25 * pow(cw_value, 2)))

        else:
            result = 1.0

        self.__be_c_cw = result


    def be_c_t_func(self, t_0):
        """
            C_T = (T_0 / 183)^3

            T_0 = Operating Temperature of the Bearing (℃)
        """

        if t_0 < 183:
            result = 1.0

        else:
            result = float(pow((t_0 / 183), 3))

        self.__be_c_t = result

  # BEARING FORMULAS - END -

  # GEAR FORMULAS - START -

    def lambda_gr_func(self):
        """
            λGR =  λGR,B * C_GS * C_GP * C_GA * C_GL * C_GT * C_GV

            λGR = Failure rate of gear under specific operation, failures/millionoperating hours

            λGR,B = Base failure rate of gear, failures/million operating hours

            C_GS = Multiplying factor considering speed deviation with respect todesign

            C_GP = Multiplying factor considering actual gear loading with respectto design

            C_GA = Multiplying factor considering misalignment

            C_GL = Multiplying factor considering lubrication deviation with respectto design

            C_GT = Multiplying factor considering the operating temperature

            C_GV = Multiplying factor considering the AGMA Service Factor
        """

        self.__lambda_gr = float(self.__gr_lambda_gr_b * self.__gr_c_gs * self.__gr_c_gp * self.__gr_c_ga * self.__gr_c_gl * self.__gr_c_gt * self.gr_c_gv) / float(pow(10, 6))


    def gr_lambda_gr_b_func(self, rpm_value, revolutions, select):
        """
            λGR,B = RPM * 60 * 1 / design life(revolutions)
        """
        if select:
            self.__gr_lambda_gr_b = float(rpm_value * 60 * (1 / revolutions))

        else:
            self.__gr_lambda_gr_b = 1.0


    def c_gs_func(self, v_o, v_d, select):
        """
            C_GS = k + (Vo / Vd)^0.7

            k = constant(1.0)

            Vo = Operating Speed, RPM

            Vd = Design Speed, RPM
        """
        if select:
            self.__gr_c_gs = float(1.0 + pow((v_o / v_d), 0.7))

        else:
            self.__gr_c_gs = 1.0


    def c_gp_func(self, l_o, l_d, select):
        """
            C_GP = ((Lo / Ld) / k)^4.69

            k = constant(0.5)

            Lo = Operating Load, lbs

            Ld = Design Load, lbs
        """
        if select:
            self.__gr_c_gp = float(0.5 + pow(((l_o / l_d) / 0.5), 4.69))

        else:
            self.__gr_c_gp = 1.0


    def c_ga_func(self, a_e, select):
        """
            C_GA = (AE / 0.006)^2.36

            AE = Misalignment angle in radians
        """
        if select:
            self.__gr_c_ga = float(pow((a_e / 0.006), 2.36))

        else:
            self.__gr_c_ga = 1.0



    def c_gl_func(self, v_o, v_l, select):
        """
            C_GL = k + (Vo / Vl)^0.54

            Vo = Viscosity of specification lubricant, lb-min/in2

            Vl = Viscosity of lubricant used, lb-min/in2

            k = constant(1.0)

            NOT: k değeri dökümanda yok!
        """
        if select:
            self.__gr_c_gl = float(1.0 + pow((v_o / v_l), 0.54))

        else:
            self.__gr_c_gl = 1.0


    def c_gt_func(self, t_at):
        """
            C_GT= (460 + T_AT) / 620

            T_AT= Operating temperature, ℉
        """

        if t_at <= 160:
            result = 1.0

        else:
            result = float((460 + t_at) / 620)

        self.__gr_c_gt = result

  # GEAR FORMULAS - END -

  # ACTUATOR FORMULAS - START -

    def lambda_ac_func(self):
        """
            calculate __lambda_ac attribute function
        """
        self.__lambda_ac = float(self.__ac_lambda_ac_b * self.__ac_c_cp * self.__ac_c_t) / float(pow(10, 6))


    def ac_n_o_func(self, w_a, d_1, d_2, mu_1, mu_2, e_1, e_2):
        """
            calculate ac_n_o attribute function
        """
        self.ac_n_o = float(self.ac_k_2 * pow((self.ac_gama * self.ac_f_y) / pow((float(w_a) * (pow((d_1 - d_2) / (d_1 * d_2), 2))) / pow(((1 - pow(mu_1, 2)) / e_1) - ((1 - pow(mu_2, 2)) / e_2), 2), 1/3), 9))


    def ac_lambda_ac_b_func(self):
        """
            calculate __ac_lambda_ac_b attribute function
        """
        self.__ac_lambda_ac_b = float(pow(10, 6) / self.ac_n_o)


    def ac_c_h_func(self, h_p, h_c, select):
        """
            calculate ac_c_h attribute function
        """
        if select:
            self.ac_c_h = float(h_p / h_c)

        else:
            self.ac_c_h = 1.0


    def ac_c_s_func(self, filter_size):
        """
            calculate ac_c_s attribute function
        """
        self.ac_c_s = float(filter_size / 10)


    def ac_c_cp_func(self, select):
        """
            calculate __ac_c_cp attribute function
        """
        if select:
            self.__ac_c_cp = float(self.ac_c_h * self.ac_c_s * self.ac_c_n)
        else:
            self.__ac_c_cp = 1.0


    def ac_c_t(self, teta, t_a, time_value):
        """
            calculate __ac_c_t attribute function
        """
        self.__ac_c_t = float(pow(math.e, (teta / self.ac_t_a) * (1 - t_a / time_value)))

  # ACTUATOR FORMULAS - END -

  # ELECTRIC MOTORS FORMULAS - START -

    def electric_motor_system_failure_rate_func(self):
        """
            calculate __electric_motor_system_failure_rate attribute function
        """
        electric_motor_failure_rate = self.electric_motor_failure_rate_func()
        self.__electric_motor_system_failure_rate = float(electric_motor_failure_rate * float((1.0 / pow(10, 6))))


    def electric_motor_failure_rate_func(self):
        """
            λM = (λM,B * CSF) + λWI + λBS + λST + λSH + λBE + λGR + λC
        """

        self.__em_lambda_m_b = self.lambda_m_b
        self.__em_c_sf = self.c_sf

        if self.__em_lambda_m_b == 1.0 and self.__em_c_sf == 1.0:
            temp_value = 0.0

        else:
            temp_value = self.__em_lambda_m_b * self.__em_c_sf

        result = float(temp_value) + float(self.__em_lambda_wi + self.__em_lambda_bs + self.__em_lambda_st + self.__lambda_sh + self.__lambda_be + self.__lambda_gr + self.__em_lambda_c)

        return result


    def elec_mot_lambda_wi_func(self):
        """
            λWI = λWI,B * C_T * C_V * C_alt

            λWI,B = Base failure rate of the electric motor windings, failures/millionhours

            C_T = Multiplying factor which considers the effects of ambient temperature
            on the base failure rate

            C_V = Multiplying factor which considers the effects of electrical source voltage
            variations

            C_alt =  Multiplying factor which considers the effects of operation at high altitudes
        """

        self.__em_lambda_wi = float(self.__em_lambda_wi_b * self.__em_c_t * self.__em_c_v * self.__em_c_alt)


    def elec_mot_lambda_wi_b_func(self, l_i):
        """
             λWI,B = (1.0 * 10^6) / L_I

             L_I = Expected winding life, hours
        """

        self.__em_lambda_wi_b = float((1.0 * pow(10, 6)) / l_i)


    def elec_mot_c_t_func(self, t_0):
        """
             C_T = 2^((T_0 - 40) / 10)

            T_0 = Ambient temperature surrounding motor with motor running atexpected
            full load conditions, oC
        """
        if str(t_0) != "None":
            self.__em_c_t = float(pow(2, ((float(t_0) - 40) / 10)))

        else:
            self.__em_c_t = 1.0


    def elec_mot_c_v_func(self, v_d, v_r, v_u, select):
        """
            Single Phase Motors ->  C_V = 2^(10 * (V_D / V_R)

            V_D = Difference between rated and actual voltage

            V_R = Rated voltage

            Three Phase Motors ->  C_V = 1 + (0.40 * V_U)^2.5
        """
        if select:
            if self.em_motor_phase == 1:
                result = pow(2, 10 * (float(v_d) / float(v_r)))

            elif self.em_motor_phase == 3:
                result = float(1 + pow(0.40 * float(v_u), 2.5))

            else:
                result = 1.0

        else:
            result = 1.0

        self.__em_c_v = result


    def elec_mot_v_u_func(self, greatest_voltage_difference, average_phase_voltage, select):
        """
            calculate __em_v_u attribute function
        """
        if select:
            self.__em_v_u = float(100 * (greatest_voltage_difference / average_phase_voltage))

        else:
            self.__em_v_u = 1.0


    def elec_mot_alt_func(self, altitude):
        """
            Calt = Multiplying factor which considers the effects of operation athigh altitudes

            C_alt= 1.00 + 8*10^-5(a-3300 ft)

            #Altitude = ft
        """

        if altitude <= 3300:
            result = 1.0

        else:
            result = float(1.00 + (8 * pow(10, -5)) * (altitude - 3300))

        self.__em_c_alt = result


    def elec_mot_lambda_c_func(self, value):
        """
            calculate __em_lambda_c attribute function
        """
        self.__em_lambda_c = value

  # ELECTRIC MOTORS FORMULAS - END -

  # SHAFTS MOTORS FORMULAS - START -

    def lambda_sh_func(self):
        """
            λSH = λSH,B * C_f * C_T * C_DY * C_SC

            λSH = Shaft failure rate, failures/million cycles

            λSH,B = Shaft base failure rate, failures/million cycles

            C_f = Shaft surface finish multiplying factor

            C_T = Material temperature multiplying factor

            C_DY = Shaft displacement multiplying factor

            C_SC = Stress concentration factor for shaft discontinuities
        """

        self.__lambda_sh = float(self.__sh_lambda_sh_b * self.__sh_c_f * self.__sh_c_t * self.__sh_c_dy * self.__sh_c_sc) / float(pow(10, 6))


    def sh_lambda_sh_b_func(self, n_value):
        """
            λSH,B = 1 / N

            N = Number of cycles to failure at application stress level, SED

            SED = Material endurance limit, lbs/in2
        """

        self.__sh_lambda_sh_b = float(1 / n_value)


    def sh_c_f_func(self, t_s, select):
        """
            calculate __sh_c_f attribute function
        """
        if select == 0:
            result = float(t_s)

        elif select == 1:
            result = self.sh_c_f_hot_rolled(float(t_s))

        elif select == 2:
            result = self.sh_c_f_macined(float(t_s))

        elif select == 3:
            result = self.sh_c_f_forged(float(t_s))

        else:
            result = 1.0

        self.__sh_c_f = result

    @classmethod
    def sh_c_f_hot_rolled(cls, t_s):
        """
            c_f = 0.94 - 0.0046 x Ts + 8.37 x 10^-6 x (Ts)^2

            Ts = Tensile strength of material, kpsi
        """

        result = float(0.94 - (0.0046 * t_s) + 8.37 * pow(10, -6) * pow(t_s, 2))

        return result

    @classmethod
    def sh_c_f_macined(cls, t_s):
        """
            c_f = 1.07 - 0.0051 x Ts + 2.21 x 10^-5 x (Ts )^2 - 3.57 x 10^-8 x (Ts)^3

            Ts = Tensile strength of material, kpsi
        """

        result = float(1.07 - 0.0051 * t_s + 2.21 * pow(10, -5) * pow(t_s, 2) - 3.57 * pow(10, -8) * pow(t_s, 3))

        return result

    @classmethod
    def sh_c_f_forged(cls, t_s):
        """
            c_f = 0.75 - 4.06 x 10^-3 x Ts + 7.58 x 10^-6 x (Ts)^2

            Ts = Tensile strength of material, kpsi
        """

        result = float(0.75 - 4.06 * pow(10, -3) * t_s + 7.58 * pow(10, -6) * pow(t_s, 2))

        return result


    def sh_c_t_func(self, t_at):
        """
            C_T= (460+T_AT) / 620

            #T_AT= Operating temperature (℉)
        """

        if t_at < 160:
            result = 1.0

        else:
            result = float((460 + t_at) / 620)

        self.__sh_c_t = result


    def sh_c_dy_func(self, e_value, f_value, b_value, sections_dict):
        """
            CDY= ((0.0043 * F) / Eb) * [(X^3 /I_X) + (L^3 / I_L) + (M^3 / I_M) + (N^3 / I_N)]

            E = Modulus of elasticity of shaft material, lbs/in2

            F = Fluid radial unbalance force or load weight, lb

            I = Shaft moment of inertia (πd4/64)  -> shaft_moment_of_inertia_func(d) fonksiyonu

            b = Specified shaft deflection, in

            e = E_b

            X, L, M, N = Length of shaft section
        """
        try:
            section_names = list(sections_dict.keys())
            constant = (0.0043 * f_value) / (e_value * b_value)
            result = 0

            for item in section_names:
                length = float(sections_dict[str(item)]['length'])
                inertia = float(sections_dict[str(item)]['I'])
                result += float(pow(length, 3) / inertia)

            self.__sh_c_dy = constant * result

        except Exception as err:
            print(err)

    @classmethod
    def shaft_moment_of_inertia_func(cls, d_value):
        """
            I = (π * d * 4) / 64
        """
        result = float((math.pi * d_value * 4) / 64)

        return result


    def sh_c_sc_func(self):
        """
            C_SC = C_SC,R + C_SC,G

            CSC,R = Stress concentration factor due to transition between shaft sections

            CSC,G = Stress concentration factor due to shaft grooves

            C_SC,R   -> sh_c_sc_r_func(r, bd, sd)
        """

        self.__sh_c_sc = float(self.__sh_c_sc_r + self.sh_c_sc_g)


    def sh_c_sc_r_func(self, r_value, b_d, s_d, select):
        """
            CSC,R= ((0.3)/(r / s_d))^0.2 * (b_d / s_d)^(1-(r/s_d))

            r = Radius of fillet, in

            b_d = Initial shaft diameter,

            s_d = Transitioned shaft diameter
        """
        if select:
            result = float(pow((0.3) / (r_value / s_d), (0.2)) * pow((b_d / s_d), ((1.0) - (r_value / s_d))))

        else:
            result = 1.0

        self.__sh_c_sc_r = result

  # SHAFTS MOTORS FORMULAS - END -

  # MECHANICAL COUPLINGS MOTORS FORMULAS - START -

    def lambda_cp_func(self):
        """
            calculate __lambda_cp attribute function
        """
        self.__lambda_cp = ((float(self.cp_lambda_cp_b) * float(self.cp_c_sf)) + float(self.__lambda_gr) + float(self.__lambda_se) + float(self.cp_lambda_h)) / float(pow(10, 6))

  # MECHANICAL COUPLINGS MOTORS FORMULAS - END -

  # BATTERY FORMULAS - START -

    def lambda_bat_func(self):
        """
            calculate __lambda_bat attribute function
        """
        self.__lambda_bat = float(self.bat_lambda_0 * pow(10, -9))

  # BATTERY FORMULAS - END -

# COMPONENTS' FORMULAS - END -
