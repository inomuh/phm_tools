#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""

    PHM Gui Electrical Equipment Class

"""

import math

class ElectricalEquipment:
    """
        (Failures / 10 ^ 6 * Hours)

        CC_Lambda = λCapasitor

        DD_Lambda = λDiode

        ID_Lambda = λInductor

        TS_Lambda = λTransistor

        FS_Lambda = λFuse
    """

    def __init__(self):
        # Private System Attribute
        self.__qrtz_lambda = 1.0
        self.__con_sock_lambda = 1.0
        self.__con_gen_lambda = 1.0
        self.__rel_lambda = 1.0
        self.__rd_lambda = 1.0
        self.__res_lambda = 1.0
        self.__cc_lambda = 1.0
        self.__dd_lambda = 1.0
        self.__id_lambda = 1.0
        self.__ts_lambda = 1.0
        self.__fs_lambda = 1.0

# COMPONENTS' ATTRIBUTES - START -

     # QUARTZ CRYSTALS ATTRIBUTES - START -
        # Public Relays Attributes
        self.qrtz_pi_q = 1.0
        self.qrtz_pi_e = 1.0

        # Private Relays Attributes
        self.qrtz_lambda_b = 1.0

     # QUARTZ CRYSTALS ATTRIBUTES - END -

     # CONNECTORS, SOCKETS ATTRIBUTES - START -
        # Public Relays Attributes
        self.con_sock_lambda_b = 1.0
        self.con_sock_pi_q = 1.0
        self.con_sock_pi_e = 1.0

        # Private Relays Attributes
        self.con_sock_pi_p = 1.0

     # CONNECTORS, SOCKETS ATTRIBUTES - END -

     # CONNECTORS, GENERAL ATTRIBUTES - START -
        # Public Relays Attributes
        self.con_gen_lambda_b = 1.0
        self.con_gen_pi_k = 1.0
        self.con_gen_pi_q = 1.0
        self.con_gen_pi_e = 1.0

        # Private Relays Attributes
        self.con_gen_pi_t = 1.0

     # CONNECTORS, GENERAL ATTRIBUTES - END -

     # RELAYS ATTRIBUTES - START -
        # Public Relays Attributes
        self.rel_lambda_b = 1.0
        self.rel_pi_c = 1.0
        self.rel_pi_q = 1.0
        self.rel_pi_e = 1.0
        self.rel_pi_f = 1.0

        # Private Relays Attributes
        self.rel_pi_l = 1.0
        self.rel_pi_cyc = 1.0

     # RELAYS ATTRIBUTES - END -

     # ROTATING DEVICES, MOTORS ATTRIBUTES - START -
        # Public Rotating Devices Attributes
        self.rd_determination_a = 1.0
        self.rd_determination_b = 1.0

        # Private Rotating Devices Attributes
        self.rd_lambda_1 = 1.0
        self.rd_lambda_2 = 1.0
        self.rd_alpha_b = 1.0
        self.rd_alpha_w = 1.0
        self.__rd_lambda_p_1 = 0.0
        self.__rd_lambda_p_2 = 0.0
     # ROTATING DEVICES, MOTORS ATTRIBUTES - END -

     # RESISTOR ATTRIBUTES - START -
        # Public Resistor Attributes
        self.res_lambda_b = 1.0
        self.res_pi_q = 1.0
        self.res_pi_e = 1.0

        # Private Resistor Attributes
        self.res_pi_t = 1.0
        self.res_pi_p = 1.0
        self.res_pi_s = 1.0
     # RESISTOR ATTRIBUTES - END -

     # CAPACITOR ATTRIBUTES - START -
        # Public Capacitor Attribute
        self.cc_lambda_b = 1.0
        self.cc_pi_q = 1.0
        self.cc_pi_e = 1.0

        # Private Capacitor Attribute
        self.__cc_pi_t = 1.0
        self.__cc_pi_c = 1.0
        self.__cc_pi_v = 1.0
        self.__cc_pi_sr = 1.0
     # CAPACITOR ATTRIBUTES - END -

     # DIODE ATTRIBUTES - START -
        # Public Diod Attribute
        self.dd_lambda_b = 1.0
        self.dd_pi_c = 1.0
        self.dd_pi_q = 1.0
        self.dd_pi_e = 1.0

        # Private Diod Attribute
        self.__dd_pi_t = 1.0
        self.__dd_pi_s = 1.0
     # DIODE ATTRIBUTES - END -

     # INDUCTOR ATTRIBUTES - START -
        # Public Inductor Attribute
        self.id_lambda_b = 1.0
        self.id_pi_q = 1.0
        self.id_pi_e = 1.0

        # Private Inductor Attribute
        self.__id_pi_t = 1.0
     # INDUCTOR ATTRIBUTES - END -

     # TRANSISTOR ATTRIBUTES - START -
        # Public Transistor Attribute
        self.ts_lambda_b = 1.0
        self.ts_pi_q = 1.0
        self.ts_pi_e = 1.0

        # Private Transistor Attribute
        self.__ts_pi_t = 1.0
     # TRANSISTOR ATTRIBUTES - END -

     # FUSE ATTRIBUTES - START -
        # Public Fuse Attribute
        self.fs_lambda_b = 1.0
        self.fs_pi_e = 1.0
     # FUSE ATTRIBUTES - END -

# COMPONENTS' ATTRIBUTES - END -

 # Get Main Functions
    def get_qrtz_lambda(self):
        """
            get __qrtz_lambda attribute function
        """
        return self.__qrtz_lambda


    def get_con_sock_lambda(self):
        """
            get __con_sock_lambda attribute function
        """
        return self.__con_sock_lambda


    def get_con_gen_lambda(self):
        """
            get __con_gen_lambda attribute function
        """
        return self.__con_gen_lambda


    def get_rel_lambda(self):
        """
            get __rel_lambda attribute function
        """
        return self.__rel_lambda


    def get_rd_lambda(self):
        """
            get __rd_lambda attribute function
        """
        return self.__rd_lambda


    def get_res_lambda(self):
        """
            get __res_lambda attribute function
        """
        return self.__res_lambda


    def get_cc_lambda(self):
        """
            get __cc_lambda attribute function
        """
        return self.__cc_lambda


    def get_dd_lambda(self):
        """
            get __dd_lambda attribute function
        """
        return self.__dd_lambda


    def get_id_lambda(self):
        """
            get __id_lambda attribute function
        """
        return self.__id_lambda


    def get_ts_lambda(self):
        """
            get __ts_lambda attribute function
        """
        return self.__ts_lambda


    def get_fs_lambda(self):
        """
            get __fs_lambda attribute function
        """
        return self.__fs_lambda

# GET COMPONENTS' FUNCTIONS - START -

 # GET QUARTZ CRYSTALS FUNCTIONS - START-

    def get_qrtz_lambda_b(self):
        """
            get qrtz_lambda_b attribute function
        """
        return self.qrtz_lambda_b

 # GET QUARTZ CRYSTALS FUNCTIONS - END-

 # GET CONNECTORS, SOCKETS FUNCTIONS - START-

    def get_con_sock_pi_p(self):
        """
            get con_sock_pi_p attribute function
        """
        return self.con_sock_pi_p

 # GET CONNECTORS, SOCKETS FUNCTIONS - END-

 # GET CONNECTORS, GENERAL FUNCTIONS - START-

    def get_con_gen_pi_t(self):
        """
            get con_gen_pi_t attribute function
        """
        return self.con_gen_pi_t

 # GET CONNECTORS, GENERAL FUNCTIONS - END-

 # GET RELAYS FUNCTIONS - START-

    def get_rel_lambda_b(self):
        """
            get rel_lambda_b attribute function
        """
        return self.rel_lambda_b


    def get_rel_pi_l(self):
        """
            get rel_pi_l attribute function
        """
        return self.rel_pi_l


    def get_rel_pi_cyc(self):
        """
            get rel_pi_cyc attribute function
        """
        return self.rel_pi_cyc

 # GET RELAYS FUNCTIONS - END -

 # GET ROTATING DEVICES, MOTORS FUNCTIONS - START-

    def get_rd_lambda_p_1(self):
        """
            get __rd_lambda_p_1 attribute function
        """
        return self.__rd_lambda_p_1


    def get_rd_lambda_p_2(self):
        """
            get __rd_lambda_p_2 attribute function
        """
        return self.__rd_lambda_p_2


    def get_rd_alpha_b(self):
        """
            get rd_alpha_b attribute function
        """
        return self.rd_alpha_b


    def get_rd_alpha_w(self):
        """
            get rd_alpha_w attribute function
        """
        return self.rd_alpha_w

 # GET ROTATING DEVICES, MOTORS FUNCTIONS - END -

 # GET RESISTOR FUNCTIONS - START-

    def get_res_pi_t(self):
        """
            get res_pi_t attribute function
        """
        return self.res_pi_t


    def get_res_pi_p(self):
        """
            get res_pi_p attribute function
        """
        return self.res_pi_p


    def get_res_pi_s(self):
        """
            get res_pi_s attribute function
        """
        return self.res_pi_s

 # GET RESISTOR FUNCTIONS - END -

 # GET CAPACITOR FUNCTIONS - START-

    def get_cc_pi_t(self):
        """
            get __cc_pi_t attribute function
        """
        return self.__cc_pi_t


    def get_cc_pi_c(self):
        """
            get __cc_pi_c attribute function
        """
        return self.__cc_pi_c


    def get_cc_pi_v(self):
        """
            get __cc_pi_v attribute function
        """
        return self.__cc_pi_v


    def get_cc_pi_sr(self):
        """
            get __cc_pi_sr attribute function
        """
        return self.__cc_pi_sr

 # GET CAPACITOR FUNCTIONS - END -

 # GET DIODE FUNCTIONS - START-

    def get_dd_pi_t(self):
        """
            get __dd_pi_t attribute function
        """
        return self.__dd_pi_t


    def get_dd_pi_s(self):
        """
            get __dd_pi_s attribute function
        """
        return self.__dd_pi_s

 # GET DIODE FUNCTIONS - END -


 # GET INDUCTOR FUNCTIONS - START-

    def get_id_pi_t(self):
        """
            get __id_pi_t attribute function
        """
        return self.__id_pi_t

 # GET INDUCTOR FUNCTIONS - END -


 # GET TRANSISTOR FUNCTIONS - START-

    def get_ts_pi_t(self):
        """
            get __ts_pi_t attribute function
        """
        return self.__ts_pi_t

 # GET TRANSISTOR FUNCTIONS - END -

# GET COMPONENTS' FUNCTIONS - END -


# COMPONENTS' FORMULAS - START -

 # QUARTZ CRYSTALS FORMULAS - START -
    def quartz_crystals_func(self):
        """
            λp = (λb * πQ * πE) /10^6
        """
        self.__qrtz_lambda = float(self.qrtz_lambda_b * self.qrtz_pi_q * self.qrtz_pi_e) / float(pow(10, 6))


    def qrtz_lambda_b_func(self, f_value):
        """
            calculate qrtz_lambda_b attribute function
        """
        self.qrtz_lambda_b = float(0.013 * pow(f_value, 0.23))

 # QUARTZ CRYSTALS FORMULAS - END -


 # CONNECTORS, SOCKETS FORMULAS - START -

    def connectors_sockets_func(self):
        """
            λp = λb * πP * πQ * πE /10 6
        """
        self.__con_sock_lambda = float(self.con_sock_lambda_b * self.con_sock_pi_p * self.con_sock_pi_q * self.con_sock_pi_e) / float(pow(10, 6))


    def con_sock_pi_p_func(self, n_value):
        """
            calculate con_sock_pi_p attribute function
        """
        self.con_sock_pi_p = math.exp(pow(((n_value - 1) / 10), 0.39))

 # CONNECTORS, SOCKETS FORMULAS - END -


 # CONNECTORS, GENERAL FORMULAS - START -

    def connectors_general_func(self):
        """
            λp = (λb * πT * πK * πQ * πE) / 10^6
        """
        self.__con_gen_lambda = float(self.con_gen_lambda_b * self.con_gen_pi_t * self.con_gen_pi_k * self.con_gen_pi_q * self.con_gen_pi_e) / float(pow(10, 6))


    def con_gen_pi_t_func(self, t0_value):
        """
            calculate con_gen_pi_t attribute function
        """
        self.con_gen_pi_t = float(math.exp((-0.14 / (8.617 * (pow(10, -5)))) * ((1 / (t0_value + 273)) - (1 / 298))))

 # CONNECTORS, GENERAL FORMULAS - END -

 # RELAYS FORMULAS - START -

    def relay_func(self):
        """
            λp = (λb * πL * πC * πCYC * πF * πQ * πE) / 10^6
        """
        self.__rel_lambda = float(self.rel_lambda_b * self.rel_pi_l * self.rel_pi_c * self.rel_pi_cyc * self.rel_pi_f * self.rel_pi_q * self.rel_pi_e) / float(pow(10, 6))


    def rel_lambda_b_func(self, temperature, rated_temp):
        """
            calculate rel_lambda_b attribute function
        """
        if rated_temp == 85:
            self.rel_lambda_b = float(0.0059 * float(math.exp((-0.19 / (8.617 * pow(10, -5))) * ((1 / (temperature + 273)) - (1 / 298)))))

        elif rated_temp == 125:
            self.rel_lambda_b = float(0.0059 * float(math.exp((-0.17 / (8.617 * pow(10, -5))) * ((1 / (temperature + 273)) - (1 / 298)))))


    def rel_pi_l_func(self, stress, select):
        """
            calculate rel_pi_l attribute function
        """
        if select == 1:
            self.rel_pi_l = float(pow((stress / 0.8), 2) + 1)

        elif select == 2:
            self.rel_pi_l = float(pow((stress / 0.4), 2) + 1)

        elif select == 3:
            self.rel_pi_l = float(pow((stress / 0.2), 2) + 1)


    def rel_pi_cyc_func(self, cycles, select):
        """
            calculate rel_pi_cyc attribute function
        """
        if select == 0:
            if cycles >= float(1.0):
                result = float(cycles / 10)

            elif cycles < float(1.0):
                result = float(0.1)

        else:
            if cycles <= 10:
                result = float(1.0)

            elif cycles <= 1000:
                result = float(cycles / 10)

            elif cycles > 1000:
                result = float(pow(cycles/100, 2))

        self.rel_pi_cyc = result

 # RELAYS FORMULAS - END -

 # ROTATING DEVICES, MOTORS FORMULAS - START -

    def rotating_device_func(self):
        """
            λp = ((λ1 / (A *αB)) + (λ2 / (B *αW))) *  1 / 10^6
        """
        self.__rd_lambda = self.__rd_lambda_p_1 + self.__rd_lambda_p_2


    def rotating_device_func_1(self):
        """
            λp1 = (λ1 / (A *αB)) * 1 / 10^6
        """
        self.__rd_lambda_p_1 = float((self.rd_lambda_1 / (self.rd_determination_a * self.rd_alpha_b)) / float(pow(10, 6)))


    def rotating_device_func_2(self):
        """
            λp2 = (λ2 / (B *αW)) * 1 / 10^6
        """
        self.__rd_lambda_p_2 = float((self.rd_lambda_2 / (self.rd_determination_b * self.rd_alpha_w)) / float(pow(10, 6)))


    def rd_alpha_b_func(self, temperature):
        """
            calculate rd_alpha_b attribute function
        """
        self.rd_alpha_b = float(pow(float(pow(10, float(2.534 - float(2357 / (temperature + 273)))) + (1 / float(pow(10, float(20 - (4500 / float(temperature + 273)))) + 300))), -1))


    def rd_alpha_w_func(self, temperature):
        """
            calculate rd_alpha_w attribute function
        """
        self.rd_alpha_w = float(pow(10, float(float(2357 / (temperature + 273)) - 1.83)))

 # ROTATING DEVICES, MOTORS FORMULAS - END -

 # RESISTOR FORMULAS - START -
    def resistors_func(self):
        """
            λp = λb * πT * πP * πS * πQ * πE *  1 / 10^6
        """
        self.__res_lambda = float((self.res_lambda_b * self.res_pi_t * self.res_pi_p * self.res_pi_s * self.res_pi_q * self.res_pi_e) / float(pow(10, 6)))


    def res_pi_t_func(self, temperature, select):
        """
            calculate res_pi_t attribute function
        """
        if select == 1:
            ea_value = 0.2
            result = self.calculate_res_pi_t_func(temperature, ea_value)

        elif select == 2:
            ea_value = 0.08
            result = self.calculate_res_pi_t_func(temperature, ea_value)

        else:
            result = 1.0

        self.res_pi_t = result

    @classmethod
    def calculate_res_pi_t_func(cls, temperature, ea_value):
        """
            calculate res_pi_t value function
        """
        result = float(math.exp(((-1 * ea_value) / (8.617 * pow(10, -5))) * ((1 / (temperature + 273)) - (1 / 298))))

        return result


    def res_pi_p_func(self, power_dissipiation):
        """
            calculate res_pi_p attribute function
        """
        self.res_pi_p = float(pow(power_dissipiation, 0.39))


    def res_pi_s_func(self, stress, select):
        """
            calculate res_pi_s attribute function
        """
        if select == 1:
            result = 0.71 * float(math.exp(1.1 * stress))

        elif select == 2:
            result = 0.54 * float(math.exp(2.04 * stress))

        else:
            result = 1.0

        self.res_pi_s = result

 # RESISTOR FORMULAS - END -

 # CAPACITOR FORMULAS - START -

    def capacitors_func(self):
        """
            λp = λb * πT * πC * πV * πSR * πQ * πE *  1 / 10^6
        """
        self.__cc_lambda = float(self.cc_lambda_b * self.__cc_pi_t * self.__cc_pi_c * self.__cc_pi_v * self.__cc_pi_sr * self.cc_pi_q * self.cc_pi_e) / float(pow(10, 6))


    def cc_pi_t_func(self, temperature, select):
        """
            calculate __cc_pi_t attribute function
        """
        if select == 0:
            result = float(temperature)

        elif select == 1:
            ea_value = 0.15
            result = self.calculate_cc_pi_t_func(temperature, ea_value)

        elif select == 2:
            ea_value = 0.35
            result = self.calculate_cc_pi_t_func(temperature, ea_value)

        else:
            result = 1.0

        self.__cc_pi_t = result

    @classmethod
    def calculate_cc_pi_t_func(cls, temperature, ea_value):
        """
            calculate __cc_pi_t value function
        """
        result = float(math.exp(((-1 * ea_value) / (8.617 * pow(10, -5))) * ((1 / (temperature + 273)) - (1 / 298))))

        return result


    def cc_pi_c_func(self, capacitance, select):
        """
            calculate __cc_pi_c attribute function
        """
        if select == 0:
            result = float(capacitance)

        elif select == 1:
            result = float(pow(capacitance, 0.09))

        elif select == 2:
            result = float(pow(capacitance, 0.23))

        else:
            result = 1.0

        self.__cc_pi_c = result


    def cc_pi_v_func(self, voltage_stress, select):
        """
            calculate __cc_pi_v attribute function
        """
        if select == 0:
            result = float(voltage_stress)

        elif select == 1:
            result = float(pow((voltage_stress / 0.6), 5) + 1)

        elif select == 2:
            result = float(pow((voltage_stress / 0.6), 10) + 1)

        elif select == 3:
            result = float(pow((voltage_stress / 0.6), 3) + 1)

        elif select == 4:
            result = float(pow((voltage_stress / 0.6), 17) + 1)

        elif select == 5:
            result = float(pow((voltage_stress / 0.6), 3) + 1)

        else:
            result = 1.0

        self.__cc_pi_v = result


    def cc_pi_sr_func(self, select):
        """
            calculate __cc_pi_sr attribute function
        """
        if select == "None":
            result = 1.0

        elif select == "cr_g_0_8":
            result = 0.66

        elif select == "cr_g_0_6_to_0_8":
            result = 1.0

        elif select == "cr_g_0_4_to_0_6":
            result = 1.3

        elif select == "cr_g_0_2_to_0_4":
            result = 2.0

        elif select == "cr_g_0_1_to_0_2":
            result = 2.7

        elif select == "cr_0_to_0_1":
            result = 3.3

        else:
            result = float(select)

        self.__cc_pi_sr = result

 # CAPACITOR FORMULAS - END -


 # DIODE FORMULAS - START -
    def diodes_func(self):
        """
            λp = λb * πT * πS * πC * πQ * πE *  1 / 10^6
        """
        self.__dd_lambda = float(self.dd_lambda_b * self.__dd_pi_t * self.__dd_pi_s * self.dd_pi_c * self.dd_pi_q * self.dd_pi_e) / float(pow(10, 6))


    def dd_pi_t_func(self, temperature, select):
        """
            calculate __dd_pi_t attribute function
        """
        if select == 0:
            result = float(temperature)

        elif select == 1:
            constant = 1925
            result = self.calculate_dd_pi_t_func(temperature, constant)

        elif select == 2:
            constant = 3091
            result = self.calculate_dd_pi_t_func(temperature, constant)

        else:
            result = 1.0

        self.__dd_pi_t = result

    @classmethod
    def calculate_dd_pi_t_func(cls, temperature, constant):
        """
            calculate __dd_pi_t value function
        """
        result = float(math.exp((-1 * constant) * ((1 / (temperature + 273)) - (1 / 298))))

        return result


    def dd_pi_s_func(self, voltage_stress, select):
        """
            calculate __dd_pi_s attribute function
        """
        if select == 1:
            if voltage_stress < 0.3:
                result = 0.054

            elif voltage_stress <= 1.0:
                result = float(pow(voltage_stress, 2.43))

        elif select == 2:
            result = float(voltage_stress)

        else:
            result = 1.0

        self.__dd_pi_s = result

 # DIODE FORMULAS - END -

 # INDUCTOR FORMULAS - START -

    def inductors_func(self):
        """
            λp = λb * πT * πQ * πE *  1 / 10^6
        """
        self.__id_lambda = float(self.id_lambda_b * self.__id_pi_t * self.id_pi_q * self.id_pi_e) / float(pow(10, 6))


    def id_pi_t_func(self, hot_spot_temperature, select):
        """
            calculate __id_pi_t attribute function
        """
        if select == 0:
            result = hot_spot_temperature

        elif select == 1:
            result = float(math.exp(((-1 * 0.11) / (8.617 * pow(10, -5))) * ((1 / (hot_spot_temperature + 273)) - (1 / 298))))

        else:
            result = 1.0

        self.__id_pi_t = result

 # INDUCTOR FORMULAS - END -

 # TRANSISTOR FORMULAS - START -

    def transistors_func(self):
        """
            λp = λb * πT * πQ * πE *  1 / 10^6
        """
        self.__ts_lambda = float(self.ts_lambda_b * self.__ts_pi_t * self.ts_pi_q * self.ts_pi_e) / float(pow(10, 6))

    def ts_pi_t_func(self, temperature, select):
        """
            calculate __ts_pi_t attribute function
        """
        if select == 0:
            result = temperature

        elif select == 1:
            result = float(math.exp((-1 * 1925) * ((1 / (temperature + 273)) - (1 / 298))))

        else:
            result = 1.0

        self.__ts_pi_t = result

 # TRANSISTOR FORMULAS - END -

 # FUSE FORMULAS - START -
    def fuses_func(self):
        """
            λp = λb * πE *  1 / 10^6
        """
        self.__fs_lambda = float(self.fs_lambda_b * self.fs_pi_e) / float(pow(10, 6))
 # FUSE FORMULAS - END -
# COMPONENTS' FORMULAS - END -
