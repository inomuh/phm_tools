#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
    PHM Electrical Equipments Gui
"""

from PyQt5 import QtCore, QtWidgets
from phm_hazard_rate_calculation.class_electrical_equipment import ElectricalEquipment

class ElectricalEquipmentsWindow(object):
    """
        Electrical Equipment Window Class
    """
    def __init__(self, ui_class, data_path, equipment_type):
        self.ui_class = ui_class
        self.data_path = data_path
        self.equipment_type = equipment_type

        self.e_eq = ElectricalEquipment()

        self.title_name = "Electrical Equipments"

        if self.equipment_type:
            self.title_name = str("Sub Module " + self.title_name)

        else:
            self.title_name = str("Component " + self.title_name)

        self.electrical_equipments_dict = self.ui_class.electrical_equipments_dict
        self.capacitor_style_dict = self.ui_class.capacitor_style_dict
        self.diode_type_dict = self.ui_class.diode_type_dict
        self.resistor_type_dict = self.ui_class.resistor_type_dict

# ---------------------------------------------------------------------
    def setupUi(self, MainWindow):
        """
            Setup Ui Function
        """
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1100, 800)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(MainWindow.sizePolicy().hasHeightForWidth())
        MainWindow.setSizePolicy(sizePolicy)
        MainWindow.setMinimumSize(QtCore.QSize(1100, 800))
        MainWindow.setMaximumSize(QtCore.QSize(1100, 800))
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.tabWidget_electrical_equipments = QtWidgets.QTabWidget(self.centralwidget)
        self.tabWidget_electrical_equipments.setGeometry(QtCore.QRect(0, 0, 1100, 800))
        self.tabWidget_electrical_equipments.setObjectName("tabWidget_electrical_equipments")
        self.tab_capacitor = QtWidgets.QWidget()
        self.tab_capacitor.setObjectName("tab_capacitor")
        self.groupBox_cc_lambda_p = QtWidgets.QGroupBox(self.tab_capacitor)
        self.groupBox_cc_lambda_p.setGeometry(QtCore.QRect(450, 640, 200, 70))
        self.groupBox_cc_lambda_p.setObjectName("groupBox_cc_lambda_p")
        self.lineEdit_cc_lamda_p = QtWidgets.QLineEdit(self.groupBox_cc_lambda_p)
        self.lineEdit_cc_lamda_p.setGeometry(QtCore.QRect(0, 30, 200, 27))
        self.lineEdit_cc_lamda_p.setReadOnly(True)
        self.lineEdit_cc_lamda_p.setObjectName("lineEdit_cc_lamda_p")
        self.groupBox = QtWidgets.QGroupBox(self.tab_capacitor)
        self.groupBox.setGeometry(QtCore.QRect(90, 10, 350, 311))
        self.groupBox.setTitle("")
        self.groupBox.setObjectName("groupBox")
        self.label_cc_quality = QtWidgets.QLabel(self.groupBox)
        self.label_cc_quality.setGeometry(QtCore.QRect(60, 180, 71, 21))
        self.label_cc_quality.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_cc_quality.setWordWrap(False)
        self.label_cc_quality.setObjectName("label_cc_quality")
        self.comboBox_cc_capacitance = QtWidgets.QComboBox(self.groupBox)
        self.comboBox_cc_capacitance.setGeometry(QtCore.QRect(140, 90, 191, 27))
        self.comboBox_cc_capacitance.setCurrentText("")
        self.comboBox_cc_capacitance.setObjectName("comboBox_cc_capacitance")
        self.label_cc_capacitance_value = QtWidgets.QLabel(self.groupBox)
        self.label_cc_capacitance_value.setGeometry(QtCore.QRect(0, 90, 131, 21))
        self.label_cc_capacitance_value.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_cc_capacitance_value.setObjectName("label_cc_capacitance_value")
        self.comboBox_cc_quality = QtWidgets.QComboBox(self.groupBox)
        self.comboBox_cc_quality.setGeometry(QtCore.QRect(140, 180, 191, 27))
        self.comboBox_cc_quality.setCurrentText("")
        self.comboBox_cc_quality.setObjectName("comboBox_cc_quality")
        self.label_cc_t = QtWidgets.QLabel(self.groupBox)
        self.label_cc_t.setGeometry(QtCore.QRect(0, 50, 131, 41))
        self.label_cc_t.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_cc_t.setWordWrap(True)
        self.label_cc_t.setObjectName("label_cc_t")
        self.label_cc_circuit_resistance = QtWidgets.QLabel(self.groupBox)
        self.label_cc_circuit_resistance.setGeometry(QtCore.QRect(-10, 150, 141, 21))
        self.label_cc_circuit_resistance.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_cc_circuit_resistance.setObjectName("label_cc_circuit_resistance")
        self.comboBox_cc_environment = QtWidgets.QComboBox(self.groupBox)
        self.comboBox_cc_environment.setGeometry(QtCore.QRect(140, 210, 191, 27))
        self.comboBox_cc_environment.setCurrentText("")
        self.comboBox_cc_environment.setObjectName("comboBox_cc_environment")
        self.comboBox_cc_circuit_resistance = QtWidgets.QComboBox(self.groupBox)
        self.comboBox_cc_circuit_resistance.setGeometry(QtCore.QRect(140, 150, 191, 27))
        self.comboBox_cc_circuit_resistance.setCurrentText("")
        self.comboBox_cc_circuit_resistance.setObjectName("comboBox_cc_circuit_resistance")
        self.comboBox_cc_style = QtWidgets.QComboBox(self.groupBox)
        self.comboBox_cc_style.setGeometry(QtCore.QRect(140, 30, 191, 27))
        self.comboBox_cc_style.setCurrentText("")
        self.comboBox_cc_style.setObjectName("comboBox_cc_style")
        self.label_cc_environment = QtWidgets.QLabel(self.groupBox)
        self.label_cc_environment.setGeometry(QtCore.QRect(20, 210, 111, 21))
        self.label_cc_environment.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_cc_environment.setWordWrap(True)
        self.label_cc_environment.setObjectName("label_cc_environment")
        self.comboBox_cc_voltage_stress = QtWidgets.QComboBox(self.groupBox)
        self.comboBox_cc_voltage_stress.setGeometry(QtCore.QRect(140, 120, 191, 27))
        self.comboBox_cc_voltage_stress.setCurrentText("")
        self.comboBox_cc_voltage_stress.setObjectName("comboBox_cc_voltage_stress")
        self.label_cc_voltage_stress = QtWidgets.QLabel(self.groupBox)
        self.label_cc_voltage_stress.setGeometry(QtCore.QRect(30, 120, 101, 21))
        self.label_cc_voltage_stress.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_cc_voltage_stress.setObjectName("label_cc_voltage_stress")
        self.label_cc_capacitor_style = QtWidgets.QLabel(self.groupBox)
        self.label_cc_capacitor_style.setGeometry(QtCore.QRect(10, 30, 121, 21))
        self.label_cc_capacitor_style.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_cc_capacitor_style.setObjectName("label_cc_capacitor_style")
        self.comboBox_cc_capacitor_temperature = QtWidgets.QComboBox(self.groupBox)
        self.comboBox_cc_capacitor_temperature.setGeometry(QtCore.QRect(140, 60, 191, 27))
        self.comboBox_cc_capacitor_temperature.setCurrentText("")
        self.comboBox_cc_capacitor_temperature.setObjectName("comboBox_cc_capacitor_temperature")
        self.groupBox_2 = QtWidgets.QGroupBox(self.tab_capacitor)
        self.groupBox_2.setGeometry(QtCore.QRect(490, 10, 350, 301))
        self.groupBox_2.setTitle("")
        self.groupBox_2.setObjectName("groupBox_2")
        self.lineEdit_cc_pi_t = QtWidgets.QLineEdit(self.groupBox_2)
        self.lineEdit_cc_pi_t.setGeometry(QtCore.QRect(230, 60, 113, 27))
        self.lineEdit_cc_pi_t.setText("")
        self.lineEdit_cc_pi_t.setReadOnly(True)
        self.lineEdit_cc_pi_t.setObjectName("lineEdit_cc_pi_t")
        self.label_cc_pi_sr = QtWidgets.QLabel(self.groupBox_2)
        self.label_cc_pi_sr.setGeometry(QtCore.QRect(10, 150, 211, 31))
        self.label_cc_pi_sr.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_cc_pi_sr.setObjectName("label_cc_pi_sr")
        self.lineEdit_cc_pi_v = QtWidgets.QLineEdit(self.groupBox_2)
        self.lineEdit_cc_pi_v.setGeometry(QtCore.QRect(230, 120, 113, 27))
        self.lineEdit_cc_pi_v.setText("")
        self.lineEdit_cc_pi_v.setReadOnly(True)
        self.lineEdit_cc_pi_v.setObjectName("lineEdit_cc_pi_v")
        self.lineEdit_cc_pi_sr = QtWidgets.QLineEdit(self.groupBox_2)
        self.lineEdit_cc_pi_sr.setGeometry(QtCore.QRect(230, 150, 113, 27))
        self.lineEdit_cc_pi_sr.setText("")
        self.lineEdit_cc_pi_sr.setReadOnly(True)
        self.lineEdit_cc_pi_sr.setObjectName("lineEdit_cc_pi_sr")
        self.label_cc_pi_c = QtWidgets.QLabel(self.groupBox_2)
        self.label_cc_pi_c.setGeometry(QtCore.QRect(50, 90, 171, 31))
        self.label_cc_pi_c.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_cc_pi_c.setObjectName("label_cc_pi_c")
        self.label_cc_pi_e = QtWidgets.QLabel(self.groupBox_2)
        self.label_cc_pi_e.setGeometry(QtCore.QRect(0, 210, 221, 31))
        self.label_cc_pi_e.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_cc_pi_e.setObjectName("label_cc_pi_e")
        self.label_cc_pi_t = QtWidgets.QLabel(self.groupBox_2)
        self.label_cc_pi_t.setGeometry(QtCore.QRect(50, 60, 171, 31))
        self.label_cc_pi_t.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_cc_pi_t.setObjectName("label_cc_pi_t")
        self.lineEdit_cc_pi_e = QtWidgets.QLineEdit(self.groupBox_2)
        self.lineEdit_cc_pi_e.setGeometry(QtCore.QRect(230, 210, 113, 27))
        self.lineEdit_cc_pi_e.setText("")
        self.lineEdit_cc_pi_e.setReadOnly(True)
        self.lineEdit_cc_pi_e.setObjectName("lineEdit_cc_pi_e")
        self.lineEdit_cc_pi_c = QtWidgets.QLineEdit(self.groupBox_2)
        self.lineEdit_cc_pi_c.setGeometry(QtCore.QRect(230, 90, 113, 27))
        self.lineEdit_cc_pi_c.setText("")
        self.lineEdit_cc_pi_c.setReadOnly(True)
        self.lineEdit_cc_pi_c.setObjectName("lineEdit_cc_pi_c")
        self.label_cc_lambda_b = QtWidgets.QLabel(self.groupBox_2)
        self.label_cc_lambda_b.setGeometry(QtCore.QRect(70, 30, 151, 31))
        self.label_cc_lambda_b.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_cc_lambda_b.setObjectName("label_cc_lambda_b")
        self.lineEdit_cc_pi_q = QtWidgets.QLineEdit(self.groupBox_2)
        self.lineEdit_cc_pi_q.setGeometry(QtCore.QRect(230, 180, 113, 27))
        self.lineEdit_cc_pi_q.setText("")
        self.lineEdit_cc_pi_q.setReadOnly(True)
        self.lineEdit_cc_pi_q.setObjectName("lineEdit_cc_pi_q")
        self.lineEdit_cc_lambda_b = QtWidgets.QLineEdit(self.groupBox_2)
        self.lineEdit_cc_lambda_b.setGeometry(QtCore.QRect(230, 30, 113, 27))
        self.lineEdit_cc_lambda_b.setReadOnly(True)
        self.lineEdit_cc_lambda_b.setObjectName("lineEdit_cc_lambda_b")
        self.label_cc_pi_q = QtWidgets.QLabel(self.groupBox_2)
        self.label_cc_pi_q.setGeometry(QtCore.QRect(0, 180, 221, 31))
        self.label_cc_pi_q.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_cc_pi_q.setObjectName("label_cc_pi_q")
        self.label_cc_pi_v = QtWidgets.QLabel(self.groupBox_2)
        self.label_cc_pi_v.setGeometry(QtCore.QRect(40, 120, 181, 31))
        self.label_cc_pi_v.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_cc_pi_v.setObjectName("label_cc_pi_v")
        self.addButton_c_ee_capacitor = QtWidgets.QPushButton(self.tab_capacitor)
        self.addButton_c_ee_capacitor.setGeometry(QtCore.QRect(925, 650, 125, 50))
        self.addButton_c_ee_capacitor.setObjectName("addButton_c_ee_capacitor")
        self.lineEdit_c_cc_warning_box = QtWidgets.QLineEdit(self.tab_capacitor)
        self.lineEdit_c_cc_warning_box.setGeometry(QtCore.QRect(10, 640, 330, 70))
        self.lineEdit_c_cc_warning_box.setReadOnly(True)
        self.lineEdit_c_cc_warning_box.setObjectName("lineEdit_c_cc_warning_box")
        self.tabWidget_electrical_equipments.addTab(self.tab_capacitor, "")
        self.tab_diode = QtWidgets.QWidget()
        self.tab_diode.setObjectName("tab_diode")
        self.groupBox_dd_lambda_p = QtWidgets.QGroupBox(self.tab_diode)
        self.groupBox_dd_lambda_p.setGeometry(QtCore.QRect(450, 640, 200, 70))
        self.groupBox_dd_lambda_p.setObjectName("groupBox_dd_lambda_p")
        self.lineEdit_dd_lamda_p = QtWidgets.QLineEdit(self.groupBox_dd_lambda_p)
        self.lineEdit_dd_lamda_p.setGeometry(QtCore.QRect(0, 30, 200, 27))
        self.lineEdit_dd_lamda_p.setReadOnly(True)
        self.lineEdit_dd_lamda_p.setObjectName("lineEdit_dd_lamda_p")
        self.groupBox_3 = QtWidgets.QGroupBox(self.tab_diode)
        self.groupBox_3.setGeometry(QtCore.QRect(40, 10, 391, 211))
        self.groupBox_3.setTitle("")
        self.groupBox_3.setObjectName("groupBox_3")
        self.comboBox_dd_quality = QtWidgets.QComboBox(self.groupBox_3)
        self.comboBox_dd_quality.setGeometry(QtCore.QRect(190, 150, 191, 27))
        self.comboBox_dd_quality.setCurrentText("")
        self.comboBox_dd_quality.setObjectName("comboBox_dd_quality")
        self.comboBox_dd_contact_construction = QtWidgets.QComboBox(self.groupBox_3)
        self.comboBox_dd_contact_construction.setGeometry(QtCore.QRect(190, 120, 191, 27))
        self.comboBox_dd_contact_construction.setCurrentText("")
        self.comboBox_dd_contact_construction.setObjectName("comboBox_dd_contact_construction")
        self.label_dd_voltage_stress = QtWidgets.QLabel(self.groupBox_3)
        self.label_dd_voltage_stress.setGeometry(QtCore.QRect(10, 90, 171, 21))
        self.label_dd_voltage_stress.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_dd_voltage_stress.setObjectName("label_dd_voltage_stress")
        self.comboBox_dd_environment = QtWidgets.QComboBox(self.groupBox_3)
        self.comboBox_dd_environment.setGeometry(QtCore.QRect(190, 180, 191, 27))
        self.comboBox_dd_environment.setCurrentText("")
        self.comboBox_dd_environment.setObjectName("comboBox_dd_environment")
        self.label_dd_diode_type = QtWidgets.QLabel(self.groupBox_3)
        self.label_dd_diode_type.setGeometry(QtCore.QRect(10, 30, 171, 21))
        self.label_dd_diode_type.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_dd_diode_type.setObjectName("label_dd_diode_type")
        self.comboBox_dd_type = QtWidgets.QComboBox(self.groupBox_3)
        self.comboBox_dd_type.setGeometry(QtCore.QRect(190, 30, 191, 27))
        self.comboBox_dd_type.setCurrentText("")
        self.comboBox_dd_type.setObjectName("comboBox_dd_type")
        self.label_dd_contact_construction = QtWidgets.QLabel(self.groupBox_3)
        self.label_dd_contact_construction.setGeometry(QtCore.QRect(10, 120, 171, 21))
        self.label_dd_contact_construction.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_dd_contact_construction.setObjectName("label_dd_contact_construction")
        self.label_dd_quality = QtWidgets.QLabel(self.groupBox_3)
        self.label_dd_quality.setGeometry(QtCore.QRect(10, 150, 171, 21))
        self.label_dd_quality.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_dd_quality.setObjectName("label_dd_quality")
        self.comboBox_dd_junction_temperature = QtWidgets.QComboBox(self.groupBox_3)
        self.comboBox_dd_junction_temperature.setGeometry(QtCore.QRect(190, 60, 191, 27))
        self.comboBox_dd_junction_temperature.setCurrentText("")
        self.comboBox_dd_junction_temperature.setObjectName("comboBox_dd_junction_temperature")
        self.label_dd_junction_temperature = QtWidgets.QLabel(self.groupBox_3)
        self.label_dd_junction_temperature.setGeometry(QtCore.QRect(10, 60, 171, 21))
        self.label_dd_junction_temperature.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_dd_junction_temperature.setObjectName("label_dd_junction_temperature")
        self.comboBox_dd_voltage_stress = QtWidgets.QComboBox(self.groupBox_3)
        self.comboBox_dd_voltage_stress.setGeometry(QtCore.QRect(190, 90, 191, 27))
        self.comboBox_dd_voltage_stress.setCurrentText("")
        self.comboBox_dd_voltage_stress.setObjectName("comboBox_dd_voltage_stress")
        self.label_dd_environment = QtWidgets.QLabel(self.groupBox_3)
        self.label_dd_environment.setGeometry(QtCore.QRect(10, 180, 171, 21))
        self.label_dd_environment.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_dd_environment.setObjectName("label_dd_environment")
        self.groupBox_4 = QtWidgets.QGroupBox(self.tab_diode)
        self.groupBox_4.setGeometry(QtCore.QRect(470, 30, 371, 201))
        self.groupBox_4.setTitle("")
        self.groupBox_4.setObjectName("groupBox_4")
        self.lineEdit_dd_pi_t = QtWidgets.QLineEdit(self.groupBox_4)
        self.lineEdit_dd_pi_t.setGeometry(QtCore.QRect(250, 40, 113, 27))
        self.lineEdit_dd_pi_t.setText("")
        self.lineEdit_dd_pi_t.setReadOnly(True)
        self.lineEdit_dd_pi_t.setObjectName("lineEdit_dd_pi_t")
        self.label_dd_pi_s = QtWidgets.QLabel(self.groupBox_4)
        self.label_dd_pi_s.setGeometry(QtCore.QRect(50, 70, 191, 31))
        self.label_dd_pi_s.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_dd_pi_s.setObjectName("label_dd_pi_s")
        self.label_dd_pi_q = QtWidgets.QLabel(self.groupBox_4)
        self.label_dd_pi_q.setGeometry(QtCore.QRect(0, 130, 241, 31))
        self.label_dd_pi_q.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_dd_pi_q.setObjectName("label_dd_pi_q")
        self.lineEdit_dd_pi_c = QtWidgets.QLineEdit(self.groupBox_4)
        self.lineEdit_dd_pi_c.setGeometry(QtCore.QRect(250, 100, 113, 27))
        self.lineEdit_dd_pi_c.setText("")
        self.lineEdit_dd_pi_c.setReadOnly(True)
        self.lineEdit_dd_pi_c.setObjectName("lineEdit_dd_pi_c")
        self.label_dd_pi_e = QtWidgets.QLabel(self.groupBox_4)
        self.label_dd_pi_e.setGeometry(QtCore.QRect(0, 160, 241, 31))
        self.label_dd_pi_e.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_dd_pi_e.setObjectName("label_dd_pi_e")
        self.lineEdit_dd_lambda_b = QtWidgets.QLineEdit(self.groupBox_4)
        self.lineEdit_dd_lambda_b.setGeometry(QtCore.QRect(250, 10, 113, 27))
        self.lineEdit_dd_lambda_b.setReadOnly(True)
        self.lineEdit_dd_lambda_b.setObjectName("lineEdit_dd_lambda_b")
        self.lineEdit_dd_pi_s = QtWidgets.QLineEdit(self.groupBox_4)
        self.lineEdit_dd_pi_s.setGeometry(QtCore.QRect(250, 70, 113, 27))
        self.lineEdit_dd_pi_s.setText("")
        self.lineEdit_dd_pi_s.setReadOnly(True)
        self.lineEdit_dd_pi_s.setObjectName("lineEdit_dd_pi_s")
        self.label_dd_pi_c = QtWidgets.QLabel(self.groupBox_4)
        self.label_dd_pi_c.setGeometry(QtCore.QRect(0, 100, 241, 31))
        self.label_dd_pi_c.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_dd_pi_c.setObjectName("label_dd_pi_c")
        self.label_dd_lambda_b = QtWidgets.QLabel(self.groupBox_4)
        self.label_dd_lambda_b.setGeometry(QtCore.QRect(90, 10, 151, 31))
        self.label_dd_lambda_b.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_dd_lambda_b.setObjectName("label_dd_lambda_b")
        self.lineEdit_dd_pi_e = QtWidgets.QLineEdit(self.groupBox_4)
        self.lineEdit_dd_pi_e.setGeometry(QtCore.QRect(250, 160, 113, 27))
        self.lineEdit_dd_pi_e.setText("")
        self.lineEdit_dd_pi_e.setReadOnly(True)
        self.lineEdit_dd_pi_e.setObjectName("lineEdit_dd_pi_e")
        self.lineEdit_dd_pi_q = QtWidgets.QLineEdit(self.groupBox_4)
        self.lineEdit_dd_pi_q.setGeometry(QtCore.QRect(250, 130, 113, 27))
        self.lineEdit_dd_pi_q.setText("")
        self.lineEdit_dd_pi_q.setReadOnly(True)
        self.lineEdit_dd_pi_q.setObjectName("lineEdit_dd_pi_q")
        self.label_dd_pi_t = QtWidgets.QLabel(self.groupBox_4)
        self.label_dd_pi_t.setGeometry(QtCore.QRect(70, 40, 171, 31))
        self.label_dd_pi_t.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_dd_pi_t.setObjectName("label_dd_pi_t")
        self.addButton_c_ee_diode = QtWidgets.QPushButton(self.tab_diode)
        self.addButton_c_ee_diode.setGeometry(QtCore.QRect(925, 650, 125, 50))
        self.addButton_c_ee_diode.setObjectName("addButton_c_ee_diode")
        self.lineEdit_c_dd_warning_box = QtWidgets.QLineEdit(self.tab_diode)
        self.lineEdit_c_dd_warning_box.setGeometry(QtCore.QRect(10, 640, 330, 70))
        self.lineEdit_c_dd_warning_box.setReadOnly(True)
        self.lineEdit_c_dd_warning_box.setObjectName("lineEdit_c_dd_warning_box")
        self.tabWidget_electrical_equipments.addTab(self.tab_diode, "")
        self.tab_inductor = QtWidgets.QWidget()
        self.tab_inductor.setObjectName("tab_inductor")
        self.groupBox_id_lambda_p = QtWidgets.QGroupBox(self.tab_inductor)
        self.groupBox_id_lambda_p.setGeometry(QtCore.QRect(450, 640, 200, 70))
        self.groupBox_id_lambda_p.setObjectName("groupBox_id_lambda_p")
        self.lineEdit_id_lamda_p = QtWidgets.QLineEdit(self.groupBox_id_lambda_p)
        self.lineEdit_id_lamda_p.setGeometry(QtCore.QRect(0, 30, 200, 27))
        self.lineEdit_id_lamda_p.setReadOnly(True)
        self.lineEdit_id_lamda_p.setObjectName("lineEdit_id_lamda_p")
        self.groupBox_5 = QtWidgets.QGroupBox(self.tab_inductor)
        self.groupBox_5.setGeometry(QtCore.QRect(50, 40, 381, 131))
        self.groupBox_5.setTitle("")
        self.groupBox_5.setObjectName("groupBox_5")
        self.comboBox_id_type = QtWidgets.QComboBox(self.groupBox_5)
        self.comboBox_id_type.setGeometry(QtCore.QRect(180, 0, 191, 27))
        self.comboBox_id_type.setCurrentText("")
        self.comboBox_id_type.setObjectName("comboBox_id_type")
        self.comboBox_id_environment = QtWidgets.QComboBox(self.groupBox_5)
        self.comboBox_id_environment.setGeometry(QtCore.QRect(180, 90, 191, 27))
        self.comboBox_id_environment.setCurrentText("")
        self.comboBox_id_environment.setObjectName("comboBox_id_environment")
        self.label_id_environment = QtWidgets.QLabel(self.groupBox_5)
        self.label_id_environment.setGeometry(QtCore.QRect(0, 90, 171, 21))
        self.label_id_environment.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_id_environment.setObjectName("label_id_environment")
        self.comboBox_id_quality = QtWidgets.QComboBox(self.groupBox_5)
        self.comboBox_id_quality.setGeometry(QtCore.QRect(180, 60, 191, 27))
        self.comboBox_id_quality.setCurrentText("")
        self.comboBox_id_quality.setObjectName("comboBox_id_quality")
        self.label_id_inductor_type = QtWidgets.QLabel(self.groupBox_5)
        self.label_id_inductor_type.setGeometry(QtCore.QRect(0, 0, 171, 21))
        self.label_id_inductor_type.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_id_inductor_type.setObjectName("label_id_inductor_type")
        self.label_id_quality = QtWidgets.QLabel(self.groupBox_5)
        self.label_id_quality.setGeometry(QtCore.QRect(110, 60, 61, 21))
        self.label_id_quality.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_id_quality.setObjectName("label_id_quality")
        self.label_id_hot_spot_temperature = QtWidgets.QLabel(self.groupBox_5)
        self.label_id_hot_spot_temperature.setGeometry(QtCore.QRect(0, 30, 171, 21))
        self.label_id_hot_spot_temperature.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_id_hot_spot_temperature.setObjectName("label_id_hot_spot_temperature")
        self.comboBox_id_hot_spot_temperature = QtWidgets.QComboBox(self.groupBox_5)
        self.comboBox_id_hot_spot_temperature.setGeometry(QtCore.QRect(180, 30, 191, 27))
        self.comboBox_id_hot_spot_temperature.setCurrentText("")
        self.comboBox_id_hot_spot_temperature.setObjectName("comboBox_id_hot_spot_temperature")
        self.groupBox_11 = QtWidgets.QGroupBox(self.tab_inductor)
        self.groupBox_11.setGeometry(QtCore.QRect(540, 40, 301, 131))
        self.groupBox_11.setTitle("")
        self.groupBox_11.setObjectName("groupBox_11")
        self.lineEdit_id_lambda_b = QtWidgets.QLineEdit(self.groupBox_11)
        self.lineEdit_id_lambda_b.setGeometry(QtCore.QRect(180, 0, 113, 27))
        self.lineEdit_id_lambda_b.setReadOnly(True)
        self.lineEdit_id_lambda_b.setObjectName("lineEdit_id_lambda_b")
        self.label_id_pi_q = QtWidgets.QLabel(self.groupBox_11)
        self.label_id_pi_q.setGeometry(QtCore.QRect(30, 60, 141, 31))
        self.label_id_pi_q.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_id_pi_q.setObjectName("label_id_pi_q")
        self.lineEdit_id_pi_e = QtWidgets.QLineEdit(self.groupBox_11)
        self.lineEdit_id_pi_e.setGeometry(QtCore.QRect(180, 90, 113, 27))
        self.lineEdit_id_pi_e.setText("")
        self.lineEdit_id_pi_e.setReadOnly(True)
        self.lineEdit_id_pi_e.setObjectName("lineEdit_id_pi_e")
        self.lineEdit_id_pi_t = QtWidgets.QLineEdit(self.groupBox_11)
        self.lineEdit_id_pi_t.setGeometry(QtCore.QRect(180, 30, 113, 27))
        self.lineEdit_id_pi_t.setText("")
        self.lineEdit_id_pi_t.setReadOnly(True)
        self.lineEdit_id_pi_t.setObjectName("lineEdit_id_pi_t")
        self.label_id_lambda_b = QtWidgets.QLabel(self.groupBox_11)
        self.label_id_lambda_b.setGeometry(QtCore.QRect(20, 0, 151, 31))
        self.label_id_lambda_b.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_id_lambda_b.setObjectName("label_id_lambda_b")
        self.label_id_pi_t = QtWidgets.QLabel(self.groupBox_11)
        self.label_id_pi_t.setGeometry(QtCore.QRect(0, 30, 171, 31))
        self.label_id_pi_t.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_id_pi_t.setObjectName("label_id_pi_t")
        self.lineEdit_id_pi_q = QtWidgets.QLineEdit(self.groupBox_11)
        self.lineEdit_id_pi_q.setGeometry(QtCore.QRect(180, 60, 113, 27))
        self.lineEdit_id_pi_q.setText("")
        self.lineEdit_id_pi_q.setReadOnly(True)
        self.lineEdit_id_pi_q.setObjectName("lineEdit_id_pi_q")
        self.label_id_pi_e = QtWidgets.QLabel(self.groupBox_11)
        self.label_id_pi_e.setGeometry(QtCore.QRect(0, 90, 171, 31))
        self.label_id_pi_e.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_id_pi_e.setObjectName("label_id_pi_e")
        self.addButton_c_ee_inductor = QtWidgets.QPushButton(self.tab_inductor)
        self.addButton_c_ee_inductor.setGeometry(QtCore.QRect(925, 650, 125, 50))
        self.addButton_c_ee_inductor.setObjectName("addButton_c_ee_inductor")
        self.lineEdit_c_id_warning_box = QtWidgets.QLineEdit(self.tab_inductor)
        self.lineEdit_c_id_warning_box.setGeometry(QtCore.QRect(10, 640, 330, 70))
        self.lineEdit_c_id_warning_box.setReadOnly(True)
        self.lineEdit_c_id_warning_box.setObjectName("lineEdit_c_id_warning_box")
        self.tabWidget_electrical_equipments.addTab(self.tab_inductor, "")
        self.tab_transistor = QtWidgets.QWidget()
        self.tab_transistor.setObjectName("tab_transistor")
        self.groupBox_ts_lambda_p = QtWidgets.QGroupBox(self.tab_transistor)
        self.groupBox_ts_lambda_p.setGeometry(QtCore.QRect(450, 640, 200, 70))
        self.groupBox_ts_lambda_p.setObjectName("groupBox_ts_lambda_p")
        self.lineEdit_ts_lamda_p = QtWidgets.QLineEdit(self.groupBox_ts_lambda_p)
        self.lineEdit_ts_lamda_p.setGeometry(QtCore.QRect(0, 30, 200, 27))
        self.lineEdit_ts_lamda_p.setReadOnly(True)
        self.lineEdit_ts_lamda_p.setObjectName("lineEdit_ts_lamda_p")
        self.groupBox_12 = QtWidgets.QGroupBox(self.tab_transistor)
        self.groupBox_12.setGeometry(QtCore.QRect(70, 40, 361, 131))
        self.groupBox_12.setTitle("")
        self.groupBox_12.setObjectName("groupBox_12")
        self.label_ts_transistor_type = QtWidgets.QLabel(self.groupBox_12)
        self.label_ts_transistor_type.setGeometry(QtCore.QRect(40, 0, 111, 21))
        self.label_ts_transistor_type.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_ts_transistor_type.setObjectName("label_ts_transistor_type")
        self.comboBox_ts_environment = QtWidgets.QComboBox(self.groupBox_12)
        self.comboBox_ts_environment.setGeometry(QtCore.QRect(160, 90, 191, 27))
        self.comboBox_ts_environment.setCurrentText("")
        self.comboBox_ts_environment.setObjectName("comboBox_ts_environment")
        self.label_ts_quality = QtWidgets.QLabel(self.groupBox_12)
        self.label_ts_quality.setGeometry(QtCore.QRect(90, 30, 61, 21))
        self.label_ts_quality.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_ts_quality.setObjectName("label_ts_quality")
        self.comboBox_ts_junction_temperature = QtWidgets.QComboBox(self.groupBox_12)
        self.comboBox_ts_junction_temperature.setGeometry(QtCore.QRect(160, 60, 191, 27))
        self.comboBox_ts_junction_temperature.setCurrentText("")
        self.comboBox_ts_junction_temperature.setObjectName("comboBox_ts_junction_temperature")
        self.label_ts_environment = QtWidgets.QLabel(self.groupBox_12)
        self.label_ts_environment.setGeometry(QtCore.QRect(60, 90, 91, 21))
        self.label_ts_environment.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_ts_environment.setObjectName("label_ts_environment")
        self.comboBox_ts_quality = QtWidgets.QComboBox(self.groupBox_12)
        self.comboBox_ts_quality.setGeometry(QtCore.QRect(160, 30, 191, 27))
        self.comboBox_ts_quality.setCurrentText("")
        self.comboBox_ts_quality.setObjectName("comboBox_ts_quality")
        self.comboBox_ts_type = QtWidgets.QComboBox(self.groupBox_12)
        self.comboBox_ts_type.setGeometry(QtCore.QRect(160, 0, 191, 27))
        self.comboBox_ts_type.setCurrentText("")
        self.comboBox_ts_type.setObjectName("comboBox_ts_type")
        self.label_ts_junction_temperature = QtWidgets.QLabel(self.groupBox_12)
        self.label_ts_junction_temperature.setGeometry(QtCore.QRect(0, 60, 151, 21))
        self.label_ts_junction_temperature.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_ts_junction_temperature.setObjectName("label_ts_junction_temperature")
        self.groupBox_13 = QtWidgets.QGroupBox(self.tab_transistor)
        self.groupBox_13.setGeometry(QtCore.QRect(540, 40, 301, 121))
        self.groupBox_13.setTitle("")
        self.groupBox_13.setObjectName("groupBox_13")
        self.label_ts_pi_q = QtWidgets.QLabel(self.groupBox_13)
        self.label_ts_pi_q.setGeometry(QtCore.QRect(30, 30, 141, 31))
        self.label_ts_pi_q.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_ts_pi_q.setObjectName("label_ts_pi_q")
        self.lineEdit_ts_pi_t = QtWidgets.QLineEdit(self.groupBox_13)
        self.lineEdit_ts_pi_t.setGeometry(QtCore.QRect(180, 60, 113, 27))
        self.lineEdit_ts_pi_t.setText("")
        self.lineEdit_ts_pi_t.setReadOnly(True)
        self.lineEdit_ts_pi_t.setObjectName("lineEdit_ts_pi_t")
        self.lineEdit_ts_lambda_b = QtWidgets.QLineEdit(self.groupBox_13)
        self.lineEdit_ts_lambda_b.setGeometry(QtCore.QRect(180, 0, 113, 27))
        self.lineEdit_ts_lambda_b.setReadOnly(True)
        self.lineEdit_ts_lambda_b.setObjectName("lineEdit_ts_lambda_b")
        self.label_ts_pi_t = QtWidgets.QLabel(self.groupBox_13)
        self.label_ts_pi_t.setGeometry(QtCore.QRect(0, 60, 171, 31))
        self.label_ts_pi_t.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_ts_pi_t.setObjectName("label_ts_pi_t")
        self.lineEdit_ts_pi_q = QtWidgets.QLineEdit(self.groupBox_13)
        self.lineEdit_ts_pi_q.setGeometry(QtCore.QRect(180, 30, 113, 27))
        self.lineEdit_ts_pi_q.setText("")
        self.lineEdit_ts_pi_q.setReadOnly(True)
        self.lineEdit_ts_pi_q.setObjectName("lineEdit_ts_pi_q")
        self.label_ts_lambda_b = QtWidgets.QLabel(self.groupBox_13)
        self.label_ts_lambda_b.setGeometry(QtCore.QRect(20, 0, 151, 31))
        self.label_ts_lambda_b.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_ts_lambda_b.setObjectName("label_ts_lambda_b")
        self.lineEdit_ts_pi_e = QtWidgets.QLineEdit(self.groupBox_13)
        self.lineEdit_ts_pi_e.setGeometry(QtCore.QRect(180, 90, 113, 27))
        self.lineEdit_ts_pi_e.setText("")
        self.lineEdit_ts_pi_e.setReadOnly(True)
        self.lineEdit_ts_pi_e.setObjectName("lineEdit_ts_pi_e")
        self.label_ts_pi_e = QtWidgets.QLabel(self.groupBox_13)
        self.label_ts_pi_e.setGeometry(QtCore.QRect(0, 90, 171, 31))
        self.label_ts_pi_e.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_ts_pi_e.setObjectName("label_ts_pi_e")
        self.addButton_c_ee_transistor = QtWidgets.QPushButton(self.tab_transistor)
        self.addButton_c_ee_transistor.setGeometry(QtCore.QRect(925, 650, 125, 50))
        self.addButton_c_ee_transistor.setObjectName("addButton_c_ee_transistor")
        self.lineEdit_c_ts_warning_box = QtWidgets.QLineEdit(self.tab_transistor)
        self.lineEdit_c_ts_warning_box.setGeometry(QtCore.QRect(10, 640, 330, 70))
        self.lineEdit_c_ts_warning_box.setReadOnly(True)
        self.lineEdit_c_ts_warning_box.setObjectName("lineEdit_c_ts_warning_box")
        self.tabWidget_electrical_equipments.addTab(self.tab_transistor, "")
        self.tab_fuse = QtWidgets.QWidget()
        self.tab_fuse.setObjectName("tab_fuse")
        self.groupBox_fs_lambda_p = QtWidgets.QGroupBox(self.tab_fuse)
        self.groupBox_fs_lambda_p.setGeometry(QtCore.QRect(450, 640, 200, 70))
        self.groupBox_fs_lambda_p.setObjectName("groupBox_fs_lambda_p")
        self.lineEdit_fs_lamda_p = QtWidgets.QLineEdit(self.groupBox_fs_lambda_p)
        self.lineEdit_fs_lamda_p.setGeometry(QtCore.QRect(0, 30, 200, 27))
        self.lineEdit_fs_lamda_p.setReadOnly(True)
        self.lineEdit_fs_lamda_p.setObjectName("lineEdit_fs_lamda_p")
        self.groupBox_14 = QtWidgets.QGroupBox(self.tab_fuse)
        self.groupBox_14.setGeometry(QtCore.QRect(130, 40, 301, 61))
        self.groupBox_14.setTitle("")
        self.groupBox_14.setObjectName("groupBox_14")
        self.comboBox_fs_type = QtWidgets.QComboBox(self.groupBox_14)
        self.comboBox_fs_type.setGeometry(QtCore.QRect(100, 0, 191, 27))
        self.comboBox_fs_type.setCurrentText("")
        self.comboBox_fs_type.setObjectName("comboBox_fs_type")
        self.label_fs_environment = QtWidgets.QLabel(self.groupBox_14)
        self.label_fs_environment.setGeometry(QtCore.QRect(0, 30, 91, 21))
        self.label_fs_environment.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_fs_environment.setObjectName("label_fs_environment")
        self.comboBox_fs_environment = QtWidgets.QComboBox(self.groupBox_14)
        self.comboBox_fs_environment.setGeometry(QtCore.QRect(100, 30, 191, 27))
        self.comboBox_fs_environment.setCurrentText("")
        self.comboBox_fs_environment.setObjectName("comboBox_fs_environment")
        self.label_fs_fuse_type = QtWidgets.QLabel(self.groupBox_14)
        self.label_fs_fuse_type.setGeometry(QtCore.QRect(20, 0, 71, 21))
        self.label_fs_fuse_type.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_fs_fuse_type.setObjectName("label_fs_fuse_type")
        self.groupBox_15 = QtWidgets.QGroupBox(self.tab_fuse)
        self.groupBox_15.setGeometry(QtCore.QRect(540, 40, 301, 61))
        self.groupBox_15.setTitle("")
        self.groupBox_15.setObjectName("groupBox_15")
        self.label_fs_pi_e = QtWidgets.QLabel(self.groupBox_15)
        self.label_fs_pi_e.setGeometry(QtCore.QRect(0, 30, 171, 31))
        self.label_fs_pi_e.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_fs_pi_e.setObjectName("label_fs_pi_e")
        self.lineEdit_fs_pi_e = QtWidgets.QLineEdit(self.groupBox_15)
        self.lineEdit_fs_pi_e.setGeometry(QtCore.QRect(180, 30, 113, 27))
        self.lineEdit_fs_pi_e.setText("")
        self.lineEdit_fs_pi_e.setReadOnly(True)
        self.lineEdit_fs_pi_e.setObjectName("lineEdit_fs_pi_e")
        self.lineEdit_fs_lambda_b = QtWidgets.QLineEdit(self.groupBox_15)
        self.lineEdit_fs_lambda_b.setGeometry(QtCore.QRect(180, 0, 113, 27))
        self.lineEdit_fs_lambda_b.setReadOnly(True)
        self.lineEdit_fs_lambda_b.setObjectName("lineEdit_fs_lambda_b")
        self.label_fs_lambda_b = QtWidgets.QLabel(self.groupBox_15)
        self.label_fs_lambda_b.setGeometry(QtCore.QRect(20, 0, 151, 31))
        self.label_fs_lambda_b.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_fs_lambda_b.setObjectName("label_fs_lambda_b")
        self.addButton_c_ee_fuse = QtWidgets.QPushButton(self.tab_fuse)
        self.addButton_c_ee_fuse.setGeometry(QtCore.QRect(925, 650, 125, 50))
        self.addButton_c_ee_fuse.setObjectName("addButton_c_ee_fuse")
        self.lineEdit_c_fs_warning_box = QtWidgets.QLineEdit(self.tab_fuse)
        self.lineEdit_c_fs_warning_box.setGeometry(QtCore.QRect(10, 640, 330, 70))
        self.lineEdit_c_fs_warning_box.setReadOnly(True)
        self.lineEdit_c_fs_warning_box.setObjectName("lineEdit_c_fs_warning_box")
        self.tabWidget_electrical_equipments.addTab(self.tab_fuse, "")
        self.tab_resistor = QtWidgets.QWidget()
        self.tab_resistor.setObjectName("tab_resistor")
        self.groupBox_16 = QtWidgets.QGroupBox(self.tab_resistor)
        self.groupBox_16.setGeometry(QtCore.QRect(90, 40, 431, 221))
        self.groupBox_16.setTitle("")
        self.groupBox_16.setObjectName("groupBox_16")
        self.comboBox_res_style = QtWidgets.QComboBox(self.groupBox_16)
        self.comboBox_res_style.setGeometry(QtCore.QRect(220, 10, 191, 27))
        self.comboBox_res_style.setCurrentText("")
        self.comboBox_res_style.setObjectName("comboBox_res_style")
        self.label_res_style = QtWidgets.QLabel(self.groupBox_16)
        self.label_res_style.setGeometry(QtCore.QRect(90, 10, 121, 21))
        self.label_res_style.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_res_style.setObjectName("label_res_style")
        self.label_res_t = QtWidgets.QLabel(self.groupBox_16)
        self.label_res_t.setGeometry(QtCore.QRect(-10, 30, 221, 41))
        self.label_res_t.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_res_t.setWordWrap(True)
        self.label_res_t.setObjectName("label_res_t")
        self.comboBox_res_w = QtWidgets.QComboBox(self.groupBox_16)
        self.comboBox_res_w.setGeometry(QtCore.QRect(220, 70, 70, 27))
        self.comboBox_res_w.setCurrentText("")
        self.comboBox_res_w.setObjectName("comboBox_res_w")
        self.label_res_w = QtWidgets.QLabel(self.groupBox_16)
        self.label_res_w.setGeometry(QtCore.QRect(20, 70, 191, 21))
        self.label_res_w.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_res_w.setWordWrap(True)
        self.label_res_w.setObjectName("label_res_w")
        self.comboBox_res_environment = QtWidgets.QComboBox(self.groupBox_16)
        self.comboBox_res_environment.setGeometry(QtCore.QRect(220, 130, 191, 27))
        self.comboBox_res_environment.setCurrentText("")
        self.comboBox_res_environment.setObjectName("comboBox_res_environment")
        self.label_res_environment = QtWidgets.QLabel(self.groupBox_16)
        self.label_res_environment.setGeometry(QtCore.QRect(80, 120, 131, 41))
        self.label_res_environment.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_res_environment.setWordWrap(True)
        self.label_res_environment.setObjectName("label_res_environment")
        self.label_res_quality = QtWidgets.QLabel(self.groupBox_16)
        self.label_res_quality.setGeometry(QtCore.QRect(80, 150, 131, 41))
        self.label_res_quality.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_res_quality.setWordWrap(True)
        self.label_res_quality.setObjectName("label_res_quality")
        self.comboBox_res_quality = QtWidgets.QComboBox(self.groupBox_16)
        self.comboBox_res_quality.setGeometry(QtCore.QRect(220, 160, 191, 27))
        self.comboBox_res_quality.setCurrentText("")
        self.comboBox_res_quality.setObjectName("comboBox_res_quality")
        self.comboBox_res_case_temp = QtWidgets.QComboBox(self.groupBox_16)
        self.comboBox_res_case_temp.setGeometry(QtCore.QRect(220, 40, 70, 27))
        self.comboBox_res_case_temp.setCurrentText("")
        self.comboBox_res_case_temp.setObjectName("comboBox_res_case_temp")
        self.lineEdit_res_case_temp = QtWidgets.QLineEdit(self.groupBox_16)
        self.lineEdit_res_case_temp.setEnabled(False)
        self.lineEdit_res_case_temp.setGeometry(QtCore.QRect(330, 40, 70, 27))
        self.lineEdit_res_case_temp.setDragEnabled(False)
        self.lineEdit_res_case_temp.setObjectName("lineEdit_res_case_temp")
        self.label_30 = QtWidgets.QLabel(self.groupBox_16)
        self.label_30.setEnabled(False)
        self.label_30.setGeometry(QtCore.QRect(300, 40, 21, 31))
        self.label_30.setObjectName("label_30")
        self.lineEdit_res_w = QtWidgets.QLineEdit(self.groupBox_16)
        self.lineEdit_res_w.setEnabled(False)
        self.lineEdit_res_w.setGeometry(QtCore.QRect(330, 70, 70, 27))
        self.lineEdit_res_w.setDragEnabled(False)
        self.lineEdit_res_w.setObjectName("lineEdit_res_w")
        self.label_40 = QtWidgets.QLabel(self.groupBox_16)
        self.label_40.setEnabled(False)
        self.label_40.setGeometry(QtCore.QRect(300, 70, 21, 31))
        self.label_40.setObjectName("label_40")
        self.comboBox_res_stress = QtWidgets.QComboBox(self.groupBox_16)
        self.comboBox_res_stress.setGeometry(QtCore.QRect(220, 100, 70, 27))
        self.comboBox_res_stress.setCurrentText("")
        self.comboBox_res_stress.setObjectName("comboBox_res_stress")
        self.label_res_stress = QtWidgets.QLabel(self.groupBox_16)
        self.label_res_stress.setGeometry(QtCore.QRect(20, 100, 191, 21))
        self.label_res_stress.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_res_stress.setWordWrap(True)
        self.label_res_stress.setObjectName("label_res_stress")
        self.lineEdit_res_stress = QtWidgets.QLineEdit(self.groupBox_16)
        self.lineEdit_res_stress.setEnabled(False)
        self.lineEdit_res_stress.setGeometry(QtCore.QRect(330, 100, 70, 27))
        self.lineEdit_res_stress.setDragEnabled(False)
        self.lineEdit_res_stress.setObjectName("lineEdit_res_stress")
        self.label_41 = QtWidgets.QLabel(self.groupBox_16)
        self.label_41.setEnabled(False)
        self.label_41.setGeometry(QtCore.QRect(300, 100, 21, 31))
        self.label_41.setObjectName("label_41")
        self.groupBox_17 = QtWidgets.QGroupBox(self.tab_resistor)
        self.groupBox_17.setGeometry(QtCore.QRect(580, 50, 301, 291))
        self.groupBox_17.setTitle("")
        self.groupBox_17.setObjectName("groupBox_17")
        self.lineEdit_res_lambda_b = QtWidgets.QLineEdit(self.groupBox_17)
        self.lineEdit_res_lambda_b.setGeometry(QtCore.QRect(180, 0, 113, 27))
        self.lineEdit_res_lambda_b.setReadOnly(True)
        self.lineEdit_res_lambda_b.setObjectName("lineEdit_res_lambda_b")
        self.label_res_lambda_b = QtWidgets.QLabel(self.groupBox_17)
        self.label_res_lambda_b.setGeometry(QtCore.QRect(20, 0, 151, 31))
        self.label_res_lambda_b.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_res_lambda_b.setObjectName("label_res_lambda_b")
        self.lineEdit_res_pi_t = QtWidgets.QLineEdit(self.groupBox_17)
        self.lineEdit_res_pi_t.setGeometry(QtCore.QRect(180, 30, 113, 27))
        self.lineEdit_res_pi_t.setText("")
        self.lineEdit_res_pi_t.setReadOnly(True)
        self.lineEdit_res_pi_t.setObjectName("lineEdit_res_pi_t")
        self.label_res_pi_t = QtWidgets.QLabel(self.groupBox_17)
        self.label_res_pi_t.setGeometry(QtCore.QRect(0, 30, 171, 31))
        self.label_res_pi_t.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_res_pi_t.setObjectName("label_res_pi_t")
        self.label_res_pi_p = QtWidgets.QLabel(self.groupBox_17)
        self.label_res_pi_p.setGeometry(QtCore.QRect(0, 60, 171, 31))
        self.label_res_pi_p.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_res_pi_p.setObjectName("label_res_pi_p")
        self.lineEdit_res_pi_p = QtWidgets.QLineEdit(self.groupBox_17)
        self.lineEdit_res_pi_p.setGeometry(QtCore.QRect(180, 60, 113, 27))
        self.lineEdit_res_pi_p.setText("")
        self.lineEdit_res_pi_p.setReadOnly(True)
        self.lineEdit_res_pi_p.setObjectName("lineEdit_res_pi_p")
        self.label_res_pi_e = QtWidgets.QLabel(self.groupBox_17)
        self.label_res_pi_e.setGeometry(QtCore.QRect(0, 120, 171, 31))
        self.label_res_pi_e.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_res_pi_e.setObjectName("label_res_pi_e")
        self.lineEdit_res_pi_e = QtWidgets.QLineEdit(self.groupBox_17)
        self.lineEdit_res_pi_e.setGeometry(QtCore.QRect(180, 120, 113, 27))
        self.lineEdit_res_pi_e.setText("")
        self.lineEdit_res_pi_e.setReadOnly(True)
        self.lineEdit_res_pi_e.setObjectName("lineEdit_res_pi_e")
        self.label_res_pi_q = QtWidgets.QLabel(self.groupBox_17)
        self.label_res_pi_q.setGeometry(QtCore.QRect(0, 150, 171, 31))
        self.label_res_pi_q.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_res_pi_q.setObjectName("label_res_pi_q")
        self.lineEdit_res_pi_q = QtWidgets.QLineEdit(self.groupBox_17)
        self.lineEdit_res_pi_q.setGeometry(QtCore.QRect(180, 150, 113, 27))
        self.lineEdit_res_pi_q.setText("")
        self.lineEdit_res_pi_q.setReadOnly(True)
        self.lineEdit_res_pi_q.setObjectName("lineEdit_res_pi_q")
        self.label_res_pi_s = QtWidgets.QLabel(self.groupBox_17)
        self.label_res_pi_s.setGeometry(QtCore.QRect(0, 90, 171, 31))
        self.label_res_pi_s.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_res_pi_s.setObjectName("label_res_pi_s")
        self.lineEdit_res_pi_s = QtWidgets.QLineEdit(self.groupBox_17)
        self.lineEdit_res_pi_s.setGeometry(QtCore.QRect(180, 90, 113, 27))
        self.lineEdit_res_pi_s.setText("")
        self.lineEdit_res_pi_s.setReadOnly(True)
        self.lineEdit_res_pi_s.setObjectName("lineEdit_res_pi_s")
        self.lineEdit_c_res_warning_box = QtWidgets.QLineEdit(self.tab_resistor)
        self.lineEdit_c_res_warning_box.setGeometry(QtCore.QRect(15, 640, 330, 70))
        self.lineEdit_c_res_warning_box.setReadOnly(True)
        self.lineEdit_c_res_warning_box.setObjectName("lineEdit_c_res_warning_box")
        self.groupBox_res_lambda_p = QtWidgets.QGroupBox(self.tab_resistor)
        self.groupBox_res_lambda_p.setGeometry(QtCore.QRect(455, 640, 200, 70))
        self.groupBox_res_lambda_p.setObjectName("groupBox_res_lambda_p")
        self.lineEdit_res_lamda_p = QtWidgets.QLineEdit(self.groupBox_res_lambda_p)
        self.lineEdit_res_lamda_p.setGeometry(QtCore.QRect(0, 30, 200, 27))
        self.lineEdit_res_lamda_p.setReadOnly(True)
        self.lineEdit_res_lamda_p.setObjectName("lineEdit_res_lamda_p")
        self.addButton_c_ee_res = QtWidgets.QPushButton(self.tab_resistor)
        self.addButton_c_ee_res.setGeometry(QtCore.QRect(930, 650, 125, 50))
        self.addButton_c_ee_res.setObjectName("addButton_c_ee_res")
        self.tabWidget_electrical_equipments.addTab(self.tab_resistor, "")
        self.tab_rd = QtWidgets.QWidget()
        self.tab_rd.setObjectName("tab_rd")
        self.groupBox_bearings = QtWidgets.QGroupBox(self.tab_rd)
        self.groupBox_bearings.setGeometry(QtCore.QRect(50, 30, 470, 460))
        self.groupBox_bearings.setObjectName("groupBox_bearings")
        self.label_rd_bearing_ambient_temperature = QtWidgets.QLabel(self.groupBox_bearings)
        self.label_rd_bearing_ambient_temperature.setGeometry(QtCore.QRect(0, 40, 191, 21))
        self.label_rd_bearing_ambient_temperature.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_rd_bearing_ambient_temperature.setWordWrap(True)
        self.label_rd_bearing_ambient_temperature.setObjectName("label_rd_bearing_ambient_temperature")
        self.comboBox_rd_bearing_ambient_temperature = QtWidgets.QComboBox(self.groupBox_bearings)
        self.comboBox_rd_bearing_ambient_temperature.setGeometry(QtCore.QRect(200, 40, 70, 27))
        self.comboBox_rd_bearing_ambient_temperature.setCurrentText("")
        self.comboBox_rd_bearing_ambient_temperature.setObjectName("comboBox_rd_bearing_ambient_temperature")
        self.label_rd_bearing_characteristic_life = QtWidgets.QLabel(self.groupBox_bearings)
        self.label_rd_bearing_characteristic_life.setGeometry(QtCore.QRect(40, 70, 151, 31))
        self.label_rd_bearing_characteristic_life.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_rd_bearing_characteristic_life.setWordWrap(True)
        self.label_rd_bearing_characteristic_life.setObjectName("label_rd_bearing_characteristic_life")
        self.lineEdit_rd_bearing_characteristic_life = QtWidgets.QLineEdit(self.groupBox_bearings)
        self.lineEdit_rd_bearing_characteristic_life.setGeometry(QtCore.QRect(200, 80, 113, 27))
        self.lineEdit_rd_bearing_characteristic_life.setReadOnly(True)
        self.lineEdit_rd_bearing_characteristic_life.setObjectName("lineEdit_rd_bearing_characteristic_life")
        self.label_rd_bearing_motor_type = QtWidgets.QLabel(self.groupBox_bearings)
        self.label_rd_bearing_motor_type.setGeometry(QtCore.QRect(60, 140, 131, 21))
        self.label_rd_bearing_motor_type.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_rd_bearing_motor_type.setWordWrap(True)
        self.label_rd_bearing_motor_type.setObjectName("label_rd_bearing_motor_type")
        self.comboBox_rd_bearing_motor_type = QtWidgets.QComboBox(self.groupBox_bearings)
        self.comboBox_rd_bearing_motor_type.setGeometry(QtCore.QRect(200, 140, 181, 27))
        self.comboBox_rd_bearing_motor_type.setCurrentText("")
        self.comboBox_rd_bearing_motor_type.setObjectName("comboBox_rd_bearing_motor_type")
        self.lineEdit_rd_bearing_a_determination = QtWidgets.QLineEdit(self.groupBox_bearings)
        self.lineEdit_rd_bearing_a_determination.setGeometry(QtCore.QRect(200, 170, 113, 27))
        self.lineEdit_rd_bearing_a_determination.setReadOnly(True)
        self.lineEdit_rd_bearing_a_determination.setObjectName("lineEdit_rd_bearing_a_determination")
        self.label_rd_bearing_a_determination = QtWidgets.QLabel(self.groupBox_bearings)
        self.label_rd_bearing_a_determination.setGeometry(QtCore.QRect(60, 170, 131, 21))
        self.label_rd_bearing_a_determination.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_rd_bearing_a_determination.setWordWrap(True)
        self.label_rd_bearing_a_determination.setObjectName("label_rd_bearing_a_determination")
        self.label_rd_bearing_lambda1_determination = QtWidgets.QLabel(self.groupBox_bearings)
        self.label_rd_bearing_lambda1_determination.setGeometry(QtCore.QRect(60, 270, 131, 21))
        self.label_rd_bearing_lambda1_determination.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_rd_bearing_lambda1_determination.setWordWrap(True)
        self.label_rd_bearing_lambda1_determination.setObjectName("label_rd_bearing_lambda1_determination")
        self.lineEdit_rd_bearing_lambda1_determination = QtWidgets.QLineEdit(self.groupBox_bearings)
        self.lineEdit_rd_bearing_lambda1_determination.setGeometry(QtCore.QRect(200, 270, 113, 27))
        self.lineEdit_rd_bearing_lambda1_determination.setReadOnly(True)
        self.lineEdit_rd_bearing_lambda1_determination.setObjectName("lineEdit_rd_bearing_lambda1_determination")
        self.comboBox_rd_bearing_LC = QtWidgets.QComboBox(self.groupBox_bearings)
        self.comboBox_rd_bearing_LC.setGeometry(QtCore.QRect(200, 240, 181, 27))
        self.comboBox_rd_bearing_LC.setCurrentText("")
        self.comboBox_rd_bearing_LC.setObjectName("comboBox_rd_bearing_LC")
        self.label_rd_bearing_LC = QtWidgets.QLabel(self.groupBox_bearings)
        self.label_rd_bearing_LC.setGeometry(QtCore.QRect(60, 240, 131, 21))
        self.label_rd_bearing_LC.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_rd_bearing_LC.setWordWrap(True)
        self.label_rd_bearing_LC.setObjectName("label_rd_bearing_LC")
        self.groupBox_rd_bearing_fr = QtWidgets.QGroupBox(self.groupBox_bearings)
        self.groupBox_rd_bearing_fr.setGeometry(QtCore.QRect(120, 380, 200, 70))
        self.groupBox_rd_bearing_fr.setObjectName("groupBox_rd_bearing_fr")
        self.lineEdit_rd_bearing_fr = QtWidgets.QLineEdit(self.groupBox_rd_bearing_fr)
        self.lineEdit_rd_bearing_fr.setGeometry(QtCore.QRect(0, 30, 200, 27))
        self.lineEdit_rd_bearing_fr.setReadOnly(True)
        self.lineEdit_rd_bearing_fr.setObjectName("lineEdit_rd_bearing_fr")
        self.label_9 = QtWidgets.QLabel(self.groupBox_bearings)
        self.label_9.setEnabled(False)
        self.label_9.setGeometry(QtCore.QRect(280, 40, 21, 31))
        self.label_9.setObjectName("label_9")
        self.lineEdit_rd_bearing_ambient_temperature = QtWidgets.QLineEdit(self.groupBox_bearings)
        self.lineEdit_rd_bearing_ambient_temperature.setEnabled(False)
        self.lineEdit_rd_bearing_ambient_temperature.setGeometry(QtCore.QRect(310, 40, 70, 27))
        self.lineEdit_rd_bearing_ambient_temperature.setDragEnabled(False)
        self.lineEdit_rd_bearing_ambient_temperature.setObjectName("lineEdit_rd_bearing_ambient_temperature")
        self.line_12 = QtWidgets.QFrame(self.groupBox_bearings)
        self.line_12.setGeometry(QtCore.QRect(10, 120, 441, 16))
        self.line_12.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_12.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_12.setObjectName("line_12")
        self.line_33 = QtWidgets.QFrame(self.groupBox_bearings)
        self.line_33.setGeometry(QtCore.QRect(10, 210, 441, 16))
        self.line_33.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_33.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_33.setObjectName("line_33")
        self.groupBox_winding = QtWidgets.QGroupBox(self.tab_rd)
        self.groupBox_winding.setGeometry(QtCore.QRect(540, 30, 470, 460))
        self.groupBox_winding.setObjectName("groupBox_winding")
        self.label_rd_winding_ambient_temperature = QtWidgets.QLabel(self.groupBox_winding)
        self.label_rd_winding_ambient_temperature.setGeometry(QtCore.QRect(0, 40, 191, 21))
        self.label_rd_winding_ambient_temperature.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_rd_winding_ambient_temperature.setWordWrap(True)
        self.label_rd_winding_ambient_temperature.setObjectName("label_rd_winding_ambient_temperature")
        self.comboBox_rd_winding_ambient_temperature = QtWidgets.QComboBox(self.groupBox_winding)
        self.comboBox_rd_winding_ambient_temperature.setGeometry(QtCore.QRect(200, 40, 70, 27))
        self.comboBox_rd_winding_ambient_temperature.setCurrentText("")
        self.comboBox_rd_winding_ambient_temperature.setObjectName("comboBox_rd_winding_ambient_temperature")
        self.label_rd_winding_characteristic_life = QtWidgets.QLabel(self.groupBox_winding)
        self.label_rd_winding_characteristic_life.setGeometry(QtCore.QRect(40, 70, 151, 31))
        self.label_rd_winding_characteristic_life.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_rd_winding_characteristic_life.setWordWrap(True)
        self.label_rd_winding_characteristic_life.setObjectName("label_rd_winding_characteristic_life")
        self.lineEdit_rd_winding_characteristic_life = QtWidgets.QLineEdit(self.groupBox_winding)
        self.lineEdit_rd_winding_characteristic_life.setGeometry(QtCore.QRect(200, 80, 113, 27))
        self.lineEdit_rd_winding_characteristic_life.setReadOnly(True)
        self.lineEdit_rd_winding_characteristic_life.setObjectName("lineEdit_rd_winding_characteristic_life")
        self.label_rd_winding_motor_type = QtWidgets.QLabel(self.groupBox_winding)
        self.label_rd_winding_motor_type.setGeometry(QtCore.QRect(60, 140, 131, 21))
        self.label_rd_winding_motor_type.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_rd_winding_motor_type.setWordWrap(True)
        self.label_rd_winding_motor_type.setObjectName("label_rd_winding_motor_type")
        self.comboBox_rd_winding_motor_type = QtWidgets.QComboBox(self.groupBox_winding)
        self.comboBox_rd_winding_motor_type.setGeometry(QtCore.QRect(200, 140, 181, 27))
        self.comboBox_rd_winding_motor_type.setCurrentText("")
        self.comboBox_rd_winding_motor_type.setObjectName("comboBox_rd_winding_motor_type")
        self.lineEdit_rd_winding_b_determination = QtWidgets.QLineEdit(self.groupBox_winding)
        self.lineEdit_rd_winding_b_determination.setGeometry(QtCore.QRect(200, 170, 113, 27))
        self.lineEdit_rd_winding_b_determination.setReadOnly(True)
        self.lineEdit_rd_winding_b_determination.setObjectName("lineEdit_rd_winding_b_determination")
        self.label_rd_winding_b_determination = QtWidgets.QLabel(self.groupBox_winding)
        self.label_rd_winding_b_determination.setGeometry(QtCore.QRect(60, 170, 131, 21))
        self.label_rd_winding_b_determination.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_rd_winding_b_determination.setWordWrap(True)
        self.label_rd_winding_b_determination.setObjectName("label_rd_winding_b_determination")
        self.label_rd_winding_lambda2_determination = QtWidgets.QLabel(self.groupBox_winding)
        self.label_rd_winding_lambda2_determination.setGeometry(QtCore.QRect(60, 270, 131, 21))
        self.label_rd_winding_lambda2_determination.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_rd_winding_lambda2_determination.setWordWrap(True)
        self.label_rd_winding_lambda2_determination.setObjectName("label_rd_winding_lambda2_determination")
        self.lineEdit_rd_winding_lambda2_determination = QtWidgets.QLineEdit(self.groupBox_winding)
        self.lineEdit_rd_winding_lambda2_determination.setGeometry(QtCore.QRect(200, 270, 113, 27))
        self.lineEdit_rd_winding_lambda2_determination.setReadOnly(True)
        self.lineEdit_rd_winding_lambda2_determination.setObjectName("lineEdit_rd_winding_lambda2_determination")
        self.comboBox_rd_winding_LC = QtWidgets.QComboBox(self.groupBox_winding)
        self.comboBox_rd_winding_LC.setGeometry(QtCore.QRect(200, 240, 181, 27))
        self.comboBox_rd_winding_LC.setCurrentText("")
        self.comboBox_rd_winding_LC.setObjectName("comboBox_rd_winding_LC")
        self.label_rd_winding_LC = QtWidgets.QLabel(self.groupBox_winding)
        self.label_rd_winding_LC.setGeometry(QtCore.QRect(60, 240, 131, 21))
        self.label_rd_winding_LC.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_rd_winding_LC.setWordWrap(True)
        self.label_rd_winding_LC.setObjectName("label_rd_winding_LC")
        self.groupBox_rd_winding_fr = QtWidgets.QGroupBox(self.groupBox_winding)
        self.groupBox_rd_winding_fr.setGeometry(QtCore.QRect(120, 380, 200, 70))
        self.groupBox_rd_winding_fr.setObjectName("groupBox_rd_winding_fr")
        self.lineEdit_rd_winding_fr = QtWidgets.QLineEdit(self.groupBox_rd_winding_fr)
        self.lineEdit_rd_winding_fr.setGeometry(QtCore.QRect(0, 30, 200, 27))
        self.lineEdit_rd_winding_fr.setReadOnly(True)
        self.lineEdit_rd_winding_fr.setObjectName("lineEdit_rd_winding_fr")
        self.label_14 = QtWidgets.QLabel(self.groupBox_winding)
        self.label_14.setEnabled(False)
        self.label_14.setGeometry(QtCore.QRect(280, 40, 21, 31))
        self.label_14.setObjectName("label_14")
        self.lineEdit_rd_winding_ambient_temperature = QtWidgets.QLineEdit(self.groupBox_winding)
        self.lineEdit_rd_winding_ambient_temperature.setEnabled(False)
        self.lineEdit_rd_winding_ambient_temperature.setGeometry(QtCore.QRect(310, 40, 70, 27))
        self.lineEdit_rd_winding_ambient_temperature.setObjectName("lineEdit_rd_winding_ambient_temperature")
        self.line_34 = QtWidgets.QFrame(self.groupBox_winding)
        self.line_34.setGeometry(QtCore.QRect(0, 120, 441, 16))
        self.line_34.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_34.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_34.setObjectName("line_34")
        self.line_35 = QtWidgets.QFrame(self.groupBox_winding)
        self.line_35.setGeometry(QtCore.QRect(0, 210, 441, 16))
        self.line_35.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_35.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_35.setObjectName("line_35")
        self.addButton_c_ee_rd = QtWidgets.QPushButton(self.tab_rd)
        self.addButton_c_ee_rd.setGeometry(QtCore.QRect(930, 650, 125, 50))
        self.addButton_c_ee_rd.setObjectName("addButton_c_ee_rd")
        self.lineEdit_c_rd_warning_box = QtWidgets.QLineEdit(self.tab_rd)
        self.lineEdit_c_rd_warning_box.setGeometry(QtCore.QRect(15, 640, 330, 70))
        self.lineEdit_c_rd_warning_box.setReadOnly(True)
        self.lineEdit_c_rd_warning_box.setObjectName("lineEdit_c_rd_warning_box")
        self.groupBox_rd_lambda_p = QtWidgets.QGroupBox(self.tab_rd)
        self.groupBox_rd_lambda_p.setGeometry(QtCore.QRect(455, 640, 231, 70))
        self.groupBox_rd_lambda_p.setObjectName("groupBox_rd_lambda_p")
        self.lineEdit_rd_lamda_p = QtWidgets.QLineEdit(self.groupBox_rd_lambda_p)
        self.lineEdit_rd_lamda_p.setGeometry(QtCore.QRect(0, 30, 200, 27))
        self.lineEdit_rd_lamda_p.setReadOnly(True)
        self.lineEdit_rd_lamda_p.setObjectName("lineEdit_rd_lamda_p")
        self.line_13 = QtWidgets.QFrame(self.tab_rd)
        self.line_13.setGeometry(QtCore.QRect(510, 30, 20, 441))
        self.line_13.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_13.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_13.setObjectName("line_13")
        self.tabWidget_electrical_equipments.addTab(self.tab_rd, "")
        self.tab_rel = QtWidgets.QWidget()
        self.tab_rel.setObjectName("tab_rel")
        self.groupBox_18 = QtWidgets.QGroupBox(self.tab_rel)
        self.groupBox_18.setGeometry(QtCore.QRect(90, 40, 431, 511))
        self.groupBox_18.setTitle("")
        self.groupBox_18.setObjectName("groupBox_18")
        self.comboBox_rel_ambient_temperature = QtWidgets.QComboBox(self.groupBox_18)
        self.comboBox_rel_ambient_temperature.setGeometry(QtCore.QRect(230, 0, 70, 27))
        self.comboBox_rel_ambient_temperature.setCurrentText("")
        self.comboBox_rel_ambient_temperature.setObjectName("comboBox_rel_ambient_temperature")
        self.label_rel_ambient_temperature = QtWidgets.QLabel(self.groupBox_18)
        self.label_rel_ambient_temperature.setGeometry(QtCore.QRect(30, 0, 191, 21))
        self.label_rel_ambient_temperature.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_rel_ambient_temperature.setWordWrap(True)
        self.label_rel_ambient_temperature.setObjectName("label_rel_ambient_temperature")
        self.comboBox_rel_stress = QtWidgets.QComboBox(self.groupBox_18)
        self.comboBox_rel_stress.setGeometry(QtCore.QRect(230, 80, 70, 27))
        self.comboBox_rel_stress.setCurrentText("")
        self.comboBox_rel_stress.setObjectName("comboBox_rel_stress")
        self.label_rel_stress = QtWidgets.QLabel(self.groupBox_18)
        self.label_rel_stress.setGeometry(QtCore.QRect(90, 70, 131, 41))
        self.label_rel_stress.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_rel_stress.setWordWrap(True)
        self.label_rel_stress.setObjectName("label_rel_stress")
        self.comboBox_rel_environment = QtWidgets.QComboBox(self.groupBox_18)
        self.comboBox_rel_environment.setGeometry(QtCore.QRect(230, 320, 191, 27))
        self.comboBox_rel_environment.setCurrentText("")
        self.comboBox_rel_environment.setObjectName("comboBox_rel_environment")
        self.label_rel_environment = QtWidgets.QLabel(self.groupBox_18)
        self.label_rel_environment.setGeometry(QtCore.QRect(90, 310, 131, 41))
        self.label_rel_environment.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_rel_environment.setWordWrap(True)
        self.label_rel_environment.setObjectName("label_rel_environment")
        self.label_rel_quality = QtWidgets.QLabel(self.groupBox_18)
        self.label_rel_quality.setGeometry(QtCore.QRect(90, 280, 131, 41))
        self.label_rel_quality.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_rel_quality.setWordWrap(True)
        self.label_rel_quality.setObjectName("label_rel_quality")
        self.comboBox_rel_quality = QtWidgets.QComboBox(self.groupBox_18)
        self.comboBox_rel_quality.setGeometry(QtCore.QRect(230, 290, 191, 27))
        self.comboBox_rel_quality.setCurrentText("")
        self.comboBox_rel_quality.setObjectName("comboBox_rel_quality")
        self.comboBox_rel_rated_temperature = QtWidgets.QComboBox(self.groupBox_18)
        self.comboBox_rel_rated_temperature.setGeometry(QtCore.QRect(230, 30, 191, 27))
        self.comboBox_rel_rated_temperature.setCurrentText("")
        self.comboBox_rel_rated_temperature.setObjectName("comboBox_rel_rated_temperature")
        self.label_rel_rated_temperature = QtWidgets.QLabel(self.groupBox_18)
        self.label_rel_rated_temperature.setGeometry(QtCore.QRect(60, 30, 161, 21))
        self.label_rel_rated_temperature.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_rel_rated_temperature.setWordWrap(True)
        self.label_rel_rated_temperature.setObjectName("label_rel_rated_temperature")
        self.label_rel_load_type = QtWidgets.QLabel(self.groupBox_18)
        self.label_rel_load_type.setGeometry(QtCore.QRect(90, 100, 131, 41))
        self.label_rel_load_type.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_rel_load_type.setWordWrap(True)
        self.label_rel_load_type.setObjectName("label_rel_load_type")
        self.comboBox_rel_load_type = QtWidgets.QComboBox(self.groupBox_18)
        self.comboBox_rel_load_type.setGeometry(QtCore.QRect(230, 110, 191, 27))
        self.comboBox_rel_load_type.setCurrentText("")
        self.comboBox_rel_load_type.setObjectName("comboBox_rel_load_type")
        self.comboBox_rel_contact_form = QtWidgets.QComboBox(self.groupBox_18)
        self.comboBox_rel_contact_form.setGeometry(QtCore.QRect(230, 160, 191, 27))
        self.comboBox_rel_contact_form.setCurrentText("")
        self.comboBox_rel_contact_form.setObjectName("comboBox_rel_contact_form")
        self.label_rel_contact_form = QtWidgets.QLabel(self.groupBox_18)
        self.label_rel_contact_form.setGeometry(QtCore.QRect(90, 150, 131, 41))
        self.label_rel_contact_form.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_rel_contact_form.setWordWrap(True)
        self.label_rel_contact_form.setObjectName("label_rel_contact_form")
        self.label_rel_cycle_rate = QtWidgets.QLabel(self.groupBox_18)
        self.label_rel_cycle_rate.setGeometry(QtCore.QRect(90, 200, 131, 41))
        self.label_rel_cycle_rate.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_rel_cycle_rate.setWordWrap(True)
        self.label_rel_cycle_rate.setObjectName("label_rel_cycle_rate")
        self.comboBox_rel_cycle_rate = QtWidgets.QComboBox(self.groupBox_18)
        self.comboBox_rel_cycle_rate.setGeometry(QtCore.QRect(230, 210, 191, 27))
        self.comboBox_rel_cycle_rate.setCurrentText("")
        self.comboBox_rel_cycle_rate.setObjectName("comboBox_rel_cycle_rate")
        self.comboBox_rel_contact_rating = QtWidgets.QComboBox(self.groupBox_18)
        self.comboBox_rel_contact_rating.setGeometry(QtCore.QRect(230, 370, 191, 27))
        self.comboBox_rel_contact_rating.setCurrentText("")
        self.comboBox_rel_contact_rating.setObjectName("comboBox_rel_contact_rating")
        self.label_rel_contact_rating = QtWidgets.QLabel(self.groupBox_18)
        self.label_rel_contact_rating.setGeometry(QtCore.QRect(90, 360, 131, 41))
        self.label_rel_contact_rating.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_rel_contact_rating.setWordWrap(True)
        self.label_rel_contact_rating.setObjectName("label_rel_contact_rating")
        self.comboBox_rel_application_type = QtWidgets.QComboBox(self.groupBox_18)
        self.comboBox_rel_application_type.setGeometry(QtCore.QRect(230, 400, 191, 27))
        self.comboBox_rel_application_type.setCurrentText("")
        self.comboBox_rel_application_type.setObjectName("comboBox_rel_application_type")
        self.label_rel_application_type = QtWidgets.QLabel(self.groupBox_18)
        self.label_rel_application_type.setGeometry(QtCore.QRect(90, 390, 131, 41))
        self.label_rel_application_type.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_rel_application_type.setWordWrap(True)
        self.label_rel_application_type.setObjectName("label_rel_application_type")
        self.comboBox_rel_construction_type = QtWidgets.QComboBox(self.groupBox_18)
        self.comboBox_rel_construction_type.setGeometry(QtCore.QRect(230, 430, 191, 27))
        self.comboBox_rel_construction_type.setCurrentText("")
        self.comboBox_rel_construction_type.setObjectName("comboBox_rel_construction_type")
        self.label_rel_construction_type = QtWidgets.QLabel(self.groupBox_18)
        self.label_rel_construction_type.setGeometry(QtCore.QRect(90, 420, 131, 41))
        self.label_rel_construction_type.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_rel_construction_type.setWordWrap(True)
        self.label_rel_construction_type.setObjectName("label_rel_construction_type")
        self.label_28 = QtWidgets.QLabel(self.groupBox_18)
        self.label_28.setEnabled(False)
        self.label_28.setGeometry(QtCore.QRect(310, 0, 21, 31))
        self.label_28.setObjectName("label_28")
        self.lineEdit_rel_ambient_temperature = QtWidgets.QLineEdit(self.groupBox_18)
        self.lineEdit_rel_ambient_temperature.setEnabled(False)
        self.lineEdit_rel_ambient_temperature.setGeometry(QtCore.QRect(340, 0, 70, 27))
        self.lineEdit_rel_ambient_temperature.setDragEnabled(False)
        self.lineEdit_rel_ambient_temperature.setObjectName("lineEdit_rel_ambient_temperature")
        self.label_29 = QtWidgets.QLabel(self.groupBox_18)
        self.label_29.setEnabled(False)
        self.label_29.setGeometry(QtCore.QRect(310, 80, 21, 31))
        self.label_29.setObjectName("label_29")
        self.lineEdit_rel_stress = QtWidgets.QLineEdit(self.groupBox_18)
        self.lineEdit_rel_stress.setEnabled(False)
        self.lineEdit_rel_stress.setGeometry(QtCore.QRect(340, 80, 70, 27))
        self.lineEdit_rel_stress.setDragEnabled(False)
        self.lineEdit_rel_stress.setObjectName("lineEdit_rel_stress")
        self.lineEdit_rel_cycle_rate = QtWidgets.QLineEdit(self.groupBox_18)
        self.lineEdit_rel_cycle_rate.setEnabled(False)
        self.lineEdit_rel_cycle_rate.setGeometry(QtCore.QRect(230, 240, 70, 27))
        self.lineEdit_rel_cycle_rate.setDragEnabled(False)
        self.lineEdit_rel_cycle_rate.setObjectName("lineEdit_rel_cycle_rate")
        self.label_rel_cycle_rate_2 = QtWidgets.QLabel(self.groupBox_18)
        self.label_rel_cycle_rate_2.setEnabled(False)
        self.label_rel_cycle_rate_2.setGeometry(QtCore.QRect(90, 230, 131, 41))
        self.label_rel_cycle_rate_2.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_rel_cycle_rate_2.setWordWrap(True)
        self.label_rel_cycle_rate_2.setObjectName("label_rel_cycle_rate_2")
        self.groupBox_19 = QtWidgets.QGroupBox(self.tab_rel)
        self.groupBox_19.setGeometry(QtCore.QRect(570, 50, 301, 471))
        self.groupBox_19.setTitle("")
        self.groupBox_19.setObjectName("groupBox_19")
        self.lineEdit_rel_lambda_b = QtWidgets.QLineEdit(self.groupBox_19)
        self.lineEdit_rel_lambda_b.setGeometry(QtCore.QRect(180, 20, 113, 27))
        self.lineEdit_rel_lambda_b.setReadOnly(True)
        self.lineEdit_rel_lambda_b.setObjectName("lineEdit_rel_lambda_b")
        self.label_rel_lambda_b = QtWidgets.QLabel(self.groupBox_19)
        self.label_rel_lambda_b.setGeometry(QtCore.QRect(20, 20, 151, 31))
        self.label_rel_lambda_b.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_rel_lambda_b.setObjectName("label_rel_lambda_b")
        self.label_rel_pi_e = QtWidgets.QLabel(self.groupBox_19)
        self.label_rel_pi_e.setGeometry(QtCore.QRect(0, 310, 171, 31))
        self.label_rel_pi_e.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_rel_pi_e.setObjectName("label_rel_pi_e")
        self.lineEdit_rel_pi_e = QtWidgets.QLineEdit(self.groupBox_19)
        self.lineEdit_rel_pi_e.setGeometry(QtCore.QRect(180, 310, 113, 27))
        self.lineEdit_rel_pi_e.setText("")
        self.lineEdit_rel_pi_e.setReadOnly(True)
        self.lineEdit_rel_pi_e.setObjectName("lineEdit_rel_pi_e")
        self.label_rel_pi_q = QtWidgets.QLabel(self.groupBox_19)
        self.label_rel_pi_q.setGeometry(QtCore.QRect(0, 280, 171, 31))
        self.label_rel_pi_q.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_rel_pi_q.setObjectName("label_rel_pi_q")
        self.lineEdit_rel_pi_q = QtWidgets.QLineEdit(self.groupBox_19)
        self.lineEdit_rel_pi_q.setGeometry(QtCore.QRect(180, 280, 113, 27))
        self.lineEdit_rel_pi_q.setText("")
        self.lineEdit_rel_pi_q.setReadOnly(True)
        self.lineEdit_rel_pi_q.setObjectName("lineEdit_rel_pi_q")
        self.lineEdit_rel_pi_l = QtWidgets.QLineEdit(self.groupBox_19)
        self.lineEdit_rel_pi_l.setGeometry(QtCore.QRect(180, 100, 113, 27))
        self.lineEdit_rel_pi_l.setReadOnly(True)
        self.lineEdit_rel_pi_l.setObjectName("lineEdit_rel_pi_l")
        self.label_rel_pi_l = QtWidgets.QLabel(self.groupBox_19)
        self.label_rel_pi_l.setGeometry(QtCore.QRect(10, 100, 161, 31))
        self.label_rel_pi_l.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_rel_pi_l.setObjectName("label_rel_pi_l")
        self.lineEdit_rel_pi_c = QtWidgets.QLineEdit(self.groupBox_19)
        self.lineEdit_rel_pi_c.setGeometry(QtCore.QRect(180, 150, 113, 27))
        self.lineEdit_rel_pi_c.setReadOnly(True)
        self.lineEdit_rel_pi_c.setObjectName("lineEdit_rel_pi_c")
        self.label_rel_pi_c = QtWidgets.QLabel(self.groupBox_19)
        self.label_rel_pi_c.setGeometry(QtCore.QRect(0, 150, 171, 31))
        self.label_rel_pi_c.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_rel_pi_c.setObjectName("label_rel_pi_c")
        self.label_rel_pi_cyc = QtWidgets.QLabel(self.groupBox_19)
        self.label_rel_pi_cyc.setGeometry(QtCore.QRect(0, 200, 171, 31))
        self.label_rel_pi_cyc.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_rel_pi_cyc.setObjectName("label_rel_pi_cyc")
        self.lineEdit_rel_pi_cyc = QtWidgets.QLineEdit(self.groupBox_19)
        self.lineEdit_rel_pi_cyc.setGeometry(QtCore.QRect(180, 200, 113, 27))
        self.lineEdit_rel_pi_cyc.setReadOnly(True)
        self.lineEdit_rel_pi_cyc.setObjectName("lineEdit_rel_pi_cyc")
        self.label_rel_pi_f = QtWidgets.QLabel(self.groupBox_19)
        self.label_rel_pi_f.setGeometry(QtCore.QRect(0, 420, 171, 31))
        self.label_rel_pi_f.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_rel_pi_f.setObjectName("label_rel_pi_f")
        self.lineEdit_rel_pi_f = QtWidgets.QLineEdit(self.groupBox_19)
        self.lineEdit_rel_pi_f.setGeometry(QtCore.QRect(180, 420, 113, 27))
        self.lineEdit_rel_pi_f.setText("")
        self.lineEdit_rel_pi_f.setReadOnly(True)
        self.lineEdit_rel_pi_f.setObjectName("lineEdit_rel_pi_f")
        self.lineEdit_c_rel_warning_box = QtWidgets.QLineEdit(self.tab_rel)
        self.lineEdit_c_rel_warning_box.setGeometry(QtCore.QRect(15, 640, 330, 70))
        self.lineEdit_c_rel_warning_box.setReadOnly(True)
        self.lineEdit_c_rel_warning_box.setObjectName("lineEdit_c_rel_warning_box")
        self.groupBox_rel_lambda_p = QtWidgets.QGroupBox(self.tab_rel)
        self.groupBox_rel_lambda_p.setGeometry(QtCore.QRect(455, 640, 231, 70))
        self.groupBox_rel_lambda_p.setObjectName("groupBox_rel_lambda_p")
        self.lineEdit_rel_lamda_p = QtWidgets.QLineEdit(self.groupBox_rel_lambda_p)
        self.lineEdit_rel_lamda_p.setGeometry(QtCore.QRect(0, 30, 200, 27))
        self.lineEdit_rel_lamda_p.setReadOnly(True)
        self.lineEdit_rel_lamda_p.setObjectName("lineEdit_rel_lamda_p")
        self.addButton_c_ee_rel = QtWidgets.QPushButton(self.tab_rel)
        self.addButton_c_ee_rel.setGeometry(QtCore.QRect(930, 650, 125, 50))
        self.addButton_c_ee_rel.setObjectName("addButton_c_ee_rel")
        self.tabWidget_electrical_equipments.addTab(self.tab_rel, "")
        self.tab_con_gen = QtWidgets.QWidget()
        self.tab_con_gen.setObjectName("tab_con_gen")
        self.groupBox_20 = QtWidgets.QGroupBox(self.tab_con_gen)
        self.groupBox_20.setGeometry(QtCore.QRect(90, 50, 421, 231))
        self.groupBox_20.setTitle("")
        self.groupBox_20.setObjectName("groupBox_20")
        self.comboBox_con_gen_description = QtWidgets.QComboBox(self.groupBox_20)
        self.comboBox_con_gen_description.setGeometry(QtCore.QRect(220, 0, 191, 27))
        self.comboBox_con_gen_description.setCurrentText("")
        self.comboBox_con_gen_description.setObjectName("comboBox_con_gen_description")
        self.label_con_gen_description = QtWidgets.QLabel(self.groupBox_20)
        self.label_con_gen_description.setGeometry(QtCore.QRect(80, 0, 131, 31))
        self.label_con_gen_description.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_con_gen_description.setWordWrap(True)
        self.label_con_gen_description.setObjectName("label_con_gen_description")
        self.comboBox_con_gen_environment = QtWidgets.QComboBox(self.groupBox_20)
        self.comboBox_con_gen_environment.setGeometry(QtCore.QRect(220, 120, 191, 27))
        self.comboBox_con_gen_environment.setCurrentText("")
        self.comboBox_con_gen_environment.setObjectName("comboBox_con_gen_environment")
        self.label_c = QtWidgets.QLabel(self.groupBox_20)
        self.label_c.setGeometry(QtCore.QRect(80, 120, 131, 31))
        self.label_c.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_c.setWordWrap(True)
        self.label_c.setObjectName("label_c")
        self.label_con_gen_quality = QtWidgets.QLabel(self.groupBox_20)
        self.label_con_gen_quality.setGeometry(QtCore.QRect(80, 90, 131, 21))
        self.label_con_gen_quality.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_con_gen_quality.setWordWrap(True)
        self.label_con_gen_quality.setObjectName("label_con_gen_quality")
        self.comboBox_con_gen_quality = QtWidgets.QComboBox(self.groupBox_20)
        self.comboBox_con_gen_quality.setGeometry(QtCore.QRect(220, 90, 191, 27))
        self.comboBox_con_gen_quality.setCurrentText("")
        self.comboBox_con_gen_quality.setObjectName("comboBox_con_gen_quality")
        self.comboBox_con_gen_t0 = QtWidgets.QComboBox(self.groupBox_20)
        self.comboBox_con_gen_t0.setGeometry(QtCore.QRect(220, 30, 70, 27))
        self.comboBox_con_gen_t0.setCurrentText("")
        self.comboBox_con_gen_t0.setObjectName("comboBox_con_gen_t0")
        self.label_con_gen_t0 = QtWidgets.QLabel(self.groupBox_20)
        self.label_con_gen_t0.setGeometry(QtCore.QRect(80, 30, 131, 31))
        self.label_con_gen_t0.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_con_gen_t0.setWordWrap(True)
        self.label_con_gen_t0.setObjectName("label_con_gen_t0")
        self.label_con_gen_cycles = QtWidgets.QLabel(self.groupBox_20)
        self.label_con_gen_cycles.setGeometry(QtCore.QRect(30, 50, 181, 41))
        self.label_con_gen_cycles.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_con_gen_cycles.setWordWrap(True)
        self.label_con_gen_cycles.setObjectName("label_con_gen_cycles")
        self.comboBox_con_gen_cycles = QtWidgets.QComboBox(self.groupBox_20)
        self.comboBox_con_gen_cycles.setGeometry(QtCore.QRect(220, 60, 191, 27))
        self.comboBox_con_gen_cycles.setCurrentText("")
        self.comboBox_con_gen_cycles.setObjectName("comboBox_con_gen_cycles")
        self.lineEdit_con_gen_t0 = QtWidgets.QLineEdit(self.groupBox_20)
        self.lineEdit_con_gen_t0.setEnabled(False)
        self.lineEdit_con_gen_t0.setGeometry(QtCore.QRect(330, 30, 70, 27))
        self.lineEdit_con_gen_t0.setDragEnabled(False)
        self.lineEdit_con_gen_t0.setObjectName("lineEdit_con_gen_t0")
        self.label_17 = QtWidgets.QLabel(self.groupBox_20)
        self.label_17.setEnabled(False)
        self.label_17.setGeometry(QtCore.QRect(300, 30, 21, 31))
        self.label_17.setObjectName("label_17")
        self.groupBox_21 = QtWidgets.QGroupBox(self.tab_con_gen)
        self.groupBox_21.setGeometry(QtCore.QRect(540, 50, 361, 161))
        self.groupBox_21.setTitle("")
        self.groupBox_21.setObjectName("groupBox_21")
        self.lineEdit_con_gen_lambda_b = QtWidgets.QLineEdit(self.groupBox_21)
        self.lineEdit_con_gen_lambda_b.setGeometry(QtCore.QRect(220, 0, 113, 27))
        self.lineEdit_con_gen_lambda_b.setReadOnly(True)
        self.lineEdit_con_gen_lambda_b.setObjectName("lineEdit_con_gen_lambda_b")
        self.label_con_gen_lambda_b = QtWidgets.QLabel(self.groupBox_21)
        self.label_con_gen_lambda_b.setGeometry(QtCore.QRect(60, 0, 151, 31))
        self.label_con_gen_lambda_b.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_con_gen_lambda_b.setObjectName("label_con_gen_lambda_b")
        self.label_con_gen_pi_e = QtWidgets.QLabel(self.groupBox_21)
        self.label_con_gen_pi_e.setGeometry(QtCore.QRect(40, 120, 171, 21))
        self.label_con_gen_pi_e.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_con_gen_pi_e.setObjectName("label_con_gen_pi_e")
        self.lineEdit_con_gen_pi_e = QtWidgets.QLineEdit(self.groupBox_21)
        self.lineEdit_con_gen_pi_e.setGeometry(QtCore.QRect(220, 120, 113, 27))
        self.lineEdit_con_gen_pi_e.setText("")
        self.lineEdit_con_gen_pi_e.setReadOnly(True)
        self.lineEdit_con_gen_pi_e.setObjectName("lineEdit_con_gen_pi_e")
        self.label_con_gen_pi_q = QtWidgets.QLabel(self.groupBox_21)
        self.label_con_gen_pi_q.setGeometry(QtCore.QRect(40, 90, 171, 21))
        self.label_con_gen_pi_q.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_con_gen_pi_q.setObjectName("label_con_gen_pi_q")
        self.lineEdit_con_gen_pi_q = QtWidgets.QLineEdit(self.groupBox_21)
        self.lineEdit_con_gen_pi_q.setGeometry(QtCore.QRect(220, 90, 113, 27))
        self.lineEdit_con_gen_pi_q.setText("")
        self.lineEdit_con_gen_pi_q.setReadOnly(True)
        self.lineEdit_con_gen_pi_q.setObjectName("lineEdit_con_gen_pi_q")
        self.lineEdit_con_gen_pi_t = QtWidgets.QLineEdit(self.groupBox_21)
        self.lineEdit_con_gen_pi_t.setGeometry(QtCore.QRect(220, 30, 113, 27))
        self.lineEdit_con_gen_pi_t.setReadOnly(True)
        self.lineEdit_con_gen_pi_t.setObjectName("lineEdit_con_gen_pi_t")
        self.label_con_gen_pi_t = QtWidgets.QLabel(self.groupBox_21)
        self.label_con_gen_pi_t.setGeometry(QtCore.QRect(30, 30, 181, 31))
        self.label_con_gen_pi_t.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_con_gen_pi_t.setObjectName("label_con_gen_pi_t")
        self.label_con_gen_pi_k = QtWidgets.QLabel(self.groupBox_21)
        self.label_con_gen_pi_k.setGeometry(QtCore.QRect(-10, 60, 221, 31))
        self.label_con_gen_pi_k.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_con_gen_pi_k.setWordWrap(True)
        self.label_con_gen_pi_k.setObjectName("label_con_gen_pi_k")
        self.lineEdit_con_gen_pi_k = QtWidgets.QLineEdit(self.groupBox_21)
        self.lineEdit_con_gen_pi_k.setGeometry(QtCore.QRect(220, 60, 113, 27))
        self.lineEdit_con_gen_pi_k.setReadOnly(True)
        self.lineEdit_con_gen_pi_k.setObjectName("lineEdit_con_gen_pi_k")
        self.lineEdit_c_con_gen_warning_box = QtWidgets.QLineEdit(self.tab_con_gen)
        self.lineEdit_c_con_gen_warning_box.setGeometry(QtCore.QRect(15, 640, 330, 70))
        self.lineEdit_c_con_gen_warning_box.setReadOnly(True)
        self.lineEdit_c_con_gen_warning_box.setObjectName("lineEdit_c_con_gen_warning_box")
        self.groupBox_con_gen_lambda_p = QtWidgets.QGroupBox(self.tab_con_gen)
        self.groupBox_con_gen_lambda_p.setGeometry(QtCore.QRect(455, 640, 271, 70))
        self.groupBox_con_gen_lambda_p.setObjectName("groupBox_con_gen_lambda_p")
        self.lineEdit_con_gen_lamda_p = QtWidgets.QLineEdit(self.groupBox_con_gen_lambda_p)
        self.lineEdit_con_gen_lamda_p.setGeometry(QtCore.QRect(0, 30, 200, 27))
        self.lineEdit_con_gen_lamda_p.setReadOnly(True)
        self.lineEdit_con_gen_lamda_p.setObjectName("lineEdit_con_gen_lamda_p")
        self.addButton_c_ee_con_gen = QtWidgets.QPushButton(self.tab_con_gen)
        self.addButton_c_ee_con_gen.setGeometry(QtCore.QRect(930, 650, 125, 50))
        self.addButton_c_ee_con_gen.setObjectName("addButton_c_ee_con_gen")
        self.tabWidget_electrical_equipments.addTab(self.tab_con_gen, "")
        self.tab_con_sock = QtWidgets.QWidget()
        self.tab_con_sock.setObjectName("tab_con_sock")
        self.groupBox_22 = QtWidgets.QGroupBox(self.tab_con_sock)
        self.groupBox_22.setGeometry(QtCore.QRect(570, 50, 381, 131))
        self.groupBox_22.setTitle("")
        self.groupBox_22.setObjectName("groupBox_22")
        self.lineEdit_con_sock_lambda_b = QtWidgets.QLineEdit(self.groupBox_22)
        self.lineEdit_con_sock_lambda_b.setGeometry(QtCore.QRect(190, 0, 113, 27))
        self.lineEdit_con_sock_lambda_b.setReadOnly(True)
        self.lineEdit_con_sock_lambda_b.setObjectName("lineEdit_con_sock_lambda_b")
        self.label_con_sock_lambda_b = QtWidgets.QLabel(self.groupBox_22)
        self.label_con_sock_lambda_b.setGeometry(QtCore.QRect(30, 0, 151, 31))
        self.label_con_sock_lambda_b.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_con_sock_lambda_b.setObjectName("label_con_sock_lambda_b")
        self.label_con_sock_pi_e = QtWidgets.QLabel(self.groupBox_22)
        self.label_con_sock_pi_e.setGeometry(QtCore.QRect(10, 90, 171, 31))
        self.label_con_sock_pi_e.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_con_sock_pi_e.setObjectName("label_con_sock_pi_e")
        self.lineEdit_con_sock_pi_e = QtWidgets.QLineEdit(self.groupBox_22)
        self.lineEdit_con_sock_pi_e.setGeometry(QtCore.QRect(190, 90, 113, 27))
        self.lineEdit_con_sock_pi_e.setText("")
        self.lineEdit_con_sock_pi_e.setReadOnly(True)
        self.lineEdit_con_sock_pi_e.setObjectName("lineEdit_con_sock_pi_e")
        self.label_con_sock_pi_q = QtWidgets.QLabel(self.groupBox_22)
        self.label_con_sock_pi_q.setGeometry(QtCore.QRect(10, 30, 171, 31))
        self.label_con_sock_pi_q.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_con_sock_pi_q.setObjectName("label_con_sock_pi_q")
        self.lineEdit_con_sock_pi_q = QtWidgets.QLineEdit(self.groupBox_22)
        self.lineEdit_con_sock_pi_q.setGeometry(QtCore.QRect(190, 30, 113, 27))
        self.lineEdit_con_sock_pi_q.setText("")
        self.lineEdit_con_sock_pi_q.setReadOnly(True)
        self.lineEdit_con_sock_pi_q.setObjectName("lineEdit_con_sock_pi_q")
        self.lineEdit_con_sock_pi_p = QtWidgets.QLineEdit(self.groupBox_22)
        self.lineEdit_con_sock_pi_p.setGeometry(QtCore.QRect(190, 60, 113, 27))
        self.lineEdit_con_sock_pi_p.setReadOnly(True)
        self.lineEdit_con_sock_pi_p.setObjectName("lineEdit_con_sock_pi_p")
        self.label_con_sock_pi_p = QtWidgets.QLabel(self.groupBox_22)
        self.label_con_sock_pi_p.setGeometry(QtCore.QRect(0, 60, 181, 31))
        self.label_con_sock_pi_p.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_con_sock_pi_p.setObjectName("label_con_sock_pi_p")
        self.groupBox_23 = QtWidgets.QGroupBox(self.tab_con_sock)
        self.groupBox_23.setGeometry(QtCore.QRect(90, 50, 441, 131))
        self.groupBox_23.setTitle("")
        self.groupBox_23.setObjectName("groupBox_23")
        self.comboBox_con_sock_description = QtWidgets.QComboBox(self.groupBox_23)
        self.comboBox_con_sock_description.setGeometry(QtCore.QRect(220, 0, 191, 27))
        self.comboBox_con_sock_description.setCurrentText("")
        self.comboBox_con_sock_description.setObjectName("comboBox_con_sock_description")
        self.label_con_sock_description = QtWidgets.QLabel(self.groupBox_23)
        self.label_con_sock_description.setGeometry(QtCore.QRect(80, 0, 131, 31))
        self.label_con_sock_description.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_con_sock_description.setWordWrap(True)
        self.label_con_sock_description.setObjectName("label_con_sock_description")
        self.comboBox_con_sock_environment = QtWidgets.QComboBox(self.groupBox_23)
        self.comboBox_con_sock_environment.setGeometry(QtCore.QRect(220, 90, 191, 27))
        self.comboBox_con_sock_environment.setCurrentText("")
        self.comboBox_con_sock_environment.setObjectName("comboBox_con_sock_environment")
        self.label_con_sock_environment = QtWidgets.QLabel(self.groupBox_23)
        self.label_con_sock_environment.setGeometry(QtCore.QRect(80, 80, 131, 41))
        self.label_con_sock_environment.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_con_sock_environment.setWordWrap(True)
        self.label_con_sock_environment.setObjectName("label_con_sock_environment")
        self.label_con_sock_quality = QtWidgets.QLabel(self.groupBox_23)
        self.label_con_sock_quality.setGeometry(QtCore.QRect(80, 30, 131, 31))
        self.label_con_sock_quality.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_con_sock_quality.setWordWrap(True)
        self.label_con_sock_quality.setObjectName("label_con_sock_quality")
        self.comboBox_con_sock_quality = QtWidgets.QComboBox(self.groupBox_23)
        self.comboBox_con_sock_quality.setGeometry(QtCore.QRect(220, 30, 191, 27))
        self.comboBox_con_sock_quality.setCurrentText("")
        self.comboBox_con_sock_quality.setObjectName("comboBox_con_sock_quality")
        self.comboBox_con_sock_numb_of_active_contacts = QtWidgets.QComboBox(self.groupBox_23)
        self.comboBox_con_sock_numb_of_active_contacts.setGeometry(QtCore.QRect(220, 60, 70, 27))
        self.comboBox_con_sock_numb_of_active_contacts.setCurrentText("")
        self.comboBox_con_sock_numb_of_active_contacts.setObjectName("comboBox_con_sock_numb_of_active_contacts")
        self.label_con_sock_numb_of_active_coils = QtWidgets.QLabel(self.groupBox_23)
        self.label_con_sock_numb_of_active_coils.setGeometry(QtCore.QRect(50, 60, 161, 31))
        self.label_con_sock_numb_of_active_coils.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_con_sock_numb_of_active_coils.setWordWrap(True)
        self.label_con_sock_numb_of_active_coils.setObjectName("label_con_sock_numb_of_active_coils")
        self.label_16 = QtWidgets.QLabel(self.groupBox_23)
        self.label_16.setEnabled(False)
        self.label_16.setGeometry(QtCore.QRect(300, 60, 21, 31))
        self.label_16.setObjectName("label_16")
        self.lineEdit_con_sock_numb_of_active_contacts = QtWidgets.QLineEdit(self.groupBox_23)
        self.lineEdit_con_sock_numb_of_active_contacts.setEnabled(False)
        self.lineEdit_con_sock_numb_of_active_contacts.setGeometry(QtCore.QRect(330, 60, 70, 27))
        self.lineEdit_con_sock_numb_of_active_contacts.setDragEnabled(False)
        self.lineEdit_con_sock_numb_of_active_contacts.setObjectName("lineEdit_con_sock_numb_of_active_contacts")
        self.lineEdit_c_con_sock_warning_box = QtWidgets.QLineEdit(self.tab_con_sock)
        self.lineEdit_c_con_sock_warning_box.setGeometry(QtCore.QRect(15, 640, 330, 70))
        self.lineEdit_c_con_sock_warning_box.setReadOnly(True)
        self.lineEdit_c_con_sock_warning_box.setObjectName("lineEdit_c_con_sock_warning_box")
        self.groupBox_con_sock_lambda_p = QtWidgets.QGroupBox(self.tab_con_sock)
        self.groupBox_con_sock_lambda_p.setGeometry(QtCore.QRect(455, 640, 231, 70))
        self.groupBox_con_sock_lambda_p.setObjectName("groupBox_con_sock_lambda_p")
        self.lineEdit_con_sock_lamda_p = QtWidgets.QLineEdit(self.groupBox_con_sock_lambda_p)
        self.lineEdit_con_sock_lamda_p.setGeometry(QtCore.QRect(0, 30, 200, 27))
        self.lineEdit_con_sock_lamda_p.setReadOnly(True)
        self.lineEdit_con_sock_lamda_p.setObjectName("lineEdit_con_sock_lamda_p")
        self.addButton_c_ee_con_sock = QtWidgets.QPushButton(self.tab_con_sock)
        self.addButton_c_ee_con_sock.setGeometry(QtCore.QRect(930, 650, 125, 50))
        self.addButton_c_ee_con_sock.setObjectName("addButton_c_ee_con_sock")
        self.tabWidget_electrical_equipments.addTab(self.tab_con_sock, "")
        self.tab_qrtz = QtWidgets.QWidget()
        self.tab_qrtz.setObjectName("tab_qrtz")
        self.groupBox_24 = QtWidgets.QGroupBox(self.tab_qrtz)
        self.groupBox_24.setGeometry(QtCore.QRect(90, 50, 441, 131))
        self.groupBox_24.setTitle("")
        self.groupBox_24.setObjectName("groupBox_24")
        self.comboBox_qrtz_frequency = QtWidgets.QComboBox(self.groupBox_24)
        self.comboBox_qrtz_frequency.setGeometry(QtCore.QRect(220, 0, 70, 27))
        self.comboBox_qrtz_frequency.setCurrentText("")
        self.comboBox_qrtz_frequency.setObjectName("comboBox_qrtz_frequency")
        self.label_qrtz_frequency = QtWidgets.QLabel(self.groupBox_24)
        self.label_qrtz_frequency.setGeometry(QtCore.QRect(60, 0, 151, 31))
        self.label_qrtz_frequency.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_qrtz_frequency.setWordWrap(True)
        self.label_qrtz_frequency.setObjectName("label_qrtz_frequency")
        self.comboBox_qrtz_environment = QtWidgets.QComboBox(self.groupBox_24)
        self.comboBox_qrtz_environment.setGeometry(QtCore.QRect(220, 60, 191, 27))
        self.comboBox_qrtz_environment.setCurrentText("")
        self.comboBox_qrtz_environment.setObjectName("comboBox_qrtz_environment")
        self.label_qrtz_environment = QtWidgets.QLabel(self.groupBox_24)
        self.label_qrtz_environment.setGeometry(QtCore.QRect(80, 50, 131, 41))
        self.label_qrtz_environment.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_qrtz_environment.setWordWrap(True)
        self.label_qrtz_environment.setObjectName("label_qrtz_environment")
        self.label_qrtz_quality = QtWidgets.QLabel(self.groupBox_24)
        self.label_qrtz_quality.setGeometry(QtCore.QRect(80, 30, 131, 31))
        self.label_qrtz_quality.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_qrtz_quality.setWordWrap(True)
        self.label_qrtz_quality.setObjectName("label_qrtz_quality")
        self.comboBox_qrtz_quality = QtWidgets.QComboBox(self.groupBox_24)
        self.comboBox_qrtz_quality.setGeometry(QtCore.QRect(220, 30, 191, 27))
        self.comboBox_qrtz_quality.setCurrentText("")
        self.comboBox_qrtz_quality.setObjectName("comboBox_qrtz_quality")
        self.label_15 = QtWidgets.QLabel(self.groupBox_24)
        self.label_15.setEnabled(False)
        self.label_15.setGeometry(QtCore.QRect(300, 0, 21, 31))
        self.label_15.setObjectName("label_15")
        self.lineEdit_qrtz_frequency = QtWidgets.QLineEdit(self.groupBox_24)
        self.lineEdit_qrtz_frequency.setEnabled(False)
        self.lineEdit_qrtz_frequency.setGeometry(QtCore.QRect(330, 0, 70, 27))
        self.lineEdit_qrtz_frequency.setDragEnabled(False)
        self.lineEdit_qrtz_frequency.setObjectName("lineEdit_qrtz_frequency")
        self.groupBox_25 = QtWidgets.QGroupBox(self.tab_qrtz)
        self.groupBox_25.setGeometry(QtCore.QRect(570, 50, 381, 131))
        self.groupBox_25.setTitle("")
        self.groupBox_25.setObjectName("groupBox_25")
        self.lineEdit_qrtz_lambda_b = QtWidgets.QLineEdit(self.groupBox_25)
        self.lineEdit_qrtz_lambda_b.setGeometry(QtCore.QRect(190, 0, 113, 27))
        self.lineEdit_qrtz_lambda_b.setReadOnly(True)
        self.lineEdit_qrtz_lambda_b.setObjectName("lineEdit_qrtz_lambda_b")
        self.label_qrtz_lambda_b = QtWidgets.QLabel(self.groupBox_25)
        self.label_qrtz_lambda_b.setGeometry(QtCore.QRect(30, 0, 151, 31))
        self.label_qrtz_lambda_b.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_qrtz_lambda_b.setObjectName("label_qrtz_lambda_b")
        self.label_qrtz_pi_e = QtWidgets.QLabel(self.groupBox_25)
        self.label_qrtz_pi_e.setGeometry(QtCore.QRect(10, 60, 171, 31))
        self.label_qrtz_pi_e.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_qrtz_pi_e.setObjectName("label_qrtz_pi_e")
        self.lineEdit_qrtz_pi_e = QtWidgets.QLineEdit(self.groupBox_25)
        self.lineEdit_qrtz_pi_e.setGeometry(QtCore.QRect(190, 60, 113, 27))
        self.lineEdit_qrtz_pi_e.setText("")
        self.lineEdit_qrtz_pi_e.setReadOnly(True)
        self.lineEdit_qrtz_pi_e.setObjectName("lineEdit_qrtz_pi_e")
        self.label_qrtz_pi_q = QtWidgets.QLabel(self.groupBox_25)
        self.label_qrtz_pi_q.setGeometry(QtCore.QRect(10, 30, 171, 31))
        self.label_qrtz_pi_q.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_qrtz_pi_q.setObjectName("label_qrtz_pi_q")
        self.lineEdit_qrtz_pi_q = QtWidgets.QLineEdit(self.groupBox_25)
        self.lineEdit_qrtz_pi_q.setGeometry(QtCore.QRect(190, 30, 113, 27))
        self.lineEdit_qrtz_pi_q.setText("")
        self.lineEdit_qrtz_pi_q.setReadOnly(True)
        self.lineEdit_qrtz_pi_q.setObjectName("lineEdit_qrtz_pi_q")
        self.lineEdit_c_qrtz_warning_box = QtWidgets.QLineEdit(self.tab_qrtz)
        self.lineEdit_c_qrtz_warning_box.setGeometry(QtCore.QRect(15, 640, 330, 70))
        self.lineEdit_c_qrtz_warning_box.setReadOnly(True)
        self.lineEdit_c_qrtz_warning_box.setObjectName("lineEdit_c_qrtz_warning_box")
        self.groupBox_qrtz_lambda_p = QtWidgets.QGroupBox(self.tab_qrtz)
        self.groupBox_qrtz_lambda_p.setGeometry(QtCore.QRect(455, 640, 231, 70))
        self.groupBox_qrtz_lambda_p.setObjectName("groupBox_qrtz_lambda_p")
        self.lineEdit_qrtz_lamda_p = QtWidgets.QLineEdit(self.groupBox_qrtz_lambda_p)
        self.lineEdit_qrtz_lamda_p.setGeometry(QtCore.QRect(0, 30, 200, 27))
        self.lineEdit_qrtz_lamda_p.setReadOnly(True)
        self.lineEdit_qrtz_lamda_p.setObjectName("lineEdit_qrtz_lamda_p")
        self.addButton_c_ee_qrtz = QtWidgets.QPushButton(self.tab_qrtz)
        self.addButton_c_ee_qrtz.setGeometry(QtCore.QRect(930, 650, 125, 50))
        self.addButton_c_ee_qrtz.setObjectName("addButton_c_ee_qrtz")
        self.tabWidget_electrical_equipments.addTab(self.tab_qrtz, "")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1100, 25))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

    # ---------------------------------------------------------------------------------------------------------------

        self.gui_main()

    # ---------------------------------------------------------------------------------------------------------------

        self.retranslateUi(MainWindow)
        self.tabWidget_electrical_equipments.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        """
            Retranslate Ui Function
        """
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Electrical Equipments"))
        self.groupBox_cc_lambda_p.setTitle(_translate("MainWindow", "Capacitor failure rate - λp"))
        self.label_cc_quality.setText(_translate("MainWindow", "Quality:"))
        self.label_cc_capacitance_value.setText(_translate("MainWindow", "Capacitance value:"))
        self.label_cc_t.setText(_translate("MainWindow", "Capacitor Temperature - T:"))
        self.label_cc_circuit_resistance.setText(_translate("MainWindow", "Circuit resistance:"))
        self.label_cc_environment.setText(_translate("MainWindow", "Environment:"))
        self.label_cc_voltage_stress.setText(_translate("MainWindow", "Voltage stress:"))
        self.label_cc_capacitor_style.setText(_translate("MainWindow", "capacitor style:"))
        self.label_cc_pi_sr.setText(_translate("MainWindow", "Series resistance  factor - πSR :"))
        self.label_cc_pi_c.setText(_translate("MainWindow", "Capacitance factor - πC :"))
        self.label_cc_pi_e.setText(_translate("MainWindow", "Environment  factor - πE :"))
        self.label_cc_pi_t.setText(_translate("MainWindow", "Temperature factor - πT :"))
        self.label_cc_lambda_b.setText(_translate("MainWindow", "Base failure rate - λb :"))
        self.label_cc_pi_q.setText(_translate("MainWindow", "Quality  factor - πQ :"))
        self.label_cc_pi_v.setText(_translate("MainWindow", "Voltage stress factor - πV :"))
        self.addButton_c_ee_capacitor.setText(_translate("MainWindow", "Add Equipment"))
        self.tabWidget_electrical_equipments.setTabText(self.tabWidget_electrical_equipments.indexOf(self.tab_capacitor), _translate("MainWindow", "Capacitor"))
        self.groupBox_dd_lambda_p.setTitle(_translate("MainWindow", "Diode failure rate - λp"))
        self.label_dd_voltage_stress.setText(_translate("MainWindow", "Voltage stress:"))
        self.label_dd_diode_type.setText(_translate("MainWindow", "Diode Type / Application:"))
        self.label_dd_contact_construction.setText(_translate("MainWindow", "Contact construction:"))
        self.label_dd_quality.setText(_translate("MainWindow", "Quality:"))
        self.label_dd_junction_temperature.setText(_translate("MainWindow", "Junction temperature:"))
        self.label_dd_environment.setText(_translate("MainWindow", "Environment"))
        self.label_dd_pi_s.setText(_translate("MainWindow", "Electrical stress factor - πS :"))
        self.label_dd_pi_q.setText(_translate("MainWindow", "Quality factor - πQ :"))
        self.label_dd_pi_e.setText(_translate("MainWindow", "Environment factor - πE :"))
        self.label_dd_pi_c.setText(_translate("MainWindow", "Contuct construction factor - πC :"))
        self.label_dd_lambda_b.setText(_translate("MainWindow", "Base failure rate - λb :"))
        self.label_dd_pi_t.setText(_translate("MainWindow", "Temperature factor - πT :"))
        self.addButton_c_ee_diode.setText(_translate("MainWindow", "Add Equipment"))
        self.tabWidget_electrical_equipments.setTabText(self.tabWidget_electrical_equipments.indexOf(self.tab_diode), _translate("MainWindow", "Diode"))
        self.groupBox_id_lambda_p.setTitle(_translate("MainWindow", "Inductor failure rate - λp"))
        self.label_id_environment.setText(_translate("MainWindow", "Environment"))
        self.label_id_inductor_type.setText(_translate("MainWindow", "Inductor type:"))
        self.label_id_quality.setText(_translate("MainWindow", "Quality:"))
        self.label_id_hot_spot_temperature.setText(_translate("MainWindow", "Hot spot temperature:"))
        self.label_id_pi_q.setText(_translate("MainWindow", "Quality factor - πQ :"))
        self.label_id_lambda_b.setText(_translate("MainWindow", "Base failure rate - λb :"))
        self.label_id_pi_t.setText(_translate("MainWindow", "Temperature factor - πT :"))
        self.label_id_pi_e.setText(_translate("MainWindow", "Environment factor - πE :"))
        self.addButton_c_ee_inductor.setText(_translate("MainWindow", "Add Equipment"))
        self.tabWidget_electrical_equipments.setTabText(self.tabWidget_electrical_equipments.indexOf(self.tab_inductor), _translate("MainWindow", "Inductor"))
        self.groupBox_ts_lambda_p.setTitle(_translate("MainWindow", "Transistor failure rate - λp"))
        self.label_ts_transistor_type.setText(_translate("MainWindow", "Transistor type:"))
        self.label_ts_quality.setText(_translate("MainWindow", "Quality:"))
        self.label_ts_environment.setText(_translate("MainWindow", "Environment"))
        self.label_ts_junction_temperature.setText(_translate("MainWindow", "Junction temperature:"))
        self.label_ts_pi_q.setText(_translate("MainWindow", "Quality factor - πQ :"))
        self.label_ts_pi_t.setText(_translate("MainWindow", "Temperature factor - πT :"))
        self.label_ts_lambda_b.setText(_translate("MainWindow", "Base failure rate - λb :"))
        self.label_ts_pi_e.setText(_translate("MainWindow", "Environment factor - πE :"))
        self.addButton_c_ee_transistor.setText(_translate("MainWindow", "Add Equipment"))
        self.tabWidget_electrical_equipments.setTabText(self.tabWidget_electrical_equipments.indexOf(self.tab_transistor), _translate("MainWindow", "Transistor"))
        self.groupBox_fs_lambda_p.setTitle(_translate("MainWindow", "Fuse failure rate - λp"))
        self.label_fs_environment.setText(_translate("MainWindow", "Environment"))
        self.label_fs_fuse_type.setText(_translate("MainWindow", "Fuse type:"))
        self.label_fs_pi_e.setText(_translate("MainWindow", "Environment factor - πE :"))
        self.label_fs_lambda_b.setText(_translate("MainWindow", "Base failure rate - λb :"))
        self.addButton_c_ee_fuse.setText(_translate("MainWindow", "Add Equipment"))
        self.tabWidget_electrical_equipments.setTabText(self.tabWidget_electrical_equipments.indexOf(self.tab_fuse), _translate("MainWindow", "Fuse"))
        self.label_res_style.setText(_translate("MainWindow", "Resistor style:"))
        self.label_res_t.setText(_translate("MainWindow", "Resistor Case Temperature - T:"))
        self.label_res_w.setText(_translate("MainWindow", "Power Dissipiation - W:"))
        self.label_res_environment.setText(_translate("MainWindow", "Environment :"))
        self.label_res_quality.setText(_translate("MainWindow", "Quality :"))
        self.label_30.setText(_translate("MainWindow", "or"))
        self.label_40.setText(_translate("MainWindow", "or"))
        self.label_res_stress.setText(_translate("MainWindow", "Stress : "))
        self.label_41.setText(_translate("MainWindow", "or"))
        self.label_res_lambda_b.setText(_translate("MainWindow", "Base failure rate - λb :"))
        self.label_res_pi_t.setText(_translate("MainWindow", "Temperature factor - πT :"))
        self.label_res_pi_p.setText(_translate("MainWindow", "Power factor - πP :"))
        self.label_res_pi_e.setText(_translate("MainWindow", "Environment factor - πE :"))
        self.label_res_pi_q.setText(_translate("MainWindow", "Quality factor - πQ :"))
        self.label_res_pi_s.setText(_translate("MainWindow", "Power stress factor - πS :"))
        self.groupBox_res_lambda_p.setTitle(_translate("MainWindow", "Resistor failure rate - λp"))
        self.addButton_c_ee_res.setText(_translate("MainWindow", "Add Equipment"))
        self.tabWidget_electrical_equipments.setTabText(self.tabWidget_electrical_equipments.indexOf(self.tab_resistor), _translate("MainWindow", "Resistor"))
        self.groupBox_bearings.setTitle(_translate("MainWindow", "Bearing Failures"))
        self.label_rd_bearing_ambient_temperature.setText(_translate("MainWindow", "Ambient temperature - TA :"))
        self.label_rd_bearing_characteristic_life.setText(_translate("MainWindow", "Weibull characteristic life αB (hours) :"))
        self.label_rd_bearing_motor_type.setText(_translate("MainWindow", "Motor type :"))
        self.label_rd_bearing_a_determination.setText(_translate("MainWindow", "A Determination :"))
        self.label_rd_bearing_lambda1_determination.setText(_translate("MainWindow", "λ1 Determination :"))
        self.label_rd_bearing_LC.setText(_translate("MainWindow", "Life Cycle / αB : "))
        self.groupBox_rd_bearing_fr.setTitle(_translate("MainWindow", "Bearing failure rate"))
        self.label_9.setText(_translate("MainWindow", "or"))
        self.groupBox_winding.setTitle(_translate("MainWindow", "Winding Failures"))
        self.label_rd_winding_ambient_temperature.setText(_translate("MainWindow", "Ambient temperature - TA :"))
        self.label_rd_winding_characteristic_life.setText(_translate("MainWindow", "Weibull characteristic life αW (hours) :"))
        self.label_rd_winding_motor_type.setText(_translate("MainWindow", "Motor type :"))
        self.label_rd_winding_b_determination.setText(_translate("MainWindow", "B Determination :"))
        self.label_rd_winding_lambda2_determination.setText(_translate("MainWindow", "λ2 Determination :"))
        self.label_rd_winding_LC.setText(_translate("MainWindow", "Life Cycle / αW :"))
        self.groupBox_rd_winding_fr.setTitle(_translate("MainWindow", "Winding failure rate"))
        self.label_14.setText(_translate("MainWindow", "or"))
        self.addButton_c_ee_rd.setText(_translate("MainWindow", "Add Equipment"))
        self.groupBox_rd_lambda_p.setTitle(_translate("MainWindow", "Rotating Device failure rate - λp"))
        self.tabWidget_electrical_equipments.setTabText(self.tabWidget_electrical_equipments.indexOf(self.tab_rd), _translate("MainWindow", "Rotating Devices, Motors"))
        self.label_rel_ambient_temperature.setText(_translate("MainWindow", "Ambient temperature - TA :"))
        self.label_rel_stress.setText(_translate("MainWindow", "Stress : "))
        self.label_rel_environment.setText(_translate("MainWindow", "Environment :"))
        self.label_rel_quality.setText(_translate("MainWindow", "Quality :"))
        self.label_rel_rated_temperature.setText(_translate("MainWindow", "Rated  temperature :"))
        self.label_rel_load_type.setText(_translate("MainWindow", "Load type : "))
        self.label_rel_contact_form.setText(_translate("MainWindow", "Contact form : "))
        self.label_rel_cycle_rate.setText(_translate("MainWindow", "Cycle rate : "))
        self.label_rel_contact_rating.setText(_translate("MainWindow", "Contact rating : "))
        self.label_rel_application_type.setText(_translate("MainWindow", "Aplication type : "))
        self.label_rel_construction_type.setText(_translate("MainWindow", "Construction type :"))
        self.label_28.setText(_translate("MainWindow", "or"))
        self.label_29.setText(_translate("MainWindow", "or"))
        self.label_rel_cycle_rate_2.setText(_translate("MainWindow", "Enter cycle rate : "))
        self.label_rel_lambda_b.setText(_translate("MainWindow", "Base failure rate - λb :"))
        self.label_rel_pi_e.setText(_translate("MainWindow", "Environment factor - πE :"))
        self.label_rel_pi_q.setText(_translate("MainWindow", "Quality factor - πQ :"))
        self.label_rel_pi_l.setText(_translate("MainWindow", "Load stress factor - πL :"))
        self.label_rel_pi_c.setText(_translate("MainWindow", "Contact form factor - πC :"))
        self.label_rel_pi_cyc.setText(_translate("MainWindow", "Cycling factor - πCYC :"))
        self.label_rel_pi_f.setText(_translate("MainWindow", "πF :"))
        self.groupBox_rel_lambda_p.setTitle(_translate("MainWindow", "Relay failure rate - λp"))
        self.addButton_c_ee_rel.setText(_translate("MainWindow", "Add Equipment"))
        self.tabWidget_electrical_equipments.setTabText(self.tabWidget_electrical_equipments.indexOf(self.tab_rel), _translate("MainWindow", "Relays"))
        self.label_con_gen_description.setText(_translate("MainWindow", "Description : "))
        self.label_c.setText(_translate("MainWindow", "Environment :"))
        self.label_con_gen_quality.setText(_translate("MainWindow", "Quality :"))
        self.label_con_gen_t0.setText(_translate("MainWindow", "T0 : "))
        self.label_con_gen_cycles.setText(_translate("MainWindow", "Mating / Unmating cycles : "))
        self.label_17.setText(_translate("MainWindow", "or"))
        self.label_con_gen_lambda_b.setText(_translate("MainWindow", "Base failure rate - λb :"))
        self.label_con_gen_pi_e.setText(_translate("MainWindow", "Environment factor - πE :"))
        self.label_con_gen_pi_q.setText(_translate("MainWindow", "Quality factor - πQ :"))
        self.label_con_gen_pi_t.setText(_translate("MainWindow", "Temperature factor - πT :"))
        self.label_con_gen_pi_k.setText(_translate("MainWindow", "Mating / Unmating factor - πK :"))
        self.groupBox_con_gen_lambda_p.setTitle(_translate("MainWindow", "Connector (general) failure rate - λp"))
        self.addButton_c_ee_con_gen.setText(_translate("MainWindow", "Add Equipment"))
        self.tabWidget_electrical_equipments.setTabText(self.tabWidget_electrical_equipments.indexOf(self.tab_con_gen), _translate("MainWindow", "Connectors, General"))
        self.label_con_sock_lambda_b.setText(_translate("MainWindow", "Base failure rate - λb :"))
        self.label_con_sock_pi_e.setText(_translate("MainWindow", "Environment factor - πE :"))
        self.label_con_sock_pi_q.setText(_translate("MainWindow", "Quality factor - πQ :"))
        self.label_con_sock_pi_p.setText(_translate("MainWindow", "Active pins factor - πP :"))
        self.label_con_sock_description.setText(_translate("MainWindow", "Description : "))
        self.label_con_sock_environment.setText(_translate("MainWindow", "Environment :"))
        self.label_con_sock_quality.setText(_translate("MainWindow", "Quality :"))
        self.label_con_sock_numb_of_active_coils.setText(_translate("MainWindow", "No. of active contacts : "))
        self.label_16.setText(_translate("MainWindow", "or"))
        self.groupBox_con_sock_lambda_p.setTitle(_translate("MainWindow", "Socket failure rate - λp"))
        self.addButton_c_ee_con_sock.setText(_translate("MainWindow", "Add Equipment"))
        self.tabWidget_electrical_equipments.setTabText(self.tabWidget_electrical_equipments.indexOf(self.tab_con_sock), _translate("MainWindow", "Connectors, Sockets"))
        self.label_qrtz_frequency.setText(_translate("MainWindow", "Frequency, f (MHz) : "))
        self.label_qrtz_environment.setText(_translate("MainWindow", "Environment :"))
        self.label_qrtz_quality.setText(_translate("MainWindow", "Quality :"))
        self.label_15.setText(_translate("MainWindow", "or"))
        self.label_qrtz_lambda_b.setText(_translate("MainWindow", "Base failure rate - λb :"))
        self.label_qrtz_pi_e.setText(_translate("MainWindow", "Environment factor - πE :"))
        self.label_qrtz_pi_q.setText(_translate("MainWindow", "Quality factor - πQ :"))
        self.groupBox_qrtz_lambda_p.setTitle(_translate("MainWindow", "Quartz Crystal failure rate - λp"))
        self.addButton_c_ee_qrtz.setText(_translate("MainWindow", "Add Equipment"))
        self.tabWidget_electrical_equipments.setTabText(self.tabWidget_electrical_equipments.indexOf(self.tab_qrtz), _translate("MainWindow", "Quartz crystals"))

# ---------------------------------------------------------------------

    def gui_main(self):
        """
            Electrical Equipments Gui Main Functions
        """
        # CONFIGURATION // ELECTRICAL EQUIPMENTS
        self.resistor_tab_func()
        self.rotating_devices_tab_func()
        self.relays_tab_func()
        self.connectors_general_tab_func()
        self.connectors_sockets_tab_func()
        self.quartz_crystals_tab_func()
        self.capacitor_style_data()
        self.capacitor_tab_func()
        self.diode_type_data()
        self.diode_tab_func()
        self.inductor_tab_func()
        self.transistor_tab_func()
        self.fuse_tab_func()
        self.electrical_equipments_general_events_func()

        self.electrical_equipments_button_click_event_func()


    def electrical_equipments_button_click_event_func(self):
        """
            Electrical Equipments Gui Button Click Events Functions
        """
        self.addButton_c_ee_capacitor.clicked.connect(self.clicked_button_c_ee_capacitor_func)
        self.addButton_c_ee_diode.clicked.connect(self.clicked_button_c_ee_diode_func)
        self.addButton_c_ee_inductor.clicked.connect(self.clicked_button_c_ee_inductor_func)
        self.addButton_c_ee_transistor.clicked.connect(self.clicked_button_c_ee_transistor_func)
        self.addButton_c_ee_fuse.clicked.connect(self.clicked_button_c_ee_fuse_func)
        self.addButton_c_ee_res.clicked.connect(self.clicked_button_c_ee_res_func)
        self.addButton_c_ee_rd.clicked.connect(self.clicked_button_c_ee_rd_func)
        self.addButton_c_ee_rel.clicked.connect(self.clicked_button_c_ee_rel_func)
        self.addButton_c_ee_con_gen.clicked.connect(self.clicked_button_c_ee_con_gen_func)
        self.addButton_c_ee_con_sock.clicked.connect(self.clicked_button_c_ee_con_sock_func)
        self.addButton_c_ee_qrtz.clicked.connect(self.clicked_button_c_ee_qrtz_func)


    def clicked_main_func(self, set_data):
        """
            Set electrical equipments value to PHM Gui functions
        """
        create_object = eval(str("self.ui_class." + str(self.data_path)))
        create_object.setText(str(set_data))

        if self.equipment_type:
            self.ui_class.add_equipment_sub_module_window.close()

        else:
            self.ui_class.add_equipment_component_window.close()


    def clicked_button_c_ee_capacitor_func(self):
        """
            Set capacitor value function
        """
        self.clicked_main_func(self.lineEdit_cc_lamda_p.text())


    def clicked_button_c_ee_diode_func(self):
        """
            Set diode value function
        """
        self.clicked_main_func(self.lineEdit_dd_lamda_p.text())


    def clicked_button_c_ee_inductor_func(self):
        """
            Set inductor value function
        """
        self.clicked_main_func(self.lineEdit_id_lamda_p.text())


    def clicked_button_c_ee_transistor_func(self):
        """
            Set transistor value function
        """
        self.clicked_main_func(self.lineEdit_ts_lamda_p.text())


    def clicked_button_c_ee_fuse_func(self):
        """
            Set fuse value function
        """
        self.clicked_main_func(self.lineEdit_fs_lamda_p.text())


    def clicked_button_c_ee_res_func(self):
        """
            Set resistor value function
        """
        self.clicked_main_func(self.lineEdit_res_lamda_p.text())


    def clicked_button_c_ee_rd_func(self):
        """
            Set rotating devices value function
        """
        self.clicked_main_func(self.lineEdit_rd_lamda_p.text())


    def clicked_button_c_ee_rel_func(self):
        """
            Set relays value function
        """
        self.clicked_main_func(self.lineEdit_rel_lamda_p.text())


    def clicked_button_c_ee_con_gen_func(self):
        """
            Set connector, general value function
        """
        self.clicked_main_func(self.lineEdit_con_gen_lamda_p.text())


    def clicked_button_c_ee_con_sock_func(self):
        """
            Set connector, sockets value function
        """
        self.clicked_main_func(self.lineEdit_con_sock_lamda_p.text())


    def clicked_button_c_ee_qrtz_func(self):
        """
            Set quartz crystals value function
        """
        self.clicked_main_func(self.lineEdit_qrtz_lamda_p.text())


# "CONTROL FUNCTIONS OF INDEX CHANGES" - START -
  # CONFIGURATION // ELECTRICAL EQUIPMENTS
    def electrical_equipments_general_events_func(self):
        """
            Electrical Equipments General Events Func
        """
     # RESISTOR TAB CONFIGURATION
        self.comboBox_res_style.currentIndexChanged.connect(self.res_hazard_rate_func)
        self.comboBox_res_w.currentIndexChanged.connect(self.res_hazard_rate_func)
        self.lineEdit_res_w.textChanged.connect(self.res_hazard_rate_func)
        self.comboBox_res_environment.currentIndexChanged.connect(self.res_hazard_rate_func)
        self.comboBox_res_quality.currentIndexChanged.connect(self.res_hazard_rate_func)
        self.comboBox_res_case_temp.currentIndexChanged.connect(self.res_hazard_rate_func)
        self.lineEdit_res_case_temp.textChanged.connect(self.res_hazard_rate_func)
        self.comboBox_res_stress.currentIndexChanged.connect(self.res_hazard_rate_func)
        self.lineEdit_res_stress.textChanged.connect(self.res_hazard_rate_func)


     # ROTATING DEVICES, MOTORS TAB CONFIGURATION
        self.comboBox_rd_bearing_ambient_temperature.currentIndexChanged.connect(self.rd_hazard_rate_func)
        self.lineEdit_rd_bearing_ambient_temperature.textChanged.connect(self.rd_hazard_rate_func)
        self.comboBox_rd_bearing_motor_type.currentIndexChanged.connect(self.rd_hazard_rate_func)
        self.comboBox_rd_bearing_LC.currentIndexChanged.connect(self.rd_hazard_rate_func)
        self.comboBox_rd_winding_ambient_temperature.currentIndexChanged.connect(self.rd_hazard_rate_func)
        self.lineEdit_rd_winding_ambient_temperature.textChanged.connect(self.rd_hazard_rate_func)
        self.comboBox_rd_winding_motor_type.currentIndexChanged.connect(self.rd_hazard_rate_func)
        self.comboBox_rd_winding_LC.currentIndexChanged.connect(self.rd_hazard_rate_func)


     # RELAYS TAB CONFIGURATION
        self.comboBox_rel_ambient_temperature.currentIndexChanged.connect(self.relays_tab_func_2)
        self.comboBox_rel_stress.currentIndexChanged.connect(self.relays_tab_func_2)
        self.comboBox_rel_contact_rating.currentIndexChanged.connect(self.relays_tab_func_2)
        self.comboBox_rel_application_type.currentIndexChanged.connect(self.relays_tab_func_3)
        self.comboBox_rel_ambient_temperature.currentIndexChanged.connect(self.rel_hazard_rate_func)
        self.lineEdit_rel_ambient_temperature.textChanged.connect(self.rel_hazard_rate_func)
        self.comboBox_rel_rated_temperature.currentIndexChanged.connect(self.rel_hazard_rate_func)
        self.comboBox_rel_stress.currentIndexChanged.connect(self.rel_hazard_rate_func)
        self.lineEdit_rel_stress.textChanged.connect(self.rel_hazard_rate_func)
        self.comboBox_rel_load_type.currentIndexChanged.connect(self.rel_hazard_rate_func)
        self.comboBox_rel_contact_form.currentIndexChanged.connect(self.rel_hazard_rate_func)
        self.comboBox_rel_quality.currentIndexChanged.connect(self.rel_hazard_rate_func)
        self.comboBox_rel_environment.currentIndexChanged.connect(self.rel_hazard_rate_func)
        self.comboBox_rel_contact_rating.currentIndexChanged.connect(self.rel_hazard_rate_func)
        self.comboBox_rel_application_type.currentIndexChanged.connect(self.rel_hazard_rate_func)
        self.comboBox_rel_construction_type.currentIndexChanged.connect(self.rel_hazard_rate_func)
        self.comboBox_rel_cycle_rate.currentIndexChanged.connect(self.rel_hazard_rate_func)
        self.lineEdit_rel_cycle_rate.textChanged.connect(self.rel_hazard_rate_func)


     # CONNECTORS, GENERAL TAB CONFIGURATION
        self.comboBox_con_gen_description.currentIndexChanged.connect(self.con_gen_hazard_rate_func)
        self.comboBox_con_gen_t0.currentIndexChanged.connect(self.con_gen_hazard_rate_func)
        self.lineEdit_con_gen_t0.textChanged.connect(self.con_gen_hazard_rate_func)
        self.comboBox_con_gen_cycles.currentIndexChanged.connect(self.con_gen_hazard_rate_func)
        self.comboBox_con_gen_quality.currentIndexChanged.connect(self.con_gen_hazard_rate_func)
        self.comboBox_con_gen_environment.currentIndexChanged.connect(self.con_gen_hazard_rate_func)


     # CONNECTORS, SOCKETS TAB CONFIGURATION
        self.comboBox_con_sock_quality.currentIndexChanged.connect(self.con_sock_hazard_rate_func)
        self.comboBox_con_sock_environment.currentIndexChanged.connect(self.con_sock_hazard_rate_func)
        self.comboBox_con_sock_description.currentIndexChanged.connect(self.con_sock_hazard_rate_func)
        self.comboBox_con_sock_numb_of_active_contacts.currentIndexChanged.connect(self.con_sock_hazard_rate_func)
        self.lineEdit_con_sock_numb_of_active_contacts.textChanged.connect(self.con_sock_hazard_rate_func)


     # QUARTZ CRYSTALS TAB CONFIGURATION
        self.comboBox_qrtz_frequency.currentIndexChanged.connect(self.quartz_hazard_rate_func)
        self.lineEdit_qrtz_frequency.textChanged.connect(self.quartz_hazard_rate_func)
        self.comboBox_qrtz_quality.currentIndexChanged.connect(self.quartz_hazard_rate_func)
        self.comboBox_qrtz_environment.currentIndexChanged.connect(self.quartz_hazard_rate_func)


     # CAPACITOR TAB CONFIGURATION
        self.comboBox_cc_style.currentIndexChanged.connect(self.capacitor_tab_func)
        self.comboBox_cc_capacitor_temperature.currentIndexChanged.connect(self.capacitor_hazard_rate_func)
        self.comboBox_cc_capacitance.currentIndexChanged.connect(self.capacitor_hazard_rate_func)
        self.comboBox_cc_voltage_stress.currentIndexChanged.connect(self.capacitor_hazard_rate_func)
        self.comboBox_cc_circuit_resistance.currentIndexChanged.connect(self.capacitor_hazard_rate_func)
        self.comboBox_cc_quality.currentIndexChanged.connect(self.capacitor_hazard_rate_func)
        self.comboBox_cc_environment.currentIndexChanged.connect(self.capacitor_hazard_rate_func)


     # DIODE TAB CONFIGURATION
        self.comboBox_dd_type.currentIndexChanged.connect(self.diode_tab_func)
        self.comboBox_dd_junction_temperature.currentIndexChanged.connect(self.diode_hazard_rate_func)
        self.comboBox_dd_voltage_stress.currentIndexChanged.connect(self.diode_hazard_rate_func)
        self.comboBox_dd_contact_construction.currentIndexChanged.connect(self.diode_hazard_rate_func)
        self.comboBox_dd_quality.currentIndexChanged.connect(self.diode_hazard_rate_func)
        self.comboBox_dd_environment.currentIndexChanged.connect(self.diode_hazard_rate_func)


     # INDUCTOR INDUCTOR CONFIGURATION
        self.comboBox_id_type.currentIndexChanged.connect(self.inductor_hazard_rate_func)
        self.comboBox_id_hot_spot_temperature.currentIndexChanged.connect(self.inductor_hazard_rate_func)
        self.comboBox_id_quality.currentIndexChanged.connect(self.inductor_hazard_rate_func)
        self.comboBox_id_environment.currentIndexChanged.connect(self.inductor_hazard_rate_func)


     # TRANSISTOR TAB CONFIGURATION
        self.comboBox_ts_type.currentIndexChanged.connect(self.transistor_hazard_rate_func)
        self.comboBox_ts_quality.currentIndexChanged.connect(self.transistor_hazard_rate_func)
        self.comboBox_ts_junction_temperature.currentIndexChanged.connect(self.transistor_hazard_rate_func)
        self.comboBox_ts_environment.currentIndexChanged.connect(self.transistor_hazard_rate_func)


     # FUSE TAB CONFIGURATION
        self.comboBox_fs_type.currentIndexChanged.connect(self.fuse_hazard_rate_func)
        self.comboBox_fs_environment.currentIndexChanged.connect(self.fuse_hazard_rate_func)


# "CONTROL FUNCTIONS OF INDEX CHANGES" - END -

# "CONFIGURATION // ELECTRICAL EQUIPMENTS FUNCTIONS" - START -
  # RESISTORS
    def resistor_tab_func(self):
        """
            Resistors Tab Function
        """
        res_dict = dict(self.electrical_equipments_dict["Resistor"])

        res_style_list = list(res_dict["Resistor Style"].keys())
        power_factor_list = list(res_dict["Power Factor"].keys())
        environment_list = list(res_dict["Environment"].keys())
        quality_list = list(res_dict["Quality"].keys())
        temperature_list = list(res_dict["Temperature Factor"].keys())
        stress_list = list(res_dict["Power Stress Factor"].keys())

        temp_list_1 = list()
        temp_list_2 = list()
        temp_list_3 = list()

        for item in power_factor_list:
            temp_str = item.split("w_")
            new_val = float(temp_str[1].replace("_", "."))
            temp_list_1.append(new_val)

        for item in temperature_list:
            new_val = item.split("t_")
            temp_list_2.append(int(new_val[1]))

        for item in stress_list:
            temp_str = item.split("s_")
            new_val = float(temp_str[1].replace("_", "."))
            temp_list_3.append(new_val)

        temp_list_1.sort()
        temp_list_2.sort()
        temp_list_3.sort()

        temp_list_1 = map(str, temp_list_1)
        temp_list_2 = map(str, temp_list_2)
        temp_list_3 = map(str, temp_list_3)

        self.comboBox_res_style.clear()
        self.comboBox_res_w.clear()
        self.comboBox_res_environment.clear()
        self.comboBox_res_quality.clear()
        self.comboBox_res_case_temp.clear()
        self.comboBox_res_stress.clear()

        self.comboBox_res_style.addItem("None")
        self.comboBox_res_w.addItem("None")
        self.comboBox_res_environment.addItem("None")
        self.comboBox_res_quality.addItem("None")
        self.comboBox_res_case_temp.addItem("None")
        self.comboBox_res_stress.addItem("None")

        self.comboBox_res_style.addItems(list(res_style_list))
        self.comboBox_res_w.addItems(list(temp_list_1))
        self.comboBox_res_environment.addItems(list(environment_list))
        self.comboBox_res_quality.addItems(list(quality_list))
        self.comboBox_res_case_temp.addItems(list(temp_list_2))
        self.comboBox_res_stress.addItems(list(temp_list_3))

        self.comboBox_res_w.addItem("Other value...")
        self.comboBox_res_case_temp.addItem("Other value...")
        self.comboBox_res_stress.addItem("Other value...")

        self.res_hazard_rate_func()


    def resistor_stress_func(self):
        """
            Resistors Stress Function
        """
        selected_resistor = self.comboBox_res_style.currentText()
        stress = self.comboBox_res_stress.currentText()

        if selected_resistor != "" and stress != "" and selected_resistor != "None" and stress != "None":
            if stress != "Other value...":
                self.res_stress_combobox_func(stress)
            else:
                self.res_stress_other_value_func()
        else:
            self.lineEdit_res_pi_s.clear()
            self.e_eq.res_pi_s = 1.0
            self.lineEdit_res_pi_s.setText(str(self.e_eq.get_res_pi_s()))


    def res_stress_combobox_func(self, stress):
        """
            Resistors Stress Combobox Function
        """
        res_dict = dict(self.electrical_equipments_dict["Resistor"])
        selected_resistor = self.comboBox_res_style.currentText()

        self.lineEdit_res_stress.clear()
        self.lineEdit_res_pi_s.clear()
        self.label_41.setEnabled(False)
        self.lineEdit_res_stress.setEnabled(False)

        psf_column = self.resistor_type_dict[str(selected_resistor)]["Power Stress Factor"]
        psf_column_str = "column_" + str(psf_column)

        stress_str = stress.replace(".", "_")
        stress_str = "s_" + str(stress_str)

        if psf_column == "val_1":
            res_pi_s = 1.0
        else:
            res_pi_s = res_dict["Power Stress Factor"][str(stress_str)][str(psf_column_str)]

        self.e_eq.res_pi_s = res_pi_s
        self.lineEdit_res_pi_s.setText(str(self.e_eq.get_res_pi_s()))


    def res_stress_other_value_func(self):
        """
            Resistors Stress Other Value Function
        """
        self.lineEdit_res_pi_s.clear()
        self.label_41.setEnabled(True)
        self.lineEdit_res_stress.setEnabled(True)

        selected_resistor = self.comboBox_res_style.currentText()
        stress = self.lineEdit_res_stress.text()

        psf_column = self.resistor_type_dict[str(selected_resistor)]["Power Stress Factor"]
        psf_column_str = "column_" + str(psf_column)

        if stress != "":
            if psf_column == "val_1":
                res_pi_s = 1.0
            else:
                self.e_eq.res_pi_s_func(float(stress), int(psf_column))
                self.lineEdit_res_pi_s.setText(str(self.e_eq.get_res_pi_s()))


    def resistor_temperature_func(self):
        """
            Resistors Temperature Function
        """
        selected_resistor = self.comboBox_res_style.currentText()
        temperature = self.comboBox_res_case_temp.currentText()

        if selected_resistor != "" and temperature != "" and selected_resistor != "None" and temperature != "None":
            if temperature != "Other value...":
                self.res_temp_combobox_func(temperature)
            else:
                self.res_temp_other_value_func()
        else:
            self.lineEdit_res_pi_t.clear()
            self.e_eq.res_pi_t = 1.0
            self.lineEdit_res_pi_t.setText(str(self.e_eq.get_res_pi_t()))


    def res_temp_combobox_func(self, temperature):
        """
            Resistors Temperature Combobox Function
        """
        res_dict = dict(self.electrical_equipments_dict["Resistor"])
        selected_resistor = self.comboBox_res_style.currentText()

        self.lineEdit_res_case_temp.clear()
        self.lineEdit_res_pi_t.clear()
        self.label_30.setEnabled(False)
        self.lineEdit_res_case_temp.setEnabled(False)

        tf_column = self.resistor_type_dict[str(selected_resistor)]["Temperature Factor"]
        tf_column_str = "column_" + str(tf_column)

        temperature_str = "t_" + str(temperature)

        if tf_column == "val_1":
            res_pi_t = 1.0
        else:
            res_pi_t = res_dict["Temperature Factor"][str(temperature_str)][str(tf_column_str)]

        self.e_eq.res_pi_t = res_pi_t
        self.lineEdit_res_pi_t.setText(str(self.e_eq.get_res_pi_t()))


    def res_temp_other_value_func(self):
        """
            Resistors Temperature Other Value Function
        """
        self.lineEdit_res_pi_t.clear()
        self.label_30.setEnabled(True)
        self.lineEdit_res_case_temp.setEnabled(True)

        selected_resistor = self.comboBox_res_style.currentText()
        temperature = self.lineEdit_res_case_temp.text()

        tf_column = self.resistor_type_dict[str(selected_resistor)]["Temperature Factor"]
        tf_column_str = "column_" + str(tf_column)

        if temperature != "":
            if tf_column == "val_1":
                res_pi_t = 1.0
            else:
                self.e_eq.res_pi_t_func(float(temperature), int(tf_column))
                self.lineEdit_res_pi_t.setText(str(self.e_eq.get_res_pi_t()))


    def resistor_quality_func(self):
        """
            Resistors Quality Function
        """
        res_dict = dict(self.electrical_equipments_dict["Resistor"])
        quality = self.comboBox_res_quality.currentText()

        if quality != "" and quality != "None":
            self.lineEdit_res_pi_q.clear()
            res_pi_q = res_dict["Quality"][str(quality)]

            self.e_eq.res_pi_q = res_pi_q
            self.lineEdit_res_pi_q.setText(str(self.e_eq.res_pi_q))
        else:
            self.lineEdit_res_pi_q.clear()
            self.e_eq.res_pi_q = 1.0
            self.lineEdit_res_pi_q.setText(str(self.e_eq.res_pi_q))


    def resistor_environment_func(self):
        """
            Resistors Environment Function
        """
        res_dict = dict(self.electrical_equipments_dict["Resistor"])
        environment = self.comboBox_res_environment.currentText()

        if environment != "" and environment != "None":
            self.lineEdit_res_pi_e.clear()
            res_pi_e = res_dict["Environment"][str(environment)]

            self.e_eq.res_pi_e = res_pi_e
            self.lineEdit_res_pi_e.setText(str(self.e_eq.res_pi_e))
        else:
            self.lineEdit_res_pi_e.clear()
            self.e_eq.res_pi_e = 1.0
            self.lineEdit_res_pi_e.setText(str(self.e_eq.res_pi_e))


    def resistor_style_func(self):
        """
            Resistors Style Function
        """
        res_dict = dict(self.electrical_equipments_dict["Resistor"])

        res_style = self.comboBox_res_style.currentText()

        if res_style != "" and res_style != "None":
            self.lineEdit_res_lambda_b.clear()
            res_lambda_b = res_dict["Resistor Style"][str(res_style)]
            self.e_eq.res_lambda_b = res_lambda_b

            self.lineEdit_res_lambda_b.setText(str(self.e_eq.res_lambda_b))
        else:
            self.lineEdit_res_lambda_b.clear()
            self.e_eq.res_lambda_b = 1.0
            self.lineEdit_res_lambda_b.setText(str(self.e_eq.res_lambda_b))


    def res_power_dissip_func(self):
        """
            Resistors Dissip Function
        """
        power_dissip = self.comboBox_res_w.currentText()

        if power_dissip != "" and power_dissip != "None":
            if power_dissip != "Other value...":
                self.power_dissip_combobox_control(power_dissip)
            else:
                self.power_dissip_other_value_control()
        else:
            self.lineEdit_res_pi_p.clear()
            self.e_eq.res_pi_p = 1.0
            self.lineEdit_res_pi_p.setText(str(self.e_eq.get_res_pi_p()))


    def power_dissip_combobox_control(self, power_dissip):
        """
            Resistors Dissip Combobox Function
        """
        res_dict = dict(self.electrical_equipments_dict["Resistor"])
        self.lineEdit_res_w.clear()
        self.lineEdit_res_pi_p.clear()

        self.label_40.setEnabled(False)
        self.lineEdit_res_w.setEnabled(False)

        new_str = power_dissip.replace(".", "_")
        new_str = "w_" + str(new_str)
        res_pi_p = res_dict["Power Factor"][str(new_str)]

        self.e_eq.res_pi_p = res_pi_p
        self.lineEdit_res_pi_p.setText(str(self.e_eq.get_res_pi_p()))


    def power_dissip_other_value_control(self):
        """
            Resistors Dissip Other Value Function
        """
        self.lineEdit_res_pi_p.clear()
        self.label_40.setEnabled(True)
        self.lineEdit_res_w.setEnabled(True)

        power_dissip = self.lineEdit_res_w.text()

        if power_dissip != "":
            self.e_eq.res_pi_p_func(float(power_dissip))
            self.lineEdit_res_pi_p.setText(str(self.e_eq.get_res_pi_p()))


    def res_hazard_rate_func(self):
        """
            Resistors Hazard Rate Function
        """
        self.resistor_style_func()
        self.res_power_dissip_func()
        self.resistor_environment_func()
        self.resistor_quality_func()
        self.resistor_temperature_func()
        self.resistor_stress_func()

        self.e_eq.resistors_func()
        self.lineEdit_res_lamda_p.setText(str(self.e_eq.get_res_lambda()))


  # ROTATING DEVICES, MOTORS
    def rotating_devices_tab_func(self):
        """
            Rotating Devices Tab Function
        """
        rotating_devices_dict = dict(self.electrical_equipments_dict["Rotating Devices"])

        rd_bearing_ambient_temperature_list = list(rotating_devices_dict["Bearing Characteristic Life"].keys())
        rd_winding_ambient_temperature_list = list(rotating_devices_dict["Winding Characteristic Life"].keys())
        rd_bearing_motor_type_list = list(rotating_devices_dict["A Determination"].keys())
        rd_winding_motor_type_list = list(rotating_devices_dict["B Determination"].keys())
        rd_bearing_lc_div_alpha_b_list = list(rotating_devices_dict["Lambda1 Determination"].keys())
        rd_winding_lc_div_alpha_w_list = list(rotating_devices_dict["Lambda2 Determination"].keys())

        temp_list_1 = list()
        temp_list_2 = list()
        temp_list_3 = list()
        temp_list_4 = list()

        for item in rd_bearing_ambient_temperature_list:
            temp_str = item.split("t_")
            new_val = int(temp_str[1].replace("_", "."))
            temp_list_1.append(new_val)

        for item in rd_winding_ambient_temperature_list:
            temp_str = item.split("t_")
            new_val = int(temp_str[1].replace("_", "."))
            temp_list_2.append(new_val)

        for item in rd_bearing_lc_div_alpha_b_list:
            if "btw_" in item:
                temp_str = item.replace("btw_", "Between ")
                temp_str = temp_str.replace("_and_", " and ")
                temp_str = temp_str.replace("_", ".")
            elif "g_" in item:
                temp_str = item.replace("g_", "Greater than ")
                temp_str = temp_str.replace("_", ".")

            temp_list_3.append(temp_str)

        for item in rd_winding_lc_div_alpha_w_list:
            if "btw_" in item:
                temp_str = item.replace("btw_", "Between ")
                temp_str = temp_str.replace("_and_", " and ")
                temp_str = temp_str.replace("_", ".")
            elif "g_" in item:
                temp_str = item.replace("g_", "Greater than ")
                temp_str = temp_str.replace("_", ".")

            temp_list_4.append(temp_str)

        temp_list_1.sort()
        temp_list_2.sort()
        temp_list_3.sort()
        temp_list_4.sort()

        temp_list_1 = map(str, temp_list_1)
        temp_list_2 = map(str, temp_list_2)
        temp_list_3 = map(str, temp_list_3)
        temp_list_4 = map(str, temp_list_4)

        self.comboBox_rd_bearing_ambient_temperature.clear()
        self.comboBox_rd_winding_ambient_temperature.clear()
        self.comboBox_rd_bearing_motor_type.clear()
        self.comboBox_rd_winding_motor_type.clear()
        self.comboBox_rd_bearing_LC.clear()
        self.comboBox_rd_winding_LC.clear()

        self.comboBox_rd_bearing_ambient_temperature.addItem("None")
        self.comboBox_rd_winding_ambient_temperature.addItem("None")
        self.comboBox_rd_bearing_motor_type.addItem("None")
        self.comboBox_rd_winding_motor_type.addItem("None")
        self.comboBox_rd_bearing_LC.addItem("None")
        self.comboBox_rd_winding_LC.addItem("None")

        self.comboBox_rd_bearing_ambient_temperature.addItems(temp_list_1)
        self.comboBox_rd_winding_ambient_temperature.addItems(temp_list_2)
        self.comboBox_rd_bearing_motor_type.addItems(rd_bearing_motor_type_list)
        self.comboBox_rd_winding_motor_type.addItems(rd_winding_motor_type_list)
        self.comboBox_rd_bearing_LC.addItems(temp_list_3)
        self.comboBox_rd_winding_LC.addItems(temp_list_4)

        self.comboBox_rd_bearing_ambient_temperature.addItem("Other value...")
        self.comboBox_rd_winding_ambient_temperature.addItem("Other value...")

        self.rd_hazard_rate_func()


    def rot_dev_alpha_b_func(self):
        """
            Rotating Devices Alpha_b Function
        """
        temperature = self.comboBox_rd_bearing_ambient_temperature.currentText()

        if temperature != "" and temperature != "None":
            if temperature != "Other value...":
                self.alpha_b_combobox_control(temperature)
            else:
                self.alpha_b_other_value_control()
        else:
            self.lineEdit_rd_bearing_characteristic_life.clear()
            self.e_eq.rd_alpha_b = 1.0
            self.lineEdit_rd_bearing_characteristic_life.setText(str(self.e_eq.get_rd_alpha_b()))


    def rot_dev_alpha_w_func(self):
        """
            Rotating Devices Alpha_w Function
        """
        temperature = self.comboBox_rd_winding_ambient_temperature.currentText()

        if temperature != "" and temperature != "None":
            if temperature != "Other value...":
                self.alpha_w_combobox_control(temperature)
            else:
                self.alpha_w_other_value_control()
        else:
            self.lineEdit_rd_winding_characteristic_life.clear()
            self.e_eq.rd_alpha_w = 1.0
            self.lineEdit_rd_winding_characteristic_life.setText(str(self.e_eq.get_rd_alpha_w()))


    def alpha_b_combobox_control(self, temperature):
        """
            Rotating Devices Alpha_b Combobox Function
        """
        rd_dict = self.electrical_equipments_dict["Rotating Devices"]

        self.lineEdit_rd_bearing_ambient_temperature.clear()
        self.lineEdit_rd_bearing_characteristic_life.clear()
        self.label_9.setEnabled(False)
        self.lineEdit_rd_bearing_ambient_temperature.setEnabled(False)

        new_str = "t_" + str(temperature)
        rd_alpha_b = rd_dict["Bearing Characteristic Life"][str(new_str)]

        self.e_eq.rd_alpha_b = rd_alpha_b
        self.lineEdit_rd_bearing_characteristic_life.setText(str(self.e_eq.get_rd_alpha_b()))


    def alpha_w_combobox_control(self, temperature):
        """
            Rotating Devices Alpha_w Combobox Function
        """
        rd_dict = self.electrical_equipments_dict["Rotating Devices"]

        self.lineEdit_rd_winding_ambient_temperature.clear()
        self.lineEdit_rd_winding_characteristic_life.clear()
        self.label_14.setEnabled(False)
        self.lineEdit_rd_winding_ambient_temperature.setEnabled(False)

        new_str = "t_" + str(temperature)
        rd_alpha_w = rd_dict["Winding Characteristic Life"][str(new_str)]

        self.e_eq.rd_alpha_w = rd_alpha_w
        self.lineEdit_rd_winding_characteristic_life.setText(str(self.e_eq.get_rd_alpha_w()))


    def alpha_b_other_value_control(self):
        """
            Rotating Devices Alpha_b Other Value Function
        """
        self.lineEdit_rd_bearing_characteristic_life.clear()

        self.label_9.setEnabled(True)
        self.lineEdit_rd_bearing_ambient_temperature.setEnabled(True)

        temperature = self.lineEdit_rd_bearing_ambient_temperature.text()

        if temperature != "":
            self.e_eq.rd_alpha_b_func(float(temperature))
            self.lineEdit_rd_bearing_characteristic_life.setText(str(self.e_eq.get_rd_alpha_b()))


    def alpha_w_other_value_control(self):
        """
            Rotating Devices Alpha_w Other Value Function
        """
        self.lineEdit_rd_winding_characteristic_life.clear()
        self.label_14.setEnabled(True)
        self.lineEdit_rd_winding_ambient_temperature.setEnabled(True)

        temperature = self.lineEdit_rd_winding_ambient_temperature.text()

        if temperature != "":
            self.e_eq.rd_alpha_w_func(float(temperature))
            self.lineEdit_rd_winding_characteristic_life.setText(str(self.e_eq.get_rd_alpha_w()))


    def rot_dev_a_determination_func(self):
        """
            Rotating Devices A Determination Function
        """
        motor_type = self.comboBox_rd_bearing_motor_type.currentText()
        rd_dict = self.electrical_equipments_dict["Rotating Devices"]

        if motor_type != "" and motor_type != "None":
            self.lineEdit_rd_bearing_a_determination.clear()
            a_det = rd_dict["A Determination"][str(motor_type)]
            self.e_eq.rd_determination_a = a_det
            self.lineEdit_rd_bearing_a_determination.setText(str(self.e_eq.rd_determination_a))

        else:
            self.lineEdit_rd_bearing_a_determination.clear()
            self.e_eq.rd_determination_a = 1.0
            self.lineEdit_rd_bearing_a_determination.setText(str(self.e_eq.rd_determination_a))


    def rot_dev_b_determination_func(self):
        """
            Rotating Devices B Determination Function
        """
        motor_type = self.comboBox_rd_winding_motor_type.currentText()

        rd_dict = self.electrical_equipments_dict["Rotating Devices"]

        if motor_type != "" and motor_type != "None":
            self.lineEdit_rd_winding_b_determination.clear()
            b_det = rd_dict["B Determination"][str(motor_type)]
            self.e_eq.rd_determination_b = b_det
            self.lineEdit_rd_winding_b_determination.setText(str(self.e_eq.rd_determination_b))
        else:
            self.lineEdit_rd_winding_b_determination.clear()
            self.e_eq.rd_determination_b = 1.0
            self.lineEdit_rd_winding_b_determination.setText(str(self.e_eq.rd_determination_b))


    def rot_dev_lambda_1_func(self):
        """
            Rotating Devices Lambda_1 Function
        """
        rd_dict = self.electrical_equipments_dict["Rotating Devices"]
        lc_dic_alpha_b = self.comboBox_rd_bearing_LC.currentText()

        if lc_dic_alpha_b != "" and lc_dic_alpha_b != "None":
            self.lineEdit_rd_bearing_lambda1_determination.clear()

            tmp_str1 = lc_dic_alpha_b.replace(".", "_")
            tmp_str2 = tmp_str1.replace(" and ", "_and_")
            tmp_str3 = tmp_str2.replace("Between ", "btw_")
            tmp_str4 = tmp_str3.replace("Greater than ", "g_")
            lambda_1 = rd_dict["Lambda1 Determination"][str(tmp_str4)]

            self.e_eq.rd_lambda_1 = lambda_1
            self.lineEdit_rd_bearing_lambda1_determination.setText(str(self.e_eq.rd_lambda_1))
        else:
            self.lineEdit_rd_bearing_lambda1_determination.clear()
            self.e_eq.rd_lambda_1 = 1.0
            self.lineEdit_rd_bearing_lambda1_determination.setText(str(self.e_eq.rd_lambda_1))


    def rot_dev_lambda_2_func(self):
        """
            Rotating Devices Lambda_2 Function
        """
        rd_dict = self.electrical_equipments_dict["Rotating Devices"]
        lc_dic_alpha_w = self.comboBox_rd_winding_LC.currentText()

        if lc_dic_alpha_w != "" and lc_dic_alpha_w != "None":
            self.lineEdit_rd_winding_lambda2_determination.clear()

            tmp_str1 = lc_dic_alpha_w.replace(".", "_")
            tmp_str2 = tmp_str1.replace(" and ", "_and_")
            tmp_str3 = tmp_str2.replace("Between ", "btw_")
            tmp_str4 = tmp_str3.replace("Greater than ", "g_")
            lambda_2 = rd_dict["Lambda2 Determination"][str(tmp_str4)]

            self.e_eq.rd_lambda_2 = lambda_2
            self.lineEdit_rd_winding_lambda2_determination.setText(str(self.e_eq.rd_lambda_2))
        else:
            self.lineEdit_rd_winding_lambda2_determination.clear()
            self.e_eq.rd_lambda_2 = 1.0
            self.lineEdit_rd_winding_lambda2_determination.setText(str(self.e_eq.rd_lambda_2))


    def rd_hazard_rate_func(self):
        """
            Rotating Devices Hazard Rate Function
        """
        self.rot_dev_alpha_b_func()
        self.rot_dev_a_determination_func()
        self.rot_dev_lambda_1_func()
        self.rot_dev_alpha_w_func()
        self.rot_dev_b_determination_func()
        self.rot_dev_lambda_2_func()
        self.e_eq.rotating_device_func_1()
        self.lineEdit_rd_bearing_fr.setText(str(self.e_eq.get_rd_lambda_p_1()))

        self.e_eq.rotating_device_func_2()
        self.lineEdit_rd_winding_fr.setText(str(self.e_eq.get_rd_lambda_p_2()))

        self.e_eq.rotating_device_func()
        self.lineEdit_rd_lamda_p.setText(str(self.e_eq.get_rd_lambda()))


  # RELAYS
    def relays_tab_func(self):
        """
            Relays Tab Function
        """
        rel_dict = dict(self.electrical_equipments_dict["Relays"])
        ambient_temperature_list = list(rel_dict["Ambient Temperature"].keys())
        stress_list = list(rel_dict["Stress"].keys())
        contact_form_list = list(rel_dict["Contact Form"].keys())
        cycle_list = list(rel_dict["Cycle Rate"].keys())
        quality_list = list(rel_dict["Quality"].keys())
        environment_list = list(rel_dict["Environment"].keys())
        contact_rating_list = list(rel_dict["pi_f"].keys())

        temp_list_1 = list()
        temp_list_2 = list()
        temp_list_3 = list()

        for item in ambient_temperature_list:
            temp_str = item.split("t_")
            new_val = int(temp_str[1])
            temp_list_1.append(new_val)

        for item in cycle_list:
            tmp_str1 = item.replace("ge_", ">= ")
            tmp_str2 = tmp_str1.replace("s_", "< ")
            tmp_str3 = tmp_str2.replace("g_", "> ")
            tmp_str4 = tmp_str3.replace("btw_", "Between ")
            tmp_str5 = tmp_str4.replace("_and_", " and ")
            tmp_str6 = tmp_str5.replace("_", ".")
            temp_list_2.append(tmp_str6)

        for item in stress_list:
            temp_str = item.split("s_")
            new_val = float(temp_str[1].replace("_", "."))
            temp_list_3.append(new_val)

        temp_list_1.sort()
        temp_list_2.sort()
        temp_list_3.sort()

        temp_list_1 = map(str, temp_list_1)
        temp_list_2 = map(str, temp_list_2)
        temp_list_3 = map(str, temp_list_3)

        self.comboBox_rel_ambient_temperature.clear()
        self.comboBox_rel_stress.clear()
        self.comboBox_rel_contact_form.clear()
        self.comboBox_rel_cycle_rate.clear()
        self.comboBox_rel_quality.clear()
        self.comboBox_rel_environment.clear()
        self.comboBox_rel_contact_rating.clear()

        self.comboBox_rel_ambient_temperature.addItem("None")
        self.comboBox_rel_stress.addItem("None")
        self.comboBox_rel_contact_form.addItem("None")
        self.comboBox_rel_cycle_rate.addItem("None")
        self.comboBox_rel_quality.addItem("None")
        self.comboBox_rel_environment.addItem("None")
        self.comboBox_rel_contact_rating.addItem("None")

        self.comboBox_rel_ambient_temperature.addItems(list(temp_list_1))
        self.comboBox_rel_stress.addItems(list(temp_list_3))
        self.comboBox_rel_contact_form.addItems(list(contact_form_list))
        self.comboBox_rel_cycle_rate.addItems(list(temp_list_2))
        self.comboBox_rel_quality.addItems(list(quality_list))
        self.comboBox_rel_environment.addItems(list(environment_list))
        self.comboBox_rel_contact_rating.addItems(list(contact_rating_list))

        self.comboBox_rel_ambient_temperature.addItem("Other value...")
        self.comboBox_rel_stress.addItem("Other value...")

        self.rel_hazard_rate_func()


    def relays_tab_func_2(self):
        """
            Relays Tab Function 2
        """
        rel_dict = dict(self.electrical_equipments_dict["Relays"])
        ambient_temperature = self.comboBox_rel_ambient_temperature.currentText()
        stress = self.comboBox_rel_stress.currentText()
        contact_rating = self.comboBox_rel_contact_rating.currentText()

        self.comboBox_rel_rated_temperature.clear()
        self.comboBox_rel_rated_temperature.addItem("None")

        self.comboBox_rel_load_type.clear()
        self.comboBox_rel_load_type.addItem("None")

        self.comboBox_rel_application_type.clear()
        self.comboBox_rel_application_type.addItem("None")

        if ambient_temperature != "" and ambient_temperature != "None":
            if ambient_temperature != "Other value...":
                self.lineEdit_rel_ambient_temperature.clear()
                self.label_28.setEnabled(False)
                self.lineEdit_rel_ambient_temperature.setEnabled(False)

                ambient_temperature = str("t_" + str(ambient_temperature))
                rated_temperature_list = list(rel_dict["Ambient Temperature"][str(ambient_temperature)].keys())
                temp_list_1 = list()

                for item in rated_temperature_list:
                    temp_str = item.split("r_")
                    new_val = int(temp_str[1])
                    temp_list_1.append(new_val)

                temp_list_1.sort()
                temp_list_1 = map(str, temp_list_1)
                self.comboBox_rel_rated_temperature.addItems(list(temp_list_1))
            else:
                self.label_28.setEnabled(True)
                self.lineEdit_rel_ambient_temperature.setEnabled(True)

                rated_temperature_list = list(["85", "125"])
                self.comboBox_rel_rated_temperature.addItems(list(rated_temperature_list))

        if stress != "" and stress != "None":
            if stress != "Other value...":
                self.lineEdit_rel_stress.clear()
                self.label_29.setEnabled(False)
                self.lineEdit_rel_stress.setEnabled(False)

                stress = stress.replace(".", "_")
                new_str = "s_" + str(stress)
                load_type_list = list(rel_dict["Stress"][str(new_str)].keys())
                self.comboBox_rel_load_type.addItems(list(load_type_list))
            else:
                self.label_29.setEnabled(True)
                self.lineEdit_rel_stress.setEnabled(True)

                load_type_list = list(["Resistive", "Inductive", "Lamp"])
                self.comboBox_rel_load_type.addItems(load_type_list)

        if contact_rating != "" and contact_rating != "None":
            application_type_list = list(rel_dict["pi_f"][str(contact_rating)].keys())
            self.comboBox_rel_application_type.addItems(list(application_type_list))


    def relays_tab_func_3(self):
        """
            Relays Tab Function 3
        """
        rel_dict = dict(self.electrical_equipments_dict["Relays"])
        contact_rating = self.comboBox_rel_contact_rating.currentText()
        application_type = self.comboBox_rel_application_type.currentText()

        self.comboBox_rel_construction_type.clear()
        self.comboBox_rel_construction_type.addItem("None")

        try:
            if ((contact_rating and application_type) != "") and ((contact_rating and application_type) != "None"):
                construction_type_list = list(rel_dict["pi_f"][str(contact_rating)][str(application_type)].keys())
                self.comboBox_rel_construction_type.addItems(list(construction_type_list))

        except AttributeError:
            self.comboBox_rel_construction_type.clear()
            self.comboBox_rel_construction_type.addItem("None")


    def rel_ambient_temperature_func(self):
        """
            Relays Ambient Temperature Function
        """
        ta_value = self.comboBox_rel_ambient_temperature.currentText()

        if ta_value != "" and ta_value != "None":
            if ta_value != "Other value...":
                self.ta_combobox_control(ta_value)
            else:
                self.ta_other_value_control()
        else:
            self.lineEdit_rel_lambda_b.clear()
            self.e_eq.rel_lambda_b = 1.0
            self.lineEdit_rel_lambda_b.setText(str(self.e_eq.get_rel_lambda_b()))


    def ta_combobox_control(self, ta_value):
        """
            Relays Ambient Temperature Combobox Function
        """
        rel_dict = dict(self.electrical_equipments_dict["Relays"])
        rated_temp = self.comboBox_rel_rated_temperature.currentText()

        self.lineEdit_rel_ambient_temperature.clear()
        self.lineEdit_rel_lambda_b.clear()
        self.label_28.setEnabled(False)
        self.lineEdit_rel_ambient_temperature.setEnabled(False)

        ta_value = ta_value.replace(".", "_")
        new_str_ta = "t_" + str(ta_value)

        if rated_temp != "" and rated_temp != "None":
            rated_temp.replace(".", "_")
            new_str_rated_temp = "r_" + str(rated_temp)
            rel_lambda_b = rel_dict["Ambient Temperature"][str(new_str_ta)][str(new_str_rated_temp)]

            self.e_eq.rel_lambda_b = rel_lambda_b
            self.lineEdit_rel_lambda_b.setText(str(self.e_eq.get_rel_lambda_b()))
        else:
            self.e_eq.rel_lambda_b = 1.0
            self.lineEdit_rel_lambda_b.setText(str(self.e_eq.get_rel_lambda_b()))


    def ta_other_value_control(self):
        """
            Relays Ambient Temperature Other Value Function
        """
        self.lineEdit_rel_lambda_b.clear()
        self.label_28.setEnabled(True)
        self.lineEdit_rel_ambient_temperature.setEnabled(True)

        ta_value = self.lineEdit_rel_ambient_temperature.text()
        rated_temp = self.comboBox_rel_rated_temperature.currentText()

        if ta_value != "" and rated_temp != "" and rated_temp != "None":
            self.e_eq.rel_lambda_b_func(float(ta_value), float(rated_temp))
            self.lineEdit_rel_lambda_b.setText(str(self.e_eq.get_rel_lambda_b()))
        else:
            self.e_eq.rel_lambda_b = 1.0
            self.lineEdit_rel_lambda_b.setText(str(self.e_eq.get_rel_lambda_b()))


    def rel_stress_func(self):
        """
            Relays Strees Function
        """
        stress = self.comboBox_rel_stress.currentText()

        if stress != "" and stress != "None":
            if stress != "Other value...":
                self.stress_combobox_control(stress)
            else:
                self.stress_other_value_control()
        else:
            self.lineEdit_rel_pi_l.clear()
            self.e_eq.rel_pi_l = 1.0
            self.lineEdit_rel_pi_l.setText(str(self.e_eq.get_rel_pi_l()))


    def stress_combobox_control(self, stress):
        """
            Relays Strees Combobox Function
        """
        rel_dict = dict(self.electrical_equipments_dict["Relays"])
        load_type = self.comboBox_rel_load_type.currentText()

        self.lineEdit_rel_stress.clear()
        self.lineEdit_rel_pi_l.clear()
        self.label_29.setEnabled(False)
        self.lineEdit_rel_stress.setEnabled(False)

        stress = stress.replace(".", "_")
        new_str_stress = "s_" + str(stress)

        if load_type != "" and load_type != "None":
            rel_pi_l = rel_dict["Stress"][str(new_str_stress)][str(load_type)]
            self.e_eq.rel_pi_l = rel_pi_l
            self.lineEdit_rel_pi_l.setText(str(self.e_eq.get_rel_pi_l()))
        else:
            self.e_eq.rel_pi_l = 1.0
            self.lineEdit_rel_pi_l.setText(str(self.e_eq.get_rel_pi_l()))


    def stress_other_value_control(self):
        """
            Relays Strees Other Value Function
        """
        self.lineEdit_rel_pi_l.clear()
        self.label_29.setEnabled(True)
        self.lineEdit_rel_stress.setEnabled(True)
        stress = self.lineEdit_rel_stress.text()
        load_type = self.comboBox_rel_load_type.currentText()

        if stress != "" and load_type != "" and load_type != "None":
            if load_type == "Resistive":
                select = 1
            elif load_type == "Inductive":
                select = 2
            elif load_type == "Lamp":
                select = 3

            self.e_eq.rel_pi_l_func(float(stress), int(select))
            self.lineEdit_rel_pi_l.setText(str(self.e_eq.get_rel_pi_l()))

        else:
            self.e_eq.rel_pi_l = 1.0
            self.lineEdit_rel_pi_l.setText(str(self.e_eq.get_rel_pi_l()))


    def rel_contact_form_func(self):
        """
            Relays Contact Form Function
        """
        rel_dict = dict(self.electrical_equipments_dict["Relays"])
        contact_form = self.comboBox_rel_contact_form.currentText()

        if contact_form != "" and contact_form != "None":
            self.lineEdit_rel_pi_c.clear()
            rel_pi_c = rel_dict["Contact Form"][str(contact_form)]
            self.e_eq.rel_pi_c = rel_pi_c
            self.lineEdit_rel_pi_c.setText(str(self.e_eq.rel_pi_c))
        else:
            self.lineEdit_rel_pi_c.clear()
            self.e_eq.rel_pi_c = 1.0
            self.lineEdit_rel_pi_c.setText(str(self.e_eq.rel_pi_c))


    def rel_cycle_rate_func(self):
        """
            Relays Cycle Rate Function
        """
        self.lineEdit_rel_pi_cyc.clear()
        rel_dict = dict(self.electrical_equipments_dict["Relays"])
        cycle = self.comboBox_rel_cycle_rate.currentText()

        if cycle != "" and cycle != "None":
            tmp_str1 = cycle.replace(".", "_")
            tmp_str2 = tmp_str1.replace(" and ", "_and_")
            tmp_str3 = tmp_str2.replace("Between ", "btw_")
            tmp_str4 = tmp_str3.replace("> ", "g_")
            tmp_str5 = tmp_str4.replace("< ", "s_")
            tmp_str6 = tmp_str5.replace(">= ", "ge_")
            rel_pi_cyc_yaml_value = rel_dict["Cycle Rate"][str(tmp_str6)]

            if rel_pi_cyc_yaml_value == "None":
                self.rel_pi_cyc_formula_func(tmp_str6)
            else:
                self.rel_pi_cyc_default_value_func(rel_pi_cyc_yaml_value)

        else:
            self.label_rel_cycle_rate_2.setEnabled(False)
            self.lineEdit_rel_cycle_rate.setEnabled(False)
            self.lineEdit_rel_cycle_rate.clear()
            self.e_eq.rel_pi_cyc = 1.0
            self.lineEdit_rel_pi_cyc.setText(str(self.e_eq.get_rel_pi_cyc()))


    def rel_pi_cyc_default_value_func(self, rel_pi_cyc_yaml_value):
        """
            Relays Pi_cyc Default Value Function
        """
        self.lineEdit_rel_cycle_rate.clear()
        self.label_rel_cycle_rate_2.setEnabled(False)
        self.lineEdit_rel_cycle_rate.setEnabled(False)
        self.e_eq.rel_pi_cyc = rel_pi_cyc_yaml_value
        self.lineEdit_rel_pi_cyc.setText(str(self.e_eq.get_rel_pi_cyc()))


    def rel_pi_cyc_formula_func(self, tmp_str):
        """
            Relays Pi_cyc Formula Function
        """
        self.label_rel_cycle_rate_2.setEnabled(True)
        self.lineEdit_rel_cycle_rate.setEnabled(True)

        cycle_val = self.lineEdit_rel_cycle_rate.text()

        if cycle_val != "":
            if "(MIL-SPEC)" in tmp_str:
                select = 0
            elif "(Commercial Quality)" in tmp_str:
                select = 1

            self.e_eq.rel_pi_cyc_func(float(cycle_val), select)
        else:
            self.e_eq.rel_pi_cyc = 1.0

        self.lineEdit_rel_pi_cyc.setText(str(self.e_eq.get_rel_pi_cyc()))


    def rel_quality_func(self):
        """
            Relays Quality Function
        """
        rel_dict = dict(self.electrical_equipments_dict["Relays"])

        quality = self.comboBox_rel_quality.currentText()

        if quality != "" and quality != "None":
            self.lineEdit_rel_pi_q.clear()
            rel_pi_q = rel_dict["Quality"][str(quality)]

            self.e_eq.rel_pi_q = rel_pi_q
            self.lineEdit_rel_pi_q.setText(str(self.e_eq.rel_pi_q))
        else:
            self.lineEdit_rel_pi_q.clear()
            self.e_eq.rel_pi_q = 1.0
            self.lineEdit_rel_pi_q.setText(str(self.e_eq.rel_pi_q))


    def rel_environment_func(self):
        """
            Relays Environment Function
        """
        rel_dict = dict(self.electrical_equipments_dict["Relays"])
        environment = self.comboBox_rel_environment.currentText()

        if environment != "" and environment != "None":
            self.lineEdit_rel_pi_e.clear()
            rel_pi_e = rel_dict["Environment"][str(environment)]
            self.e_eq.rel_pi_e = rel_pi_e
            self.lineEdit_rel_pi_e.setText(str(self.e_eq.rel_pi_e))
        else:
            self.lineEdit_rel_pi_e.clear()
            self.e_eq.rel_pi_e = 1.0
            self.lineEdit_rel_pi_e.setText(str(self.e_eq.rel_pi_e))


    def rel_pi_f_func(self):
        """
            Relays Pi_f Function
        """
        rel_dict = dict(self.electrical_equipments_dict["Relays"])
        contact_rating = self.comboBox_rel_contact_rating.currentText()

        if contact_rating != "" and contact_rating != "None":
            self.lineEdit_rel_pi_f.clear()
            val = rel_dict["pi_f"][str(contact_rating)]

            if type(val) == int:
                self.e_eq.rel_pi_f = val
                self.lineEdit_rel_pi_f.setText(str(self.e_eq.rel_pi_f))
            else:
                self.e_eq.rel_pi_f = 1.0
                self.lineEdit_rel_pi_f.setText(str(self.e_eq.rel_pi_f))


    def rel_pi_f_func_2(self):
        """
            Relays Pi_f Function 2
        """
        rel_dict = dict(self.electrical_equipments_dict["Relays"])
        contact_rating = self.comboBox_rel_contact_rating.currentText()
        application_type = self.comboBox_rel_application_type.currentText()

        if application_type != "" and application_type != "None":
            self.lineEdit_rel_pi_f.clear()
            val = rel_dict["pi_f"][str(contact_rating)][str(application_type)]

            if type(val) == int:
                self.e_eq.rel_pi_f = val
                self.lineEdit_rel_pi_f.setText(str(self.e_eq.rel_pi_f))
            else:
                self.e_eq.rel_pi_f = 1.0
                self.lineEdit_rel_pi_f.setText(str(self.e_eq.rel_pi_f))


    def rel_pi_f_func_3(self):
        """
            Relays Pi_f Function 3
        """
        rel_dict = dict(self.electrical_equipments_dict["Relays"])

        contact_rating = self.comboBox_rel_contact_rating.currentText()
        application_type = self.comboBox_rel_application_type.currentText()
        construction_type = self.comboBox_rel_construction_type.currentText()

        if construction_type != "" and construction_type != "None":
            self.lineEdit_rel_pi_f.clear()
            val = rel_dict["pi_f"][str(contact_rating)][str(application_type)][str(construction_type)]

            if type(val) == int:
                self.e_eq.rel_pi_f = val
                self.lineEdit_rel_pi_f.setText(str(self.e_eq.rel_pi_f))
            else:
                self.e_eq.rel_pi_f = 1.0
                self.lineEdit_rel_pi_f.setText(str(self.e_eq.rel_pi_f))


    def rel_hazard_rate_func(self):
        """
            Relays Hazard Rate Function
        """
        self.rel_ambient_temperature_func()
        self.rel_stress_func()
        self.rel_contact_form_func()
        self.rel_cycle_rate_func()
        self.rel_quality_func()
        self.rel_environment_func()
        self.rel_pi_f_func()
        self.rel_pi_f_func_2()
        self.rel_pi_f_func_3()

        self.e_eq.relay_func()
        self.lineEdit_rel_lamda_p.setText(str(self.e_eq.get_rel_lambda()))


  # CONNECTORS, GENERAL
    def connectors_general_tab_func(self):
        """
            Connectors, General Tab Function
        """
        con_gen_dict = dict(self.electrical_equipments_dict["Connectors, General"])

        description_list = list(con_gen_dict["Description"].keys())
        temperature_list = list(con_gen_dict["Ambient Temperature"].keys())
        cycle_list = list(con_gen_dict["Mating-Unmating Cycles"].keys())
        quality_list = list(con_gen_dict["Quality"].keys())
        environment_list = list(con_gen_dict["Environment"].keys())

        temp_list_1 = list()
        temp_list_2 = list()

        for item in temperature_list:
            temp_str = item.split("t_")
            new_val = int(temp_str[1])
            temp_list_1.append(new_val)

        for item in cycle_list:
            tmp_str1 = item.replace("_to_", " to ")
            tmp_str2 = tmp_str1.replace("g_", "> ")
            tmp_str3 = tmp_str2.replace("_", ".")
            temp_list_2.append(tmp_str3)

        temp_list_1.sort()
        temp_list_2.sort()

        temp_list_1 = map(str, temp_list_1)
        temp_list_2 = map(str, temp_list_2)

        self.comboBox_con_gen_description.clear()
        self.comboBox_con_gen_t0.clear()
        self.comboBox_con_gen_cycles.clear()
        self.comboBox_con_gen_quality.clear()
        self.comboBox_con_gen_environment.clear()

        self.comboBox_con_gen_description.addItem("None")
        self.comboBox_con_gen_t0.addItem("None")
        self.comboBox_con_gen_cycles.addItem("None")
        self.comboBox_con_gen_quality.addItem("None")
        self.comboBox_con_gen_environment.addItem("None")

        self.comboBox_con_gen_description.addItems(list(description_list))
        self.comboBox_con_gen_t0.addItems(list(temp_list_1))
        self.comboBox_con_gen_cycles.addItems(list(temp_list_2))
        self.comboBox_con_gen_quality.addItems(list(quality_list))
        self.comboBox_con_gen_environment.addItems(list(environment_list))

        self.comboBox_con_gen_t0.addItem("Other value...")
        self.con_gen_hazard_rate_func()


    def con_gen_description_func(self):
        """
            Connectors, General Description Function
        """
        description = self.comboBox_con_gen_description.currentText()
        con_gen_dict = dict(self.electrical_equipments_dict["Connectors, General"])

        if description != "" and description != "None":
            self.lineEdit_con_gen_lambda_b.clear()
            con_gen_lambda_b = con_gen_dict["Description"][str(description)]
            self.e_eq.con_gen_lambda_b = con_gen_lambda_b
            self.lineEdit_con_gen_lambda_b.setText(str(self.e_eq.con_gen_lambda_b))
        else:
            self.lineEdit_con_gen_lambda_b.clear()
            self.e_eq.con_gen_lambda_b = 1.0
            self.lineEdit_con_gen_lambda_b.setText(str(self.e_eq.con_gen_lambda_b))


    def con_gen_t0_func(self):
        """
            Connectors, General t0 Function
        """
        t0_value = self.comboBox_con_gen_t0.currentText()

        if t0_value != "" and t0_value != "None":
            if t0_value != "Other value...":
                self.t0_combobox_control(t0_value)
            else:
                self.t0_other_value_control()
        else:
            self.lineEdit_con_gen_pi_t.clear()
            self.e_eq.con_gen_pi_t = 1.0
            self.lineEdit_con_gen_pi_t.setText(str(self.e_eq.get_con_gen_pi_t()))


    def t0_combobox_control(self, t0_value):
        """
            Connectors, General t0 Combobox Function
        """
        con_gen_dict = dict(self.electrical_equipments_dict["Connectors, General"])
        self.lineEdit_con_gen_t0.clear()
        self.lineEdit_con_gen_pi_t.clear()
        self.label_17.setEnabled(False)
        self.lineEdit_con_gen_t0.setEnabled(False)

        new_str = "t_" + str(t0_value)
        con_gen_pi_t = con_gen_dict["Ambient Temperature"][str(new_str)]

        self.e_eq.con_gen_pi_t = con_gen_pi_t
        self.lineEdit_con_gen_pi_t.setText(str(self.e_eq.get_con_gen_pi_t()))


    def t0_other_value_control(self):
        """
            Connectors, General t0 Other Value Function
        """
        self.lineEdit_con_gen_pi_t.clear()
        self.label_17.setEnabled(True)
        self.lineEdit_con_gen_t0.setEnabled(True)

        t0_value = self.lineEdit_con_gen_t0.text()

        if t0_value != "":
            self.e_eq.con_gen_pi_t_func(float(t0_value))
            self.lineEdit_con_gen_pi_t.setText(str(self.e_eq.get_con_gen_pi_t()))


    def con_gen_cycles_func(self):
        """
            Connectors, General Cycles Function
        """
        con_gen_dict = dict(self.electrical_equipments_dict["Connectors, General"])
        cycle = self.comboBox_con_gen_cycles.currentText()

        if cycle != "" and cycle != "None":
            self.lineEdit_con_gen_pi_k.clear()
            tmp_str1 = cycle.replace(".", "_")
            tmp_str2 = tmp_str1.replace(" to ", "_to_")
            tmp_str3 = tmp_str2.replace("> ", "g_")
            con_gen_pi_k = con_gen_dict["Mating-Unmating Cycles"][str(tmp_str3)]

            self.e_eq.con_gen_pi_k = con_gen_pi_k
            self.lineEdit_con_gen_pi_k.setText(str(self.e_eq.con_gen_pi_k))
        else:
            self.lineEdit_con_gen_pi_k.clear()
            self.e_eq.con_gen_pi_k = 1.0
            self.lineEdit_con_gen_pi_k.setText(str(self.e_eq.con_gen_pi_k))


    def con_gen_quality_func(self):
        """
            Connectors, General Quality Function
        """
        quality = self.comboBox_con_gen_quality.currentText()
        con_gen_dict = dict(self.electrical_equipments_dict["Connectors, General"])

        if quality != "" and quality != "None":
            self.lineEdit_con_gen_pi_q.clear()
            con_gen_pi_q = con_gen_dict["Quality"][str(quality)]
            self.e_eq.con_gen_pi_q = con_gen_pi_q
            self.lineEdit_con_gen_pi_q.setText(str(self.e_eq.con_gen_pi_q))
        else:
            self.lineEdit_con_gen_pi_q.clear()
            self.e_eq.con_gen_pi_q = 1.0
            self.lineEdit_con_gen_pi_q.setText(str(self.e_eq.con_gen_pi_q))


    def con_gen_environment_func(self):
        """
            Connectors, General Environment Function
        """
        environment = self.comboBox_con_gen_environment.currentText()
        con_gen_dict = dict(self.electrical_equipments_dict["Connectors, General"])

        if environment != "" and environment != "None":
            self.lineEdit_con_gen_pi_e.clear()
            con_gen_pi_e = con_gen_dict["Environment"][str(environment)]
            self.e_eq.con_gen_pi_e = con_gen_pi_e
            self.lineEdit_con_gen_pi_e.setText(str(self.e_eq.con_gen_pi_e))
        else:
            self.lineEdit_con_gen_pi_e.clear()
            self.e_eq.con_gen_pi_e = 1.0
            self.lineEdit_con_gen_pi_e.setText(str(self.e_eq.con_gen_pi_e))


    def con_gen_hazard_rate_func(self):
        """
            Connectors, General Hazard Rate Function
        """
        self.con_gen_description_func()
        self.con_gen_t0_func()
        self.con_gen_cycles_func()
        self.con_gen_quality_func()
        self.con_gen_environment_func()
        self.e_eq.connectors_general_func()
        self.lineEdit_con_gen_lamda_p.setText(str(self.e_eq.get_con_gen_lambda()))


  # CONNECTORS, SOCKETS
    def connectors_sockets_tab_func(self):
        """
            Connectors, Sockets Tab Function
        """
        con_sock_dict = dict(self.electrical_equipments_dict["Connectors, Sockets"])
        description_list = list(con_sock_dict["Description"].keys())
        quality_list = list(con_sock_dict["Quality"].keys())
        active_contact_numb_list = list(con_sock_dict["Number of Active Contacts"].keys())
        environment_list = list(con_sock_dict["Environment"].keys())

        temp_list_1 = list()

        for item in active_contact_numb_list:
            temp_str = item.split("numb_")
            new_val = int(temp_str[1])
            temp_list_1.append(new_val)

        temp_list_1.sort()
        temp_list_1 = map(str, temp_list_1)

        self.comboBox_con_sock_description.clear()
        self.comboBox_con_sock_quality.clear()
        self.comboBox_con_sock_numb_of_active_contacts.clear()
        self.comboBox_con_sock_environment.clear()
        self.comboBox_con_sock_description.addItem("None")
        self.comboBox_con_sock_quality.addItem("None")
        self.comboBox_con_sock_numb_of_active_contacts.addItem("None")
        self.comboBox_con_sock_environment.addItem("None")
        self.comboBox_con_sock_description.addItems(list(description_list))
        self.comboBox_con_sock_quality.addItems(list(quality_list))
        self.comboBox_con_sock_numb_of_active_contacts.addItems(list(temp_list_1))
        self.comboBox_con_sock_environment.addItems(list(environment_list))
        self.comboBox_con_sock_numb_of_active_contacts.addItem("Other value...")
        self.con_sock_hazard_rate_func()


    def con_sock_quality_func(self):
        """
            Connectors, Sockets Quality Function
        """
        quality = self.comboBox_con_sock_quality.currentText()
        con_sock_dict = dict(self.electrical_equipments_dict["Connectors, Sockets"])

        if quality != "" and quality != "None":
            self.lineEdit_con_sock_pi_q.clear()
            con_sock_pi_q = con_sock_dict["Quality"][str(quality)]
            self.e_eq.con_sock_pi_q = con_sock_pi_q
            self.lineEdit_con_sock_pi_q.setText(str(self.e_eq.con_sock_pi_q))
        else:
            self.lineEdit_con_sock_pi_q.clear()
            self.e_eq.con_sock_pi_q = 1.0
            self.lineEdit_con_sock_pi_q.setText(str(self.e_eq.con_sock_pi_q))


    def con_sock_environment_func(self):
        """
            Connectors, Sockets Environment Function
        """
        environment = self.comboBox_con_sock_environment.currentText()
        con_sock_dict = dict(self.electrical_equipments_dict["Connectors, Sockets"])

        if environment != "" and environment != "None":
            self.lineEdit_con_sock_pi_e.clear()
            con_sock_pi_e = con_sock_dict["Environment"][str(environment)]
            self.e_eq.con_sock_pi_e = con_sock_pi_e
            self.lineEdit_con_sock_pi_e.setText(str(self.e_eq.con_sock_pi_e))
        else:
            self.lineEdit_con_sock_pi_e.clear()
            self.e_eq.con_sock_pi_e = 1.0
            self.lineEdit_con_sock_pi_e.setText(str(self.e_eq.con_sock_pi_e))


    def con_sock_description_func(self):
        """
            Connectors, Sockets Description Function
        """
        description = self.comboBox_con_sock_description.currentText()
        con_sock_dict = dict(self.electrical_equipments_dict["Connectors, Sockets"])

        if description != "" and description != "None":
            self.lineEdit_con_sock_lambda_b.clear()
            con_sock_lambda_b = con_sock_dict["Description"][str(description)]
            self.e_eq.con_sock_lambda_b = con_sock_lambda_b
            self.lineEdit_con_sock_lambda_b.setText(str(self.e_eq.con_sock_lambda_b))
        else:
            self.lineEdit_con_sock_lambda_b.clear()
            self.e_eq.con_sock_lambda_b = 1.0
            self.lineEdit_con_sock_lambda_b.setText(str(self.e_eq.con_sock_lambda_b))


    def con_sock_numb_of_active_contacts_func(self):
        """
            Connectors, Sockets Numb of Active Contacts Function
        """
        numb = self.comboBox_con_sock_numb_of_active_contacts.currentText()

        if numb != "" and numb != "None":
            if numb != "Other value...":
                self.numb_of_active_contact_combobox_control(numb)
            else:
                self.con_sock_other_value_control()
        else:
            self.lineEdit_con_sock_pi_p.clear()
            self.e_eq.con_sock_pi_p = 1.0
            self.lineEdit_con_sock_pi_p.setText(str(self.e_eq.get_con_sock_pi_p()))


    def numb_of_active_contact_combobox_control(self, numb):
        """
            Connectors, Sockets Numb of Active Contacts Combobox Function
        """
        con_sock_dict = dict(self.electrical_equipments_dict["Connectors, Sockets"])
        self.lineEdit_con_sock_numb_of_active_contacts.clear()
        self.lineEdit_con_sock_pi_p.clear()
        self.label_16.setEnabled(False)
        self.lineEdit_con_sock_numb_of_active_contacts.setEnabled(False)

        new_str = "numb_" + str(numb)
        con_sock_pi_p = con_sock_dict["Number of Active Contacts"][str(new_str)]

        self.e_eq.con_sock_pi_p = con_sock_pi_p
        self.lineEdit_con_sock_pi_p.setText(str(self.e_eq.get_con_sock_pi_p()))


    def con_sock_other_value_control(self):
        """
            Connectors, Sockets Other Value Function
        """
        self.lineEdit_con_sock_pi_p.clear()
        self.label_16.setEnabled(True)
        self.lineEdit_con_sock_numb_of_active_contacts.setEnabled(True)

        numb = self.lineEdit_con_sock_numb_of_active_contacts.text()

        if numb != "":
            self.e_eq.con_sock_pi_p_func(int(numb))
            self.lineEdit_con_sock_pi_p.setText(str(self.e_eq.get_con_sock_pi_p()))


    def con_sock_hazard_rate_func(self):
        """
            Connectors, Sockets Hazard Rate Function
        """
        self.con_sock_description_func()
        self.con_sock_environment_func()
        self.con_sock_quality_func()
        self.con_sock_numb_of_active_contacts_func()
        self.e_eq.connectors_sockets_func()
        self.lineEdit_con_sock_lamda_p.setText(str(self.e_eq.get_con_sock_lambda()))


  # QUARTZ CRYSTALS
    def quartz_crystals_tab_func(self):
        """
            Quartz Crystals Tab Function
        """
        quartz_dict = dict(self.electrical_equipments_dict["Quartz Crystals"])
        frequency_list = list(quartz_dict["Frequency"].keys())
        environment_list = list(quartz_dict["Environment"].keys())
        quality_list = list(quartz_dict["Quality"].keys())
        temp_list_1 = list()

        for item in frequency_list:
            temp_str = item.split("f_")
            new_val = float(temp_str[1].replace("_", "."))
            temp_list_1.append(new_val)

        temp_list_1.sort()
        temp_list_1 = map(str, temp_list_1)

        self.comboBox_qrtz_frequency.clear()
        self.comboBox_qrtz_quality.clear()
        self.comboBox_qrtz_environment.clear()

        self.comboBox_qrtz_frequency.addItem("None")
        self.comboBox_qrtz_quality.addItem("None")
        self.comboBox_qrtz_environment.addItem("None")

        self.comboBox_qrtz_frequency.addItems(temp_list_1)
        self.comboBox_qrtz_quality.addItems(quality_list)
        self.comboBox_qrtz_environment.addItems(environment_list)

        self.comboBox_qrtz_frequency.addItem("Other value...")

        self.quartz_hazard_rate_func()


    def qrtz_frequency_func(self):
        """
            Quartz Crystals Frequency Function
        """
        frequency = self.comboBox_qrtz_frequency.currentText()

        if frequency != "" and frequency != "None":
            if frequency != "Other value...":
                self.frequency_combobox_control(frequency)
            else:
                self.qrtz_other_value_control()
        else:
            self.lineEdit_qrtz_lambda_b.clear()
            self.e_eq.qrtz_lambda_b = 1.0
            self.lineEdit_qrtz_lambda_b.setText(str(self.e_eq.get_qrtz_lambda_b()))


    def frequency_combobox_control(self, frequency):
        """
            Quartz Crystals Frequency Combobox Function
        """
        quartz_dict = dict(self.electrical_equipments_dict["Quartz Crystals"])
        self.lineEdit_qrtz_frequency.clear()
        self.lineEdit_qrtz_lambda_b.clear()
        self.label_15.setEnabled(False)
        self.lineEdit_qrtz_frequency.setEnabled(False)

        frequency = frequency.replace(".", "_")
        new_str = "f_" + str(frequency)
        qrtz_lambda_b = quartz_dict["Frequency"][str(new_str)]

        self.e_eq.qrtz_lambda_b = qrtz_lambda_b
        self.lineEdit_qrtz_lambda_b.setText(str(self.e_eq.get_qrtz_lambda_b()))


    def qrtz_other_value_control(self):
        """
            Quartz Crystals Other Value Function
        """
        self.lineEdit_qrtz_lambda_b.clear()
        self.label_15.setEnabled(True)
        self.lineEdit_qrtz_frequency.setEnabled(True)
        frequency = self.lineEdit_qrtz_frequency.text()

        if frequency != "":
            self.e_eq.qrtz_lambda_b_func(float(frequency))
            self.lineEdit_qrtz_lambda_b.setText(str(self.e_eq.get_qrtz_lambda_b()))


    def qrtz_quality_func(self):
        """
            Quartz Crystals Quality Function
        """
        quality = self.comboBox_qrtz_quality.currentText()
        quartz_dict = dict(self.electrical_equipments_dict["Quartz Crystals"])

        if quality != "" and quality != "None":
            self.lineEdit_qrtz_pi_q.clear()
            qrtz_pi_q = quartz_dict["Quality"][str(quality)]
            self.e_eq.qrtz_pi_q = qrtz_pi_q
            self.lineEdit_qrtz_pi_q.setText(str(self.e_eq.qrtz_pi_q))
        else:
            self.lineEdit_qrtz_pi_q.clear()
            self.e_eq.qrtz_pi_q = 1.0
            self.lineEdit_qrtz_pi_q.setText(str(self.e_eq.qrtz_pi_q))


    def qrtz_environment_func(self):
        """
            Quartz Crystals Environment Function
        """
        environment = self.comboBox_qrtz_environment.currentText()
        quartz_dict = dict(self.electrical_equipments_dict["Quartz Crystals"])

        if environment != "" and environment != "None":
            self.lineEdit_qrtz_pi_e.clear()
            qrtz_pi_e = quartz_dict["Environment"][str(environment)]
            self.e_eq.qrtz_pi_e = qrtz_pi_e
            self.lineEdit_qrtz_pi_e.setText(str(self.e_eq.qrtz_pi_e))
        else:
            self.lineEdit_qrtz_pi_e.clear()
            self.e_eq.qrtz_pi_e = 1.0
            self.lineEdit_qrtz_pi_e.setText(str(self.e_eq.qrtz_pi_e))


    def quartz_hazard_rate_func(self):
        """
            Quartz Crystals Hazard Rate Function
        """
        self.qrtz_frequency_func()
        self.qrtz_environment_func()
        self.qrtz_quality_func()
        self.e_eq.quartz_crystals_func()
        self.lineEdit_qrtz_lamda_p.setText(str(self.e_eq.get_qrtz_lambda()))


  # CAPACITORS
    def capacitor_style_data(self):
        """
            Capacitors Style Function
        """
        temp_capacitor_style_list = list(self.electrical_equipments_dict["Capacitor"]["Capacitor Style"].keys())
        comboBox_capacitor_style_list = list()

        for item in temp_capacitor_style_list:
            temp = str(item).upper()
            comboBox_capacitor_style_list.append(str(temp))

        comboBox_capacitor_style_list.sort()
        self.comboBox_cc_style.addItem("None")
        self.comboBox_cc_style.addItems(comboBox_capacitor_style_list)


    def capacitor_tab_func(self):
        """
            Capacitors Tab Function
        """
        self.comboBox_cc_capacitor_temperature.clear()
        self.comboBox_cc_circuit_resistance.clear()
        self.comboBox_cc_capacitance.clear()
        self.comboBox_cc_voltage_stress.clear()
        self.comboBox_cc_quality.clear()
        self.comboBox_cc_environment.clear()

        self.comboBox_cc_capacitor_temperature.addItem("None")
        self.comboBox_cc_circuit_resistance.addItem("None")
        self.comboBox_cc_capacitance.addItem("None")
        self.comboBox_cc_voltage_stress.addItem("None")
        self.comboBox_cc_quality.addItem("None")
        self.comboBox_cc_environment.addItem("None")

        if self.comboBox_cc_style.currentText() != "None":
            temp_capacitor = str(self.comboBox_cc_style.currentText()).lower()
            temp_dict = self.capacitor_style_dict[temp_capacitor]

            temp_temperature_factor_list = list(self.electrical_equipments_dict["Capacitor"]["Temperature Factor"]["column_" + str(temp_dict["Temperature Factor"])].keys())
            temp_capacitance_factor_list = list(self.electrical_equipments_dict["Capacitor"]["Capacitance Factor"]["column_" + str(temp_dict["Capacitance Factor"])].keys())
            temp_voltage_stress_list = list(self.electrical_equipments_dict["Capacitor"]["Voltage Stress"]["column_" + str(temp_dict["Voltage Stress"])].keys())
            temp_quality_list = list(self.electrical_equipments_dict["Capacitor"]["Quality"].keys())
            temp_environment_factor_list = list(self.electrical_equipments_dict["Capacitor"]["Environment Factor"].keys())

            comboBox_temperature_factor_list = list()
            comboBox_capacitance_factor_list = list()
            comboBox_voltage_stress_list = list()
            comboBox_circuit_resistance_list = list()
            comboBox_quality_list = list()
            comboBox_environment_factor_list = list()

            if temp_dict["Circuit Resistance"] == "pi_sr":
                temp_circuit_resistance_list = list(self.electrical_equipments_dict["Capacitor"]["Circuit Resistance"].keys())

                for item in temp_circuit_resistance_list:
                    temp = str(item).split("cr_")
                    replace = temp[1].replace('_to_', " to ")
                    tmp = replace.replace('g_', '> ')
                    comboBox_circuit_resistance_list.append(tmp.replace('_', '.'))

                comboBox_circuit_resistance_list.sort()
                self.comboBox_cc_circuit_resistance.addItems(comboBox_circuit_resistance_list)

            for item in temp_temperature_factor_list:
                temp = str(item).split("c_")
                comboBox_temperature_factor_list.append(int(temp[1].replace('_', '.')))

            for item in temp_capacitance_factor_list:
                temp = str(item).split("cf_")
                comboBox_capacitance_factor_list.append(float(temp[1].replace('_', '.')))

            for item in temp_voltage_stress_list:
                temp = str(item).split("vs_")
                comboBox_voltage_stress_list.append(float(temp[1].replace('_', '.')))

            for item in temp_quality_list:
                temp = str(item).upper()
                comboBox_quality_list.append(temp.replace('_', ' '))

            for item in temp_environment_factor_list:
                temp = str(item).upper()
                comboBox_environment_factor_list.append(temp.replace('_', ' - '))

            comboBox_temperature_factor_list.sort()
            comboBox_capacitance_factor_list.sort()
            comboBox_voltage_stress_list.sort()
            comboBox_quality_list.sort()
            comboBox_environment_factor_list.sort()

            comboBox_temperature_factor_list = map(str, comboBox_temperature_factor_list)
            comboBox_capacitance_factor_list = map(str, comboBox_capacitance_factor_list)
            comboBox_voltage_stress_list = map(str, comboBox_voltage_stress_list)

            self.comboBox_cc_capacitor_temperature.addItems(comboBox_temperature_factor_list)
            self.comboBox_cc_capacitance.addItems(comboBox_capacitance_factor_list)
            self.comboBox_cc_voltage_stress.addItems(comboBox_voltage_stress_list)
            self.comboBox_cc_quality.addItems(comboBox_quality_list)
            self.comboBox_cc_environment.addItems(comboBox_environment_factor_list)

        self.capacitor_hazard_rate_func()


    def capacitor_hazard_rate_func(self):
        """
            Capacitors Hazard Rate Function
        """
        cb_cc_capacitor_temperature = self.comboBox_cc_capacitor_temperature.currentText()
        cb_cc_circuit_resistance = self.comboBox_cc_circuit_resistance.currentText()
        cb_cc_capacitance = self.comboBox_cc_capacitance.currentText()
        cb_cc_voltage_stress = self.comboBox_cc_voltage_stress.currentText()
        cb_cc_quality = self.comboBox_cc_quality.currentText()
        cb_cc_environment = self.comboBox_cc_environment.currentText()

        self.lineEdit_cc_lambda_b.clear()
        self.lineEdit_cc_pi_t.clear()
        self.lineEdit_cc_pi_sr.clear()
        self.lineEdit_cc_pi_c.clear()
        self.lineEdit_cc_pi_v.clear()
        self.lineEdit_cc_pi_q.clear()
        self.lineEdit_cc_pi_e.clear()
        self.lineEdit_cc_lamda_p.clear()

        if cb_cc_capacitor_temperature != "" and cb_cc_circuit_resistance != "" and cb_cc_capacitance != "" and cb_cc_voltage_stress != "" and cb_cc_quality != "" and cb_cc_environment != "":
            temp_capacitor = str(self.comboBox_cc_style.currentText()).lower()

            if temp_capacitor != "none":
                temp_dict = self.capacitor_style_dict[temp_capacitor]
                self.e_eq.cc_lambda_b = self.electrical_equipments_dict["Capacitor"]["Capacitor Style"][temp_capacitor]
                self.lineEdit_cc_lambda_b.setText(str(self.e_eq.cc_lambda_b))
            else:
                self.e_eq.cc_lambda_b = 1.0

            if cb_cc_circuit_resistance != "None":
                temp_1 = cb_cc_circuit_resistance.replace('.', '_')
                temp_2 = temp_1.replace(' to ', '_to_')
                temp_3 = temp_2.replace('> ', 'g_')

                select_circuit_resistance = str("cr_" + temp_3)
                self.e_eq.cc_pi_sr_func(self.electrical_equipments_dict["Capacitor"]["Circuit Resistance"][select_circuit_resistance])
                self.lineEdit_cc_pi_sr.setText(str(self.e_eq.get_cc_pi_sr()))
            else:
                self.e_eq.cc_pi_sr_func("None")

            if cb_cc_capacitance != "None":
                select_capacitance_factor = str("cf_" + cb_cc_capacitance.replace('.', '_'))
                self.e_eq.cc_pi_c_func(self.electrical_equipments_dict["Capacitor"]["Capacitance Factor"]["column_" + str(temp_dict["Capacitance Factor"])][select_capacitance_factor], 0)
                self.lineEdit_cc_pi_c.setText(str(self.e_eq.get_cc_pi_c()))
            else:
                self.e_eq.cc_pi_c_func(0, 9)

            if cb_cc_voltage_stress != "None":
                select_voltage_stress = str("vs_" + cb_cc_voltage_stress.replace('.', '_'))
                self.e_eq.cc_pi_v_func(self.electrical_equipments_dict["Capacitor"]["Voltage Stress"]["column_" + str(temp_dict["Voltage Stress"])][select_voltage_stress], 0)
                self.lineEdit_cc_pi_v.setText(str(self.e_eq.get_cc_pi_v()))
            else:
                self.e_eq.cc_pi_v_func(0, 9)

            if cb_cc_quality != "None":
                temp = cb_cc_quality.replace(' ', '_')
                select_quality = str(temp.lower())
                self.e_eq.cc_pi_q = float(self.electrical_equipments_dict["Capacitor"]["Quality"][select_quality])
                self.lineEdit_cc_pi_q.setText(str(self.e_eq.cc_pi_q))
            else:
                self.e_eq.cc_pi_q = 1.0

            if cb_cc_environment != "None":
                temp = cb_cc_environment.replace(' - ', '_')
                select_environment = str(temp.lower())
                self.e_eq.cc_pi_e = float(self.electrical_equipments_dict["Capacitor"]["Environment Factor"][select_environment])
                self.lineEdit_cc_pi_e.setText(str(self.e_eq.cc_pi_e))
            else:
                self.e_eq.cc_pi_e = 1.0

            if cb_cc_capacitor_temperature != "None":
                select_capacitor_temperature = str("c_" + cb_cc_capacitor_temperature.replace('.', '_'))
                self.e_eq.cc_pi_t_func(self.electrical_equipments_dict["Capacitor"]["Temperature Factor"]["column_" + str(temp_dict["Temperature Factor"])][select_capacitor_temperature], 0)
                self.lineEdit_cc_pi_t.setText(str(self.e_eq.get_cc_pi_t()))
            else:
                self.e_eq.cc_pi_t_func(0, 9)

            # Calculate Capacitors Hazard Rate
            self.e_eq.capacitors_func()

            self.lineEdit_cc_lamda_p.setText(str(self.e_eq.get_cc_lambda()))


  # DIODES
    def diode_type_data(self):
        """
            Diodes Type Function
        """
        comboBox_dd_type_list = list(self.electrical_equipments_dict["Diode"]["Base Failure Rate"].keys())
        comboBox_dd_type_list.sort()
        self.comboBox_dd_type.addItem("None")
        self.comboBox_dd_type.addItems(comboBox_dd_type_list)


    def diode_tab_func(self):
        """
            Diodes Tab Function
        """
        self.comboBox_dd_junction_temperature.clear()
        self.comboBox_dd_voltage_stress.clear()
        self.comboBox_dd_contact_construction.clear()
        self.comboBox_dd_quality.clear()
        self.comboBox_dd_environment.clear()

        self.comboBox_dd_junction_temperature.addItem("None")
        self.comboBox_dd_voltage_stress.addItem("None")
        self.comboBox_dd_contact_construction.addItem("None")
        self.comboBox_dd_quality.addItem("None")
        self.comboBox_dd_environment.addItem("None")

        if self.comboBox_dd_type.currentText() != "None":
            temp_diode = str(self.comboBox_dd_type.currentText())
            temp_dict = self.diode_type_dict[temp_diode]

            comboBox_temperature_factor_list = list()
            comboBox_voltage_stress_list = list()
            comboBox_contact_construction_list = list()
            comboBox_quality_list = list()
            comboBox_environment_list = list()

            temp_temperature_factor_list = list(self.electrical_equipments_dict["Diode"]["Temperature Factor"]["column_" + str(temp_dict["Temperature Factor"])].keys())
            temp_voltage_stress_list = list(self.electrical_equipments_dict["Diode"]["Electrical Stress Factor"]["column_" + str(temp_dict["Electrical Stress Factor"])].keys())
            temp_contact_construction_list = list(self.electrical_equipments_dict["Diode"]["Contact Construction Factor"].keys())
            temp_quality_list = list(self.electrical_equipments_dict["Diode"]["Quality Factor"].keys())
            temp_environment_list = list(self.electrical_equipments_dict["Diode"]["Environment Factor"].keys())

            for item in temp_temperature_factor_list:
                temp = str(item).split("c_")
                comboBox_temperature_factor_list.append(int(temp[1].replace('_', '.')))

            for item in temp_voltage_stress_list:
                temp_1 = str(item).split("esf_")
                temp_2 = temp_1[1].replace('_and_', " and ")
                temp_3 = temp_2.replace('g_', '> ')
                temp_4 = temp_3.replace('s_', '< ')
                comboBox_voltage_stress_list.append(temp_4.replace('_', '.'))

            for item in temp_environment_list:
                temp = str(item).upper()
                comboBox_environment_list.append(temp.replace('_', ' - '))

            comboBox_contact_construction_list = temp_contact_construction_list
            comboBox_quality_list = temp_quality_list

            comboBox_temperature_factor_list.sort()
            comboBox_voltage_stress_list.sort()
            comboBox_contact_construction_list.sort()
            comboBox_quality_list.sort()
            comboBox_environment_list.sort()

            comboBox_temperature_factor_list = map(str, comboBox_temperature_factor_list)
            comboBox_voltage_stress_list = map(str, comboBox_voltage_stress_list)

            self.comboBox_dd_junction_temperature.addItems(comboBox_temperature_factor_list)
            self.comboBox_dd_voltage_stress.addItems(comboBox_voltage_stress_list)
            self.comboBox_dd_contact_construction.addItems(comboBox_contact_construction_list)
            self.comboBox_dd_quality.addItems(comboBox_quality_list)
            self.comboBox_dd_environment.addItems(comboBox_environment_list)

        self.diode_hazard_rate_func()


    def diode_hazard_rate_func(self):
        """
            Diodes Hazard Rate Function
        """
        cb_dd_type = self.comboBox_dd_type.currentText()
        cb_dd_junction_temperature = self.comboBox_dd_junction_temperature.currentText()
        cb_dd_voltage_stress = self.comboBox_dd_voltage_stress.currentText()
        cb_dd_contact_construction = self.comboBox_dd_contact_construction.currentText()
        cb_dd_quality = self.comboBox_dd_quality.currentText()
        cb_dd_environment = self.comboBox_dd_environment.currentText()

        self.lineEdit_dd_lambda_b.clear()
        self.lineEdit_dd_pi_t.clear()
        self.lineEdit_dd_pi_s.clear()
        self.lineEdit_dd_pi_c.clear()
        self.lineEdit_dd_pi_q.clear()
        self.lineEdit_dd_pi_e.clear()

        if cb_dd_type != "" and cb_dd_junction_temperature != "" and cb_dd_voltage_stress != "" and cb_dd_contact_construction != "" and cb_dd_quality != "" and cb_dd_environment != "":
            if cb_dd_type != "None":
                temp_dict = self.diode_type_dict[cb_dd_type]
                self.e_eq.dd_lambda_b = float(self.electrical_equipments_dict["Diode"]["Base Failure Rate"][cb_dd_type])
                self.lineEdit_dd_lambda_b.setText(str(self.e_eq.dd_lambda_b))
            else:
                self.e_eq.dd_lambda_b = 1.0

            if cb_dd_junction_temperature != "None":
                select_temperature_factor = str("c_" + cb_dd_junction_temperature.replace('.', '_'))
                self.e_eq.dd_pi_t_func(self.electrical_equipments_dict["Diode"]["Temperature Factor"]["column_" + str(temp_dict["Temperature Factor"])][select_temperature_factor], 0)
                self.lineEdit_dd_pi_t.setText(str(self.e_eq.get_dd_pi_t()))
            else:
                self.e_eq.dd_pi_t_func(0, 9)

            if cb_dd_voltage_stress != "None":
                temp_1 = cb_dd_voltage_stress.replace('.', '_')
                temp_2 = temp_1.replace(' and ', '_and_')
                temp_3 = temp_2.replace('> ', 'g_')
                temp_4 = temp_3.replace('< ', 's_')

                select_temperature_factor = str("esf_" + temp_4)
                self.e_eq.dd_pi_s_func(self.electrical_equipments_dict["Diode"]["Electrical Stress Factor"]["column_" + str(temp_dict["Electrical Stress Factor"])][select_temperature_factor], int(temp_dict["Electrical Stress Factor"]))
                self.lineEdit_dd_pi_s.setText(str(self.e_eq.get_dd_pi_s()))
            else:
                self.e_eq.dd_pi_s_func(0, 0)

            if cb_dd_contact_construction != "None":
                select_contact_construction = str(cb_dd_contact_construction)
                self.e_eq.dd_pi_c = float(self.electrical_equipments_dict["Diode"]["Contact Construction Factor"][select_contact_construction])
                self.lineEdit_dd_pi_c.setText(str(self.e_eq.dd_pi_c))
            else:
                self.e_eq.dd_pi_c = 1.0

            if cb_dd_quality != "None":
                select_quality = str(cb_dd_quality)
                self.e_eq.dd_pi_q = float(self.electrical_equipments_dict["Diode"]["Quality Factor"][select_quality])
                self.lineEdit_dd_pi_q.setText(str(self.e_eq.dd_pi_q))
            else:
                self.e_eq.dd_pi_q = 1.0

            if cb_dd_environment != "None":
                temp = cb_dd_environment.replace(' - ', '_')
                select_environment = str(temp.lower())
                self.e_eq.dd_pi_e = float(self.electrical_equipments_dict["Diode"]["Environment Factor"][select_environment])
                self.lineEdit_dd_pi_e.setText(str(self.e_eq.dd_pi_e))
            else:
                self.e_eq.dd_pi_e = 1.0

            self.e_eq.diodes_func()
            self.lineEdit_dd_lamda_p.setText(str(self.e_eq.get_dd_lambda()))


  # INDUCTORS
    def inductor_tab_func(self):
        """
            Inductors Tab Function
        """
        self.comboBox_id_type.clear()
        self.comboBox_id_hot_spot_temperature.clear()
        self.comboBox_id_quality.clear()
        self.comboBox_id_environment.clear()

        self.comboBox_id_type.addItem("None")
        self.comboBox_id_hot_spot_temperature.addItem("None")
        self.comboBox_id_quality.addItem("None")
        self.comboBox_id_environment.addItem("None")

        comboBox_id_type_list = list()
        comboBox_id_hot_spot_temperature_list = list()
        comboBox_id_quality_list = list()
        comboBox_id_environment_list = list()

        temp_id_type_list = list(self.electrical_equipments_dict["Inductor"]["Inductor Type"].keys())
        temp_id_hot_spot_temperature_list = list(self.electrical_equipments_dict["Inductor"]["Temperature Factor"].keys())
        temp_id_quality_list = list(self.electrical_equipments_dict["Inductor"]["Quality Factor"].keys())
        temp_id_environment_list = list(self.electrical_equipments_dict["Inductor"]["Environment Factor"].keys())

        for item in temp_id_hot_spot_temperature_list:
            temp = str(item).split("c_")
            comboBox_id_hot_spot_temperature_list.append(int(temp[1].replace('_', '.')))

        for item in temp_id_environment_list:
            temp = str(item).upper()
            comboBox_id_environment_list.append(temp.replace('_', ' - '))

        for item in temp_id_quality_list:
            temp = str(item)
            comboBox_id_quality_list.append(temp.replace('_', ' - '))

        comboBox_id_type_list = temp_id_type_list

        comboBox_id_type_list.sort()
        comboBox_id_hot_spot_temperature_list.sort()
        comboBox_id_quality_list.sort()
        comboBox_id_environment_list.sort()

        comboBox_id_hot_spot_temperature_list = map(str, comboBox_id_hot_spot_temperature_list)

        self.comboBox_id_type.addItems(comboBox_id_type_list)
        self.comboBox_id_hot_spot_temperature.addItems(comboBox_id_hot_spot_temperature_list)
        self.comboBox_id_quality.addItems(comboBox_id_quality_list)
        self.comboBox_id_environment.addItems(comboBox_id_environment_list)

        self.inductor_hazard_rate_func()


    def inductor_hazard_rate_func(self):
        """
            Inductors Hazard Rate Function
        """
        cb_id_type = self.comboBox_id_type.currentText()
        cb_id_hot_spot_temperature = self.comboBox_id_hot_spot_temperature.currentText()
        cb_id_quality = self.comboBox_id_quality.currentText()
        cb_id_environment = self.comboBox_id_environment.currentText()

        self.lineEdit_id_lambda_b.clear()
        self.lineEdit_id_pi_t.clear()
        self.lineEdit_id_pi_q.clear()
        self.lineEdit_id_pi_e.clear()

        if  cb_id_type != "" and cb_id_hot_spot_temperature != "" and cb_id_quality != "" and cb_id_environment != "":
            if cb_id_type != "None":
                self.e_eq.id_lambda_b = float(self.electrical_equipments_dict["Inductor"]["Inductor Type"][cb_id_type])
                self.lineEdit_id_lambda_b.setText(str(self.e_eq.id_lambda_b))
            else:
                self.e_eq.id_lambda_b = 1.0

            if cb_id_hot_spot_temperature != "None":
                select_hot_spot_temperature = str("c_" + cb_id_hot_spot_temperature.replace('.', '_'))
                self.e_eq.id_pi_t_func(self.electrical_equipments_dict["Inductor"]["Temperature Factor"][select_hot_spot_temperature], 0)
                self.lineEdit_id_pi_t.setText(str(self.e_eq.get_id_pi_t()))
            else:
                self.e_eq.id_pi_t_func(0, 9)

            if cb_id_quality != "None":
                temp = cb_id_quality.replace(' - ', '_')
                select_quality = str(temp)
                self.e_eq.id_pi_q = float(self.electrical_equipments_dict["Inductor"]["Quality Factor"][select_quality])
                self.lineEdit_id_pi_q.setText(str(self.e_eq.id_pi_q))
            else:
                self.e_eq.id_pi_q = 1.0

            if cb_id_environment != "None":
                temp = cb_id_environment.replace(' - ', '_')
                select_environment = str(temp.lower())
                self.e_eq.id_pi_e = float(self.electrical_equipments_dict["Inductor"]["Environment Factor"][select_environment])
                self.lineEdit_id_pi_e.setText(str(self.e_eq.id_pi_e))
            else:
                self.e_eq.id_pi_e = 1.0

            self.e_eq.inductors_func()
            self.lineEdit_id_lamda_p.setText(str(self.e_eq.get_id_lambda()))


  # TRANSISTORS
    def transistor_tab_func(self):
        """
            Transistors Tab Function
        """
        self.comboBox_ts_type.clear()
        self.comboBox_ts_quality.clear()
        self.comboBox_ts_junction_temperature.clear()
        self.comboBox_ts_environment.clear()

        self.comboBox_ts_type.addItem("None")
        self.comboBox_ts_quality.addItem("None")
        self.comboBox_ts_junction_temperature.addItem("None")
        self.comboBox_ts_environment.addItem("None")

        comboBox_ts_type_list = list()
        comboBox_ts_quality_list = list()
        comboBox_ts_junction_temperature_list = list()
        comboBox_ts_environment_list = list()

        temp_ts_type_list = list(self.electrical_equipments_dict["Transistor"]["Transistor Type"].keys())
        temp_ts_quality_list = list(self.electrical_equipments_dict["Transistor"]["Quality Factor"].keys())
        temp_ts_junction_temperature_list = list(self.electrical_equipments_dict["Transistor"]["Temperature Factor"].keys())
        temp_ts_environment_list = list(self.electrical_equipments_dict["Transistor"]["Environment Factor"].keys())

        for item in temp_ts_junction_temperature_list:
            temp = str(item).split("c_")
            comboBox_ts_junction_temperature_list.append(int(temp[1].replace('_', '.')))

        for item in temp_ts_environment_list:
            temp = str(item).upper()
            comboBox_ts_environment_list.append(temp.replace('_', ' - '))

        comboBox_ts_type_list = temp_ts_type_list
        comboBox_ts_quality_list = temp_ts_quality_list

        comboBox_ts_type_list.sort()
        comboBox_ts_quality_list.sort()
        comboBox_ts_junction_temperature_list.sort()
        comboBox_ts_environment_list.sort()

        comboBox_ts_junction_temperature_list = map(str, comboBox_ts_junction_temperature_list)

        self.comboBox_ts_type.addItems(comboBox_ts_type_list)
        self.comboBox_ts_quality.addItems(comboBox_ts_quality_list)
        self.comboBox_ts_junction_temperature.addItems(comboBox_ts_junction_temperature_list)
        self.comboBox_ts_environment.addItems(comboBox_ts_environment_list)

        self.transistor_hazard_rate_func()


    def transistor_hazard_rate_func(self):
        """
            Transistors Hazard Rate Function
        """
        cb_ts_type = self.comboBox_ts_type.currentText()
        cb_ts_quality = self.comboBox_ts_quality.currentText()
        cb_ts_junction_temperature = self.comboBox_ts_junction_temperature.currentText()
        cb_ts_environment = self.comboBox_ts_environment.currentText()

        self.lineEdit_ts_lambda_b.clear()
        self.lineEdit_ts_pi_q.clear()
        self.lineEdit_ts_pi_t.clear()
        self.lineEdit_ts_pi_e.clear()


        if  cb_ts_type != "" and cb_ts_quality != "" and cb_ts_junction_temperature != "" and cb_ts_environment != "":
            if cb_ts_type != "None":
                self.e_eq.ts_lambda_b = float(self.electrical_equipments_dict["Transistor"]["Transistor Type"][cb_ts_type])
                self.lineEdit_ts_lambda_b.setText(str(self.e_eq.ts_lambda_b))
            else:
                self.e_eq.ts_lambda_b = 1.0

            if cb_ts_quality != "None":
                self.e_eq.ts_pi_q = float(self.electrical_equipments_dict["Transistor"]["Quality Factor"][cb_ts_quality])
                self.lineEdit_ts_pi_q.setText(str(self.e_eq.ts_pi_q))
            else:
                self.e_eq.ts_pi_q = 1.0

            if cb_ts_junction_temperature != "None":
                select_junction_temperature = str("c_" + cb_ts_junction_temperature.replace('.', '_'))
                self.e_eq.ts_pi_t_func(self.electrical_equipments_dict["Transistor"]["Temperature Factor"][select_junction_temperature], 0)
                self.lineEdit_ts_pi_t.setText(str(self.e_eq.get_ts_pi_t()))
            else:
                self.e_eq.ts_pi_t_func(0, 9)

            if cb_ts_environment != "None":
                temp = cb_ts_environment.replace(' - ', '_')
                select_environment = str(temp.lower())
                self.e_eq.ts_pi_e = float(self.electrical_equipments_dict["Transistor"]["Environment Factor"][select_environment])
                self.lineEdit_ts_pi_e.setText(str(self.e_eq.ts_pi_e))
            else:
                self.e_eq.ts_pi_e = 1.0

            self.e_eq.transistors_func()
            self.lineEdit_ts_lamda_p.setText(str(self.e_eq.get_ts_lambda()))


  # FUSES
    def fuse_tab_func(self):
        """
            Fuses Tab Function
        """
        self.comboBox_fs_type.clear()
        self.comboBox_fs_environment.clear()
        self.comboBox_fs_type.addItem("None")
        self.comboBox_fs_environment.addItem("None")

        comboBox_fs_type_list = list()
        comboBox_fs_environment_list = list()

        temp_fs_type_list = list(self.electrical_equipments_dict["Fuse"]["Base Failure Rate"].keys())
        temp_fs_environment_list = list(self.electrical_equipments_dict["Fuse"]["Environment Factor"].keys())

        for item in temp_fs_type_list:
            temp = str(item)
            comboBox_fs_type_list.append(temp.replace('_', ' - '))

        for item in temp_fs_environment_list:
            temp = str(item).upper()
            comboBox_fs_environment_list.append(temp.replace('_', ' - '))

        comboBox_fs_type_list.sort()
        comboBox_fs_environment_list.sort()

        self.comboBox_fs_type.addItems(comboBox_fs_type_list)
        self.comboBox_fs_environment.addItems(comboBox_fs_environment_list)

        self.fuse_hazard_rate_func()


    def fuse_hazard_rate_func(self):
        """
            Fuses Hazard Rate Function
        """
        cb_fs_type = self.comboBox_fs_type.currentText()
        cb_fs_environment = self.comboBox_fs_environment.currentText()
        self.lineEdit_fs_lambda_b.clear()
        self.lineEdit_fs_pi_e.clear()

        if  cb_fs_type != "" and cb_fs_environment != "":
            if cb_fs_type != "None":
                temp = cb_fs_type.replace(' - ', '_')
                select_type = str(temp)
                self.e_eq.fs_lambda_b = float(self.electrical_equipments_dict["Fuse"]["Base Failure Rate"][select_type])
                self.lineEdit_fs_lambda_b.setText(str(self.e_eq.fs_lambda_b))
            else:
                self.e_eq.fs_lambda_b = 1.0

            if cb_fs_environment != "None":
                temp = cb_fs_environment.replace(' - ', '_')
                select_environment = str(temp.lower())
                self.e_eq.fs_pi_e = float(self.electrical_equipments_dict["Fuse"]["Environment Factor"][select_environment])
                self.lineEdit_fs_pi_e.setText(str(self.e_eq.fs_pi_e))
            else:
                self.e_eq.fs_pi_e = 1.0

            self.e_eq.fuses_func()
            self.lineEdit_fs_lamda_p.setText(str(self.e_eq.get_fs_lambda()))

# "CONFIGURATION // ELECTRICAL EQUIPMENTS FUNCTIONS" - END -
