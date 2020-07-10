#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
    PHM Gui
"""

import sys
import os
import time
from threading import Thread, Event
from datetime import datetime
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QThread, pyqtSignal
from std_msgs.msg import String
import yaml
import rospy
from phm_reliability_calculation.class_reliability_calculation import ReliabilityCalculation
from phm_hazard_rate_calculation.class_failure_rate_calculation import FailureRateCalculation
from phm_robot_task_completion.class_robot_task_completion import RobotTaskCompletion, SimulationRobotTaskCompletion
from phm_robot_task_completion.class_potc_subscriber import POTCSubscriber
from phm_start.class_real_time_plot import DynamicPlot, ReliabilityPlot, POTCPlot
from phm_start.class_ros_sensor import ROSSensor
from phm_start.phm_graph_ui import Ui_StaticGraphWindow
from phm_start.phm_electrical_equipments_gui import ElectricalEquipmentsWindow
from phm_start.phm_mechanical_equipments_gui import MechanicalEquipmentsWindow
from phm_start.select_file_gui import SelectFileGui


class PHMGui(object):
    """
        PHM Gui Class
    """
    def __init__(self):
        self.main_dict = {"System": dict()}
        self.phm_gui_time = 1

        self.publisher_gui_hazard_rate = rospy.Publisher('/gui_hazard_rate', String, queue_size=10)
        self.publisher_gui_reliability = rospy.Publisher('/gui_reliability', String, queue_size=10)
        self.publisher_gui_actual_potc = rospy.Publisher('/gui_actual_potc', String, queue_size=10)
        self.publisher_gui_predict_potc = rospy.Publisher('/gui_predict_potc', String, queue_size=10)

        self.thread_rate = rospy.Rate(1)

        self.temperature_t0 = 25
        self.temperature_sensor_dict = dict({"System": {"Avarage": self.temperature_t0, "Sensors": dict()}})

        self.thread_component_sensor_control = False
        self.thread_sub_module_sensor_control = False
        self.thread_hazard_rate_sensor_based_control = False

        self.potc_main_dict = {"Actual": {"Nominal": {"POTC": list([1]), "Time": list([0])}, "Sensor Based": {"POTC": list([1]), "Time": list([0])}},
                               "Predict": {"Nominal": {"POTC": list([1]), "Time": list([0])}, "Sensor Based": {"POTC": list([1]), "Time": list([0])}}}

        self.prognostic_potc_x_list = list([0, 1, 2, 3])
        self.prognostic_potc_y_list = list([1.0, 1.0, 1.0, 1.0])

        self.potc_ros = POTCSubscriber()
        self.potc_ros.main_func()

        self.reliability_plot_scale_list = list([0, 1])

        self.equipment_list = ["Electrical Equipment", "Mechanical Equipment"]
        self.electrical_equipments_dict = dict(rospy.get_param("~Electrical Equipment"))
        self.mechanical_equipments_dict = dict(rospy.get_param("~Mechanical Equipment"))

        self.capacitor_style_dict = dict(rospy.get_param("~Capacitor Style"))
        self.diode_type_dict = dict(rospy.get_param("~Diode Type"))
        self.resistor_type_dict = dict(rospy.get_param("~Resistor Style"))

        self.shaft_sections_dict = dict()

        self.prognostic_potc_dict = dict()

        self.configuration_reliability_model = ["Exponential Distribution", "Rayleigh Distribution", "Weibull Distribution", "Curve Distribution"]
        self.configuration_reliability_unit = ["km/h", "m/s"]

        self.selected_reliability_model = "Exponential Distribution"
        self.selected_reliability_unit = True
        self.shape_parameter = 1

        self.r_calculation_class = ReliabilityCalculation()
        self.fr_calculation_class = FailureRateCalculation()

        self.type_list = ['Serial', 'Parallel']

# ------------------------------------------------------------------------------------------------

    def setupUi(self, MainWindow):
        """
            PHM Gui Setup Ui
        """
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1100, 900)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(MainWindow.sizePolicy().hasHeightForWidth())
        MainWindow.setSizePolicy(sizePolicy)
        MainWindow.setMinimumSize(QtCore.QSize(1100, 900))
        MainWindow.setMaximumSize(QtCore.QSize(1100, 900))
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.tabWidget_general = QtWidgets.QTabWidget(self.centralwidget)
        self.tabWidget_general.setGeometry(QtCore.QRect(0, 0, 1100, 900))
        self.tabWidget_general.setFocusPolicy(QtCore.Qt.NoFocus)
        self.tabWidget_general.setObjectName("tabWidget_general")
        self.tab_configuration = QtWidgets.QWidget()
        self.tab_configuration.setObjectName("tab_configuration")
        self.tabWidget_configuration = QtWidgets.QTabWidget(self.tab_configuration)
        self.tabWidget_configuration.setGeometry(QtCore.QRect(0, 0, 1100, 731))
        self.tabWidget_configuration.setObjectName("tabWidget_configuration")
        self.tab_add_object = QtWidgets.QWidget()
        self.tab_add_object.setObjectName("tab_add_object")
        self.tabWidget = QtWidgets.QTabWidget(self.tab_add_object)
        self.tabWidget.setGeometry(QtCore.QRect(0, 0, 1100, 720))
        self.tabWidget.setObjectName("tabWidget")
        self.tab_3 = QtWidgets.QWidget()
        self.tab_3.setObjectName("tab_3")
        self.groupBox_c_ao_am_add_module = QtWidgets.QGroupBox(self.tab_3)
        self.groupBox_c_ao_am_add_module.setGeometry(QtCore.QRect(50, 20, 421, 70))
        self.groupBox_c_ao_am_add_module.setAlignment(QtCore.Qt.AlignCenter)
        self.groupBox_c_ao_am_add_module.setFlat(False)
        self.groupBox_c_ao_am_add_module.setCheckable(False)
        self.groupBox_c_ao_am_add_module.setObjectName("groupBox_c_ao_am_add_module")
        self.lineEdit_c_ao_am_module_name = QtWidgets.QLineEdit(self.groupBox_c_ao_am_add_module)
        self.lineEdit_c_ao_am_module_name.setGeometry(QtCore.QRect(140, 25, 210, 27))
        self.lineEdit_c_ao_am_module_name.setObjectName("lineEdit_c_ao_am_module_name")
        self.label_12 = QtWidgets.QLabel(self.groupBox_c_ao_am_add_module)
        self.label_12.setGeometry(QtCore.QRect(0, 30, 131, 21))
        self.label_12.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_12.setObjectName("label_12")
        self.groupBox_c_ao_am_module_fr = QtWidgets.QGroupBox(self.tab_3)
        self.groupBox_c_ao_am_module_fr.setGeometry(QtCore.QRect(50, 110, 351, 191))
        self.groupBox_c_ao_am_module_fr.setFlat(False)
        self.groupBox_c_ao_am_module_fr.setCheckable(True)
        self.groupBox_c_ao_am_module_fr.setChecked(False)
        self.groupBox_c_ao_am_module_fr.setObjectName("groupBox_c_ao_am_module_fr")
        self.lineEdit_c_ao_am_module_fr = QtWidgets.QLineEdit(self.groupBox_c_ao_am_module_fr)
        self.lineEdit_c_ao_am_module_fr.setGeometry(QtCore.QRect(160, 30, 160, 27))
        self.lineEdit_c_ao_am_module_fr.setObjectName("lineEdit_c_ao_am_module_fr")
        self.label_13 = QtWidgets.QLabel(self.groupBox_c_ao_am_module_fr)
        self.label_13.setGeometry(QtCore.QRect(10, 30, 141, 21))
        self.label_13.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_13.setObjectName("label_13")
        self.label_42 = QtWidgets.QLabel(self.groupBox_c_ao_am_module_fr)
        self.label_42.setGeometry(QtCore.QRect(0, 110, 150, 31))
        self.label_42.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_42.setObjectName("label_42")
        self.label_43 = QtWidgets.QLabel(self.groupBox_c_ao_am_module_fr)
        self.label_43.setGeometry(QtCore.QRect(0, 150, 150, 21))
        self.label_43.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_43.setObjectName("label_43")
        self.comboBox_c_ao_am_select_type = QtWidgets.QComboBox(self.groupBox_c_ao_am_module_fr)
        self.comboBox_c_ao_am_select_type.setGeometry(QtCore.QRect(160, 110, 160, 27))
        self.comboBox_c_ao_am_select_type.setObjectName("comboBox_c_ao_am_select_type")
        self.label_44 = QtWidgets.QLabel(self.groupBox_c_ao_am_module_fr)
        self.label_44.setGeometry(QtCore.QRect(20, 70, 131, 31))
        self.label_44.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_44.setObjectName("label_44")
        self.spinBox_c_ao_am_quantity = QtWidgets.QSpinBox(self.groupBox_c_ao_am_module_fr)
        self.spinBox_c_ao_am_quantity.setGeometry(QtCore.QRect(160, 70, 160, 27))
        self.spinBox_c_ao_am_quantity.setMinimum(1)
        self.spinBox_c_ao_am_quantity.setMaximum(9999999)
        self.spinBox_c_ao_am_quantity.setObjectName("spinBox_c_ao_am_quantity")
        self.lineEdit_c_ao_am_total_module_fr = QtWidgets.QLineEdit(self.groupBox_c_ao_am_module_fr)
        self.lineEdit_c_ao_am_total_module_fr.setGeometry(QtCore.QRect(160, 150, 160, 27))
        self.lineEdit_c_ao_am_total_module_fr.setReadOnly(True)
        self.lineEdit_c_ao_am_total_module_fr.setObjectName("lineEdit_c_ao_am_total_module_fr")
        self.pushButton_c_ao_am_add_module = QtWidgets.QPushButton(self.tab_3)
        self.pushButton_c_ao_am_add_module.setGeometry(QtCore.QRect(480, 200, 125, 50))
        self.pushButton_c_ao_am_add_module.setObjectName("pushButton_c_ao_am_add_module")
        self.groupBox_c_ao_am_list_of_modules = QtWidgets.QGroupBox(self.tab_3)
        self.groupBox_c_ao_am_list_of_modules.setGeometry(QtCore.QRect(670, 10, 411, 291))
        self.groupBox_c_ao_am_list_of_modules.setObjectName("groupBox_c_ao_am_list_of_modules")
        self.listWidget_c_ao_am_list_of_modules = QtWidgets.QListWidget(self.groupBox_c_ao_am_list_of_modules)
        self.listWidget_c_ao_am_list_of_modules.setGeometry(QtCore.QRect(0, 30, 400, 200))
        self.listWidget_c_ao_am_list_of_modules.setObjectName("listWidget_c_ao_am_list_of_modules")
        self.pushButton_c_ao_am_delete_module = QtWidgets.QPushButton(self.groupBox_c_ao_am_list_of_modules)
        self.pushButton_c_ao_am_delete_module.setGeometry(QtCore.QRect(300, 240, 99, 27))
        self.pushButton_c_ao_am_delete_module.setObjectName("pushButton_c_ao_am_delete_module")
        self.line_4 = QtWidgets.QFrame(self.tab_3)
        self.line_4.setGeometry(QtCore.QRect(630, 0, 16, 661))
        self.line_4.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_4.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_4.setObjectName("line_4")
        self.tabWidget.addTab(self.tab_3, "")
        self.tab_5 = QtWidgets.QWidget()
        self.tab_5.setObjectName("tab_5")
        self.groupBox_c_ao_asm_sub_module_fr = QtWidgets.QGroupBox(self.tab_5)
        self.groupBox_c_ao_asm_sub_module_fr.setGeometry(QtCore.QRect(50, 90, 351, 181))
        self.groupBox_c_ao_asm_sub_module_fr.setFlat(False)
        self.groupBox_c_ao_asm_sub_module_fr.setCheckable(True)
        self.groupBox_c_ao_asm_sub_module_fr.setChecked(False)
        self.groupBox_c_ao_asm_sub_module_fr.setObjectName("groupBox_c_ao_asm_sub_module_fr")
        self.label_18 = QtWidgets.QLabel(self.groupBox_c_ao_asm_sub_module_fr)
        self.label_18.setGeometry(QtCore.QRect(0, 30, 171, 21))
        self.label_18.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_18.setObjectName("label_18")
        self.lineEdit_c_ao_asm_sub_module_fr = QtWidgets.QLineEdit(self.groupBox_c_ao_asm_sub_module_fr)
        self.lineEdit_c_ao_asm_sub_module_fr.setGeometry(QtCore.QRect(180, 30, 160, 27))
        self.lineEdit_c_ao_asm_sub_module_fr.setObjectName("lineEdit_c_ao_asm_sub_module_fr")
        self.label_36 = QtWidgets.QLabel(self.groupBox_c_ao_asm_sub_module_fr)
        self.label_36.setGeometry(QtCore.QRect(10, 70, 161, 31))
        self.label_36.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_36.setObjectName("label_36")
        self.comboBox_c_ao_asm_sfr_select_type = QtWidgets.QComboBox(self.groupBox_c_ao_asm_sub_module_fr)
        self.comboBox_c_ao_asm_sfr_select_type.setGeometry(QtCore.QRect(180, 110, 160, 27))
        self.comboBox_c_ao_asm_sfr_select_type.setObjectName("comboBox_c_ao_asm_sfr_select_type")
        self.label_40 = QtWidgets.QLabel(self.groupBox_c_ao_asm_sub_module_fr)
        self.label_40.setGeometry(QtCore.QRect(-10, 150, 181, 21))
        self.label_40.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_40.setObjectName("label_40")
        self.spinBox_c_ao_asm_sfr_quantity = QtWidgets.QSpinBox(self.groupBox_c_ao_asm_sub_module_fr)
        self.spinBox_c_ao_asm_sfr_quantity.setGeometry(QtCore.QRect(180, 70, 160, 27))
        self.spinBox_c_ao_asm_sfr_quantity.setMinimum(1)
        self.spinBox_c_ao_asm_sfr_quantity.setMaximum(9999999)
        self.spinBox_c_ao_asm_sfr_quantity.setObjectName("spinBox_c_ao_asm_sfr_quantity")
        self.lineEdit_c_ao_asm_sfr_total_component_fr = QtWidgets.QLineEdit(self.groupBox_c_ao_asm_sub_module_fr)
        self.lineEdit_c_ao_asm_sfr_total_component_fr.setGeometry(QtCore.QRect(180, 150, 160, 27))
        self.lineEdit_c_ao_asm_sfr_total_component_fr.setReadOnly(True)
        self.lineEdit_c_ao_asm_sfr_total_component_fr.setObjectName("lineEdit_c_ao_asm_sfr_total_component_fr")
        self.label_41 = QtWidgets.QLabel(self.groupBox_c_ao_asm_sub_module_fr)
        self.label_41.setGeometry(QtCore.QRect(10, 110, 161, 31))
        self.label_41.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_41.setObjectName("label_41")
        self.groupBox_c_ao_asm_default_fr = QtWidgets.QGroupBox(self.tab_5)
        self.groupBox_c_ao_asm_default_fr.setGeometry(QtCore.QRect(50, 310, 351, 301))
        font = QtGui.QFont()
        font.setBold(False)
        font.setItalic(False)
        font.setUnderline(False)
        font.setWeight(50)
        font.setStrikeOut(False)
        font.setKerning(True)
        font.setStyleStrategy(QtGui.QFont.PreferAntialias)
        self.groupBox_c_ao_asm_default_fr.setFont(font)
        self.groupBox_c_ao_asm_default_fr.setFocusPolicy(QtCore.Qt.NoFocus)
        self.groupBox_c_ao_asm_default_fr.setContextMenuPolicy(QtCore.Qt.NoContextMenu)
        self.groupBox_c_ao_asm_default_fr.setAlignment(QtCore.Qt.AlignCenter)
        self.groupBox_c_ao_asm_default_fr.setFlat(False)
        self.groupBox_c_ao_asm_default_fr.setCheckable(True)
        self.groupBox_c_ao_asm_default_fr.setChecked(False)
        self.groupBox_c_ao_asm_default_fr.setObjectName("groupBox_c_ao_asm_default_fr")
        self.label_19 = QtWidgets.QLabel(self.groupBox_c_ao_asm_default_fr)
        self.label_19.setGeometry(QtCore.QRect(10, 30, 161, 21))
        self.label_19.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_19.setObjectName("label_19")
        self.comboBox_c_ao_asm_select_equipment = QtWidgets.QComboBox(self.groupBox_c_ao_asm_default_fr)
        self.comboBox_c_ao_asm_select_equipment.setGeometry(QtCore.QRect(180, 30, 160, 27))
        self.comboBox_c_ao_asm_select_equipment.setObjectName("comboBox_c_ao_asm_select_equipment")
        self.label_20 = QtWidgets.QLabel(self.groupBox_c_ao_asm_default_fr)
        self.label_20.setGeometry(QtCore.QRect(10, 150, 161, 31))
        self.label_20.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_20.setObjectName("label_20")
        self.comboBox_c_ao_asm_select_type = QtWidgets.QComboBox(self.groupBox_c_ao_asm_default_fr)
        self.comboBox_c_ao_asm_select_type.setGeometry(QtCore.QRect(180, 150, 160, 27))
        self.comboBox_c_ao_asm_select_type.setObjectName("comboBox_c_ao_asm_select_type")
        self.lineEdit_c_ao_asm_component_fr = QtWidgets.QLineEdit(self.groupBox_c_ao_asm_default_fr)
        self.lineEdit_c_ao_asm_component_fr.setGeometry(QtCore.QRect(180, 70, 160, 27))
        self.lineEdit_c_ao_asm_component_fr.setReadOnly(True)
        self.lineEdit_c_ao_asm_component_fr.setObjectName("lineEdit_c_ao_asm_component_fr")
        self.label_21 = QtWidgets.QLabel(self.groupBox_c_ao_asm_default_fr)
        self.label_21.setGeometry(QtCore.QRect(-10, 190, 181, 21))
        self.label_21.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_21.setObjectName("label_21")
        self.label_30 = QtWidgets.QLabel(self.groupBox_c_ao_asm_default_fr)
        self.label_30.setGeometry(QtCore.QRect(10, 110, 161, 31))
        self.label_30.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_30.setObjectName("label_30")
        self.spinBox_c_ao_asm_quantity = QtWidgets.QSpinBox(self.groupBox_c_ao_asm_default_fr)
        self.spinBox_c_ao_asm_quantity.setGeometry(QtCore.QRect(180, 110, 160, 27))
        self.spinBox_c_ao_asm_quantity.setMinimum(1)
        self.spinBox_c_ao_asm_quantity.setMaximum(9999999)
        self.spinBox_c_ao_asm_quantity.setObjectName("spinBox_c_ao_asm_quantity")
        self.pushButton_c_ao_asm_set_equipment = QtWidgets.QPushButton(self.groupBox_c_ao_asm_default_fr)
        self.pushButton_c_ao_asm_set_equipment.setGeometry(QtCore.QRect(80, 240, 200, 50))
        self.pushButton_c_ao_asm_set_equipment.setObjectName("pushButton_c_ao_asm_set_equipment")
        self.lineEdit_c_ao_asm_total_component_fr = QtWidgets.QLineEdit(self.groupBox_c_ao_asm_default_fr)
        self.lineEdit_c_ao_asm_total_component_fr.setGeometry(QtCore.QRect(180, 190, 160, 27))
        self.lineEdit_c_ao_asm_total_component_fr.setReadOnly(True)
        self.lineEdit_c_ao_asm_total_component_fr.setObjectName("lineEdit_c_ao_asm_total_component_fr")
        self.label_33 = QtWidgets.QLabel(self.groupBox_c_ao_asm_default_fr)
        self.label_33.setGeometry(QtCore.QRect(10, 70, 161, 31))
        self.label_33.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_33.setWordWrap(True)
        self.label_33.setObjectName("label_33")
        self.groupBox_c_ao_asm_select_module = QtWidgets.QGroupBox(self.tab_5)
        self.groupBox_c_ao_asm_select_module.setGeometry(QtCore.QRect(20, 20, 331, 51))
        self.groupBox_c_ao_asm_select_module.setTitle("")
        self.groupBox_c_ao_asm_select_module.setAlignment(QtCore.Qt.AlignCenter)
        self.groupBox_c_ao_asm_select_module.setFlat(False)
        self.groupBox_c_ao_asm_select_module.setCheckable(False)
        self.groupBox_c_ao_asm_select_module.setObjectName("groupBox_c_ao_asm_select_module")
        self.comboBox_c_ao_asm_select_module = QtWidgets.QComboBox(self.groupBox_c_ao_asm_select_module)
        self.comboBox_c_ao_asm_select_module.setGeometry(QtCore.QRect(150, 10, 171, 27))
        self.comboBox_c_ao_asm_select_module.setCurrentText("")
        self.comboBox_c_ao_asm_select_module.setObjectName("comboBox_c_ao_asm_select_module")
        self.label_28 = QtWidgets.QLabel(self.groupBox_c_ao_asm_select_module)
        self.label_28.setGeometry(QtCore.QRect(0, 10, 141, 22))
        self.label_28.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_28.setObjectName("label_28")
        self.pushButton_c_ao_asm_add_sub_module = QtWidgets.QPushButton(self.tab_5)
        self.pushButton_c_ao_asm_add_sub_module.setGeometry(QtCore.QRect(480, 590, 125, 50))
        self.pushButton_c_ao_asm_add_sub_module.setObjectName("pushButton_c_ao_asm_add_sub_module")
        self.groupBox_c_ao_asm_set_sub_module_name = QtWidgets.QGroupBox(self.tab_5)
        self.groupBox_c_ao_asm_set_sub_module_name.setGeometry(QtCore.QRect(370, 10, 241, 60))
        self.groupBox_c_ao_asm_set_sub_module_name.setTitle("")
        self.groupBox_c_ao_asm_set_sub_module_name.setAlignment(QtCore.Qt.AlignCenter)
        self.groupBox_c_ao_asm_set_sub_module_name.setFlat(False)
        self.groupBox_c_ao_asm_set_sub_module_name.setCheckable(False)
        self.groupBox_c_ao_asm_set_sub_module_name.setObjectName("groupBox_c_ao_asm_set_sub_module_name")
        self.lineEdit_c_ao_asm_set_sub_module_name = QtWidgets.QLineEdit(self.groupBox_c_ao_asm_set_sub_module_name)
        self.lineEdit_c_ao_asm_set_sub_module_name.setGeometry(QtCore.QRect(9, 20, 210, 27))
        self.lineEdit_c_ao_asm_set_sub_module_name.setObjectName("lineEdit_c_ao_asm_set_sub_module_name")
        self.label_29 = QtWidgets.QLabel(self.groupBox_c_ao_asm_set_sub_module_name)
        self.label_29.setGeometry(QtCore.QRect(10, 0, 151, 22))
        self.label_29.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.label_29.setObjectName("label_29")
        self.groupBox_c_ao_asm_list_of_sub_modules = QtWidgets.QGroupBox(self.tab_5)
        self.groupBox_c_ao_asm_list_of_sub_modules.setGeometry(QtCore.QRect(670, 10, 401, 291))
        self.groupBox_c_ao_asm_list_of_sub_modules.setObjectName("groupBox_c_ao_asm_list_of_sub_modules")
        self.listWidget_c_ao_asm_list_of_sub_modules = QtWidgets.QListWidget(self.groupBox_c_ao_asm_list_of_sub_modules)
        self.listWidget_c_ao_asm_list_of_sub_modules.setGeometry(QtCore.QRect(0, 30, 400, 200))
        self.listWidget_c_ao_asm_list_of_sub_modules.setObjectName("listWidget_c_ao_asm_list_of_sub_modules")
        self.pushButton_c_ao_asm_delete_sub_module = QtWidgets.QPushButton(self.groupBox_c_ao_asm_list_of_sub_modules)
        self.pushButton_c_ao_asm_delete_sub_module.setGeometry(QtCore.QRect(300, 240, 99, 27))
        self.pushButton_c_ao_asm_delete_sub_module.setObjectName("pushButton_c_ao_asm_delete_sub_module")
        self.line_3 = QtWidgets.QFrame(self.tab_5)
        self.line_3.setGeometry(QtCore.QRect(630, 0, 16, 661))
        self.line_3.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_3.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_3.setObjectName("line_3")
        self.tabWidget.addTab(self.tab_5, "")
        self.tab_4 = QtWidgets.QWidget()
        self.tab_4.setObjectName("tab_4")
        self.tabWidget_5 = QtWidgets.QTabWidget(self.tab_4)
        self.tabWidget_5.setGeometry(QtCore.QRect(0, 0, 1100, 900))
        self.tabWidget_5.setObjectName("tabWidget_5")
        self.tab_10 = QtWidgets.QWidget()
        self.tab_10.setObjectName("tab_10")
        self.pushButton_c_ao_ac_add_component = QtWidgets.QPushButton(self.tab_10)
        self.pushButton_c_ao_ac_add_component.setGeometry(QtCore.QRect(480, 560, 125, 50))
        self.pushButton_c_ao_ac_add_component.setObjectName("pushButton_c_ao_ac_add_component")
        self.groupBox_c_ao_ac_select_sub_module = QtWidgets.QGroupBox(self.tab_10)
        self.groupBox_c_ao_ac_select_sub_module.setGeometry(QtCore.QRect(20, 20, 331, 81))
        self.groupBox_c_ao_ac_select_sub_module.setTitle("")
        self.groupBox_c_ao_ac_select_sub_module.setAlignment(QtCore.Qt.AlignCenter)
        self.groupBox_c_ao_ac_select_sub_module.setFlat(False)
        self.groupBox_c_ao_ac_select_sub_module.setCheckable(False)
        self.groupBox_c_ao_ac_select_sub_module.setObjectName("groupBox_c_ao_ac_select_sub_module")
        self.comboBox_c_ao_ac_select_sub_module = QtWidgets.QComboBox(self.groupBox_c_ao_ac_select_sub_module)
        self.comboBox_c_ao_ac_select_sub_module.setGeometry(QtCore.QRect(160, 40, 171, 27))
        self.comboBox_c_ao_ac_select_sub_module.setCurrentText("")
        self.comboBox_c_ao_ac_select_sub_module.setObjectName("comboBox_c_ao_ac_select_sub_module")
        self.comboBox_c_ao_ac_select_module = QtWidgets.QComboBox(self.groupBox_c_ao_ac_select_sub_module)
        self.comboBox_c_ao_ac_select_module.setGeometry(QtCore.QRect(160, 10, 171, 27))
        self.comboBox_c_ao_ac_select_module.setCurrentText("")
        self.comboBox_c_ao_ac_select_module.setObjectName("comboBox_c_ao_ac_select_module")
        self.label_9 = QtWidgets.QLabel(self.groupBox_c_ao_ac_select_sub_module)
        self.label_9.setGeometry(QtCore.QRect(10, 10, 141, 22))
        self.label_9.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_9.setObjectName("label_9")
        self.label_14 = QtWidgets.QLabel(self.groupBox_c_ao_ac_select_sub_module)
        self.label_14.setGeometry(QtCore.QRect(0, 40, 151, 22))
        self.label_14.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_14.setObjectName("label_14")
        self.groupBox_c_ao_ac_list_of_components = QtWidgets.QGroupBox(self.tab_10)
        self.groupBox_c_ao_ac_list_of_components.setGeometry(QtCore.QRect(670, 10, 401, 291))
        self.groupBox_c_ao_ac_list_of_components.setObjectName("groupBox_c_ao_ac_list_of_components")
        self.listWidget_c_ao_ac_list_of_components = QtWidgets.QListWidget(self.groupBox_c_ao_ac_list_of_components)
        self.listWidget_c_ao_ac_list_of_components.setGeometry(QtCore.QRect(0, 30, 400, 200))
        self.listWidget_c_ao_ac_list_of_components.setObjectName("listWidget_c_ao_ac_list_of_components")
        self.pushButton_c_ao_ac_delete_component = QtWidgets.QPushButton(self.groupBox_c_ao_ac_list_of_components)
        self.pushButton_c_ao_ac_delete_component.setGeometry(QtCore.QRect(300, 240, 99, 27))
        self.pushButton_c_ao_ac_delete_component.setObjectName("pushButton_c_ao_ac_delete_component")
        self.groupBox_c_ao_ac_default_fr = QtWidgets.QGroupBox(self.tab_10)
        self.groupBox_c_ao_ac_default_fr.setGeometry(QtCore.QRect(50, 310, 341, 251))
        font = QtGui.QFont()
        font.setBold(False)
        font.setItalic(False)
        font.setUnderline(False)
        font.setWeight(50)
        font.setStrikeOut(False)
        font.setKerning(True)
        font.setStyleStrategy(QtGui.QFont.PreferAntialias)
        self.groupBox_c_ao_ac_default_fr.setFont(font)
        self.groupBox_c_ao_ac_default_fr.setFocusPolicy(QtCore.Qt.NoFocus)
        self.groupBox_c_ao_ac_default_fr.setContextMenuPolicy(QtCore.Qt.NoContextMenu)
        self.groupBox_c_ao_ac_default_fr.setAlignment(QtCore.Qt.AlignCenter)
        self.groupBox_c_ao_ac_default_fr.setFlat(False)
        self.groupBox_c_ao_ac_default_fr.setCheckable(True)
        self.groupBox_c_ao_ac_default_fr.setChecked(False)
        self.groupBox_c_ao_ac_default_fr.setObjectName("groupBox_c_ao_ac_default_fr")
        self.label_7 = QtWidgets.QLabel(self.groupBox_c_ao_ac_default_fr)
        self.label_7.setGeometry(QtCore.QRect(10, 30, 161, 21))
        self.label_7.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.label_7.setObjectName("label_7")
        self.comboBox_c_ao_ac_select_equipment = QtWidgets.QComboBox(self.groupBox_c_ao_ac_default_fr)
        self.comboBox_c_ao_ac_select_equipment.setGeometry(QtCore.QRect(180, 30, 160, 27))
        self.comboBox_c_ao_ac_select_equipment.setObjectName("comboBox_c_ao_ac_select_equipment")
        self.label_8 = QtWidgets.QLabel(self.groupBox_c_ao_ac_default_fr)
        self.label_8.setGeometry(QtCore.QRect(10, 110, 161, 31))
        self.label_8.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_8.setObjectName("label_8")
        self.lineEdit_c_ao_ac_component_fr_2 = QtWidgets.QLineEdit(self.groupBox_c_ao_ac_default_fr)
        self.lineEdit_c_ao_ac_component_fr_2.setGeometry(QtCore.QRect(180, 70, 160, 27))
        self.lineEdit_c_ao_ac_component_fr_2.setReadOnly(True)
        self.lineEdit_c_ao_ac_component_fr_2.setObjectName("lineEdit_c_ao_ac_component_fr_2")
        self.label_11 = QtWidgets.QLabel(self.groupBox_c_ao_ac_default_fr)
        self.label_11.setGeometry(QtCore.QRect(-10, 70, 181, 21))
        self.label_11.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_11.setObjectName("label_11")
        self.spinBox_c_ao_ac_quantity = QtWidgets.QSpinBox(self.groupBox_c_ao_ac_default_fr)
        self.spinBox_c_ao_ac_quantity.setGeometry(QtCore.QRect(180, 110, 160, 27))
        self.spinBox_c_ao_ac_quantity.setMinimum(1)
        self.spinBox_c_ao_ac_quantity.setMaximum(9999999)
        self.spinBox_c_ao_ac_quantity.setObjectName("spinBox_c_ao_ac_quantity")
        self.pushButton_c_ao_ac_set_equipment = QtWidgets.QPushButton(self.groupBox_c_ao_ac_default_fr)
        self.pushButton_c_ao_ac_set_equipment.setGeometry(QtCore.QRect(80, 190, 200, 50))
        self.pushButton_c_ao_ac_set_equipment.setObjectName("pushButton_c_ao_ac_set_equipment")
        self.lineEdit_c_ao_ac_total_component_fr = QtWidgets.QLineEdit(self.groupBox_c_ao_ac_default_fr)
        self.lineEdit_c_ao_ac_total_component_fr.setGeometry(QtCore.QRect(180, 150, 160, 27))
        self.lineEdit_c_ao_ac_total_component_fr.setReadOnly(True)
        self.lineEdit_c_ao_ac_total_component_fr.setObjectName("lineEdit_c_ao_ac_total_component_fr")
        self.label_34 = QtWidgets.QLabel(self.groupBox_c_ao_ac_default_fr)
        self.label_34.setGeometry(QtCore.QRect(-10, 150, 181, 21))
        self.label_34.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_34.setObjectName("label_34")
        self.groupBox_c_ao_ac_set_component_name = QtWidgets.QGroupBox(self.tab_10)
        self.groupBox_c_ao_ac_set_component_name.setGeometry(QtCore.QRect(400, 20, 221, 60))
        self.groupBox_c_ao_ac_set_component_name.setTitle("")
        self.groupBox_c_ao_ac_set_component_name.setAlignment(QtCore.Qt.AlignCenter)
        self.groupBox_c_ao_ac_set_component_name.setFlat(False)
        self.groupBox_c_ao_ac_set_component_name.setCheckable(False)
        self.groupBox_c_ao_ac_set_component_name.setObjectName("groupBox_c_ao_ac_set_component_name")
        self.lineEdit_c_ao_ac_set_component_name = QtWidgets.QLineEdit(self.groupBox_c_ao_ac_set_component_name)
        self.lineEdit_c_ao_ac_set_component_name.setGeometry(QtCore.QRect(0, 20, 210, 27))
        self.lineEdit_c_ao_ac_set_component_name.setObjectName("lineEdit_c_ao_ac_set_component_name")
        self.label_17 = QtWidgets.QLabel(self.groupBox_c_ao_ac_set_component_name)
        self.label_17.setGeometry(QtCore.QRect(0, 0, 151, 22))
        self.label_17.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.label_17.setObjectName("label_17")
        self.groupBox_c_ao_ac_component_fr = QtWidgets.QGroupBox(self.tab_10)
        self.groupBox_c_ao_ac_component_fr.setGeometry(QtCore.QRect(50, 120, 351, 141))
        self.groupBox_c_ao_ac_component_fr.setAlignment(QtCore.Qt.AlignCenter)
        self.groupBox_c_ao_ac_component_fr.setFlat(False)
        self.groupBox_c_ao_ac_component_fr.setCheckable(True)
        self.groupBox_c_ao_ac_component_fr.setChecked(False)
        self.groupBox_c_ao_ac_component_fr.setObjectName("groupBox_c_ao_ac_component_fr")
        self.lineEdit_c_ao_ac_component_fr = QtWidgets.QLineEdit(self.groupBox_c_ao_ac_component_fr)
        self.lineEdit_c_ao_ac_component_fr.setGeometry(QtCore.QRect(180, 30, 160, 27))
        self.lineEdit_c_ao_ac_component_fr.setObjectName("lineEdit_c_ao_ac_component_fr")
        self.label_10 = QtWidgets.QLabel(self.groupBox_c_ao_ac_component_fr)
        self.label_10.setGeometry(QtCore.QRect(0, 35, 171, 21))
        self.label_10.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_10.setObjectName("label_10")
        self.label_31 = QtWidgets.QLabel(self.groupBox_c_ao_ac_component_fr)
        self.label_31.setGeometry(QtCore.QRect(10, 70, 161, 31))
        self.label_31.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_31.setObjectName("label_31")
        self.spinBox_c_ao_ac_sfr_quantity = QtWidgets.QSpinBox(self.groupBox_c_ao_ac_component_fr)
        self.spinBox_c_ao_ac_sfr_quantity.setGeometry(QtCore.QRect(180, 70, 160, 27))
        self.spinBox_c_ao_ac_sfr_quantity.setMinimum(1)
        self.spinBox_c_ao_ac_sfr_quantity.setMaximum(9999999)
        self.spinBox_c_ao_ac_sfr_quantity.setObjectName("spinBox_c_ao_ac_sfr_quantity")
        self.lineEdit_c_ao_ac_sfr_total_component_fr = QtWidgets.QLineEdit(self.groupBox_c_ao_ac_component_fr)
        self.lineEdit_c_ao_ac_sfr_total_component_fr.setGeometry(QtCore.QRect(180, 110, 160, 27))
        self.lineEdit_c_ao_ac_sfr_total_component_fr.setReadOnly(True)
        self.lineEdit_c_ao_ac_sfr_total_component_fr.setObjectName("lineEdit_c_ao_ac_sfr_total_component_fr")
        self.label_35 = QtWidgets.QLabel(self.groupBox_c_ao_ac_component_fr)
        self.label_35.setGeometry(QtCore.QRect(-10, 110, 181, 21))
        self.label_35.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_35.setObjectName("label_35")
        self.line = QtWidgets.QFrame(self.tab_10)
        self.line.setGeometry(QtCore.QRect(630, 0, 16, 631))
        self.line.setFrameShape(QtWidgets.QFrame.VLine)
        self.line.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line.setObjectName("line")
        self.tabWidget_5.addTab(self.tab_10, "")
        self.tab_11 = QtWidgets.QWidget()
        self.tab_11.setObjectName("tab_11")
        self.groupBox_c_ao_ace_select_module = QtWidgets.QGroupBox(self.tab_11)
        self.groupBox_c_ao_ace_select_module.setGeometry(QtCore.QRect(10, 20, 341, 111))
        self.groupBox_c_ao_ace_select_module.setTitle("")
        self.groupBox_c_ao_ace_select_module.setAlignment(QtCore.Qt.AlignCenter)
        self.groupBox_c_ao_ace_select_module.setFlat(False)
        self.groupBox_c_ao_ace_select_module.setCheckable(False)
        self.groupBox_c_ao_ace_select_module.setObjectName("groupBox_c_ao_ace_select_module")
        self.comboBox_c_ao_ace_select_module = QtWidgets.QComboBox(self.groupBox_c_ao_ace_select_module)
        self.comboBox_c_ao_ace_select_module.setGeometry(QtCore.QRect(170, 10, 171, 27))
        self.comboBox_c_ao_ace_select_module.setCurrentText("")
        self.comboBox_c_ao_ace_select_module.setObjectName("comboBox_c_ao_ace_select_module")
        self.label = QtWidgets.QLabel(self.groupBox_c_ao_ace_select_module)
        self.label.setGeometry(QtCore.QRect(20, 10, 141, 22))
        self.label.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label.setObjectName("label")
        self.comboBox_c_ao_ace_select_sub_module = QtWidgets.QComboBox(self.groupBox_c_ao_ace_select_module)
        self.comboBox_c_ao_ace_select_sub_module.setGeometry(QtCore.QRect(170, 40, 171, 27))
        self.comboBox_c_ao_ace_select_sub_module.setCurrentText("")
        self.comboBox_c_ao_ace_select_sub_module.setObjectName("comboBox_c_ao_ace_select_sub_module")
        self.label_2 = QtWidgets.QLabel(self.groupBox_c_ao_ace_select_module)
        self.label_2.setGeometry(QtCore.QRect(10, 40, 151, 22))
        self.label_2.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_2.setObjectName("label_2")
        self.groupBox_c_ao_ace_set_element_name = QtWidgets.QGroupBox(self.tab_11)
        self.groupBox_c_ao_ace_set_element_name.setGeometry(QtCore.QRect(390, 20, 241, 321))
        font = QtGui.QFont()
        font.setBold(False)
        font.setWeight(50)
        self.groupBox_c_ao_ace_set_element_name.setFont(font)
        self.groupBox_c_ao_ace_set_element_name.setTitle("")
        self.groupBox_c_ao_ace_set_element_name.setAlignment(QtCore.Qt.AlignCenter)
        self.groupBox_c_ao_ace_set_element_name.setFlat(False)
        self.groupBox_c_ao_ace_set_element_name.setCheckable(False)
        self.groupBox_c_ao_ace_set_element_name.setObjectName("groupBox_c_ao_ace_set_element_name")
        self.lineEdit_c_ao_ace_set_element_name = QtWidgets.QLineEdit(self.groupBox_c_ao_ace_set_element_name)
        self.lineEdit_c_ao_ace_set_element_name.setGeometry(QtCore.QRect(10, 80, 190, 27))
        self.lineEdit_c_ao_ace_set_element_name.setObjectName("lineEdit_c_ao_ace_set_element_name")
        self.label_4 = QtWidgets.QLabel(self.groupBox_c_ao_ace_set_element_name)
        self.label_4.setGeometry(QtCore.QRect(10, 60, 141, 22))
        self.label_4.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.label_4.setObjectName("label_4")
        self.lineEdit_c_ao_ace_set_element_variable_name = QtWidgets.QLineEdit(self.groupBox_c_ao_ace_set_element_name)
        self.lineEdit_c_ao_ace_set_element_variable_name.setGeometry(QtCore.QRect(10, 140, 190, 27))
        self.lineEdit_c_ao_ace_set_element_variable_name.setObjectName("lineEdit_c_ao_ace_set_element_variable_name")
        self.label_5 = QtWidgets.QLabel(self.groupBox_c_ao_ace_set_element_name)
        self.label_5.setGeometry(QtCore.QRect(10, 120, 231, 22))
        self.label_5.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.label_5.setObjectName("label_5")
        self.pushButton_c_ao_ace_add_component_element = QtWidgets.QPushButton(self.groupBox_c_ao_ace_set_element_name)
        self.pushButton_c_ao_ace_add_component_element.setGeometry(QtCore.QRect(40, 250, 125, 50))
        self.pushButton_c_ao_ace_add_component_element.setObjectName("pushButton_c_ao_ace_add_component_element")
        self.groupBox_c_ao_ace_set_element_value = QtWidgets.QGroupBox(self.groupBox_c_ao_ace_set_element_name)
        self.groupBox_c_ao_ace_set_element_value.setGeometry(QtCore.QRect(0, 180, 211, 60))
        self.groupBox_c_ao_ace_set_element_value.setAlignment(QtCore.Qt.AlignCenter)
        self.groupBox_c_ao_ace_set_element_value.setFlat(False)
        self.groupBox_c_ao_ace_set_element_value.setCheckable(False)
        self.groupBox_c_ao_ace_set_element_value.setChecked(False)
        self.groupBox_c_ao_ace_set_element_value.setObjectName("groupBox_c_ao_ace_set_element_value")
        self.lineEdit_c_ao_ace_set_element_value = QtWidgets.QLineEdit(self.groupBox_c_ao_ace_set_element_value)
        self.lineEdit_c_ao_ace_set_element_value.setEnabled(True)
        self.lineEdit_c_ao_ace_set_element_value.setGeometry(QtCore.QRect(10, 30, 190, 27))
        self.lineEdit_c_ao_ace_set_element_value.setObjectName("lineEdit_c_ao_ace_set_element_value")
        self.label_32 = QtWidgets.QLabel(self.groupBox_c_ao_ace_set_element_name)
        self.label_32.setGeometry(QtCore.QRect(10, 0, 151, 22))
        self.label_32.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.label_32.setObjectName("label_32")
        self.lineEdit_c_ao_ace_set_component_name = QtWidgets.QLineEdit(self.groupBox_c_ao_ace_set_element_name)
        self.lineEdit_c_ao_ace_set_component_name.setGeometry(QtCore.QRect(10, 20, 190, 27))
        self.lineEdit_c_ao_ace_set_component_name.setObjectName("lineEdit_c_ao_ace_set_component_name")
        self.groupBox_c_ao_ace_list_of_elements = QtWidgets.QGroupBox(self.tab_11)
        self.groupBox_c_ao_ace_list_of_elements.setGeometry(QtCore.QRect(670, 70, 411, 291))
        self.groupBox_c_ao_ace_list_of_elements.setObjectName("groupBox_c_ao_ace_list_of_elements")
        self.listWidget_c_ao_ace_list_of_elements = QtWidgets.QListWidget(self.groupBox_c_ao_ace_list_of_elements)
        self.listWidget_c_ao_ace_list_of_elements.setGeometry(QtCore.QRect(0, 30, 400, 200))
        self.listWidget_c_ao_ace_list_of_elements.setObjectName("listWidget_c_ao_ace_list_of_elements")
        self.pushButton_c_ao_ace_delete_element = QtWidgets.QPushButton(self.groupBox_c_ao_ace_list_of_elements)
        self.pushButton_c_ao_ace_delete_element.setGeometry(QtCore.QRect(300, 240, 99, 27))
        self.pushButton_c_ao_ace_delete_element.setObjectName("pushButton_c_ao_ace_delete_element")
        self.line_2 = QtWidgets.QFrame(self.tab_11)
        self.line_2.setGeometry(QtCore.QRect(630, 0, 16, 631))
        self.line_2.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_2.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_2.setObjectName("line_2")
        self.comboBox_c_ao_ace_select_component = QtWidgets.QComboBox(self.tab_11)
        self.comboBox_c_ao_ace_select_component.setGeometry(QtCore.QRect(810, 30, 171, 27))
        self.comboBox_c_ao_ace_select_component.setCurrentText("")
        self.comboBox_c_ao_ace_select_component.setObjectName("comboBox_c_ao_ace_select_component")
        self.label_3 = QtWidgets.QLabel(self.tab_11)
        self.label_3.setGeometry(QtCore.QRect(660, 30, 141, 22))
        self.label_3.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_3.setObjectName("label_3")
        self.line_5 = QtWidgets.QFrame(self.tab_11)
        self.line_5.setGeometry(QtCore.QRect(10, 370, 621, 16))
        self.line_5.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_5.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_5.setObjectName("line_5")
        self.groupBox_c_ao_ac_set_formula = QtWidgets.QGroupBox(self.tab_11)
        self.groupBox_c_ao_ac_set_formula.setGeometry(QtCore.QRect(20, 380, 611, 201))
        self.groupBox_c_ao_ac_set_formula.setTitle("")
        self.groupBox_c_ao_ac_set_formula.setObjectName("groupBox_c_ao_ac_set_formula")
        self.pushButton_c_ao_ac_set_formula = QtWidgets.QPushButton(self.groupBox_c_ao_ac_set_formula)
        self.pushButton_c_ao_ac_set_formula.setGeometry(QtCore.QRect(140, 130, 150, 50))
        self.pushButton_c_ao_ac_set_formula.setObjectName("pushButton_c_ao_ac_set_formula")
        self.plainTextEdit_c_ao_ac_set_formula = QtWidgets.QPlainTextEdit(self.groupBox_c_ao_ac_set_formula)
        self.plainTextEdit_c_ao_ac_set_formula.setGeometry(QtCore.QRect(10, 90, 421, 31))
        self.plainTextEdit_c_ao_ac_set_formula.setPlainText("")
        self.plainTextEdit_c_ao_ac_set_formula.setObjectName("plainTextEdit_c_ao_ac_set_formula")
        self.label_15 = QtWidgets.QLabel(self.groupBox_c_ao_ac_set_formula)
        self.label_15.setGeometry(QtCore.QRect(10, 60, 231, 22))
        self.label_15.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.label_15.setObjectName("label_15")
        self.lineEdit_c_ao_ace_component_fr = QtWidgets.QLineEdit(self.groupBox_c_ao_ac_set_formula)
        self.lineEdit_c_ao_ace_component_fr.setGeometry(QtCore.QRect(450, 90, 150, 31))
        self.lineEdit_c_ao_ace_component_fr.setReadOnly(True)
        self.lineEdit_c_ao_ace_component_fr.setObjectName("lineEdit_c_ao_ace_component_fr")
        self.label_16 = QtWidgets.QLabel(self.groupBox_c_ao_ac_set_formula)
        self.label_16.setGeometry(QtCore.QRect(450, 60, 141, 22))
        self.label_16.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.label_16.setObjectName("label_16")
        self.comboBox_c_ao_ace_select_component_2 = QtWidgets.QComboBox(self.groupBox_c_ao_ac_set_formula)
        self.comboBox_c_ao_ace_select_component_2.setGeometry(QtCore.QRect(150, 30, 171, 27))
        self.comboBox_c_ao_ace_select_component_2.setCurrentText("")
        self.comboBox_c_ao_ace_select_component_2.setObjectName("comboBox_c_ao_ace_select_component_2")
        self.label_6 = QtWidgets.QLabel(self.groupBox_c_ao_ac_set_formula)
        self.label_6.setGeometry(QtCore.QRect(0, 30, 141, 22))
        self.label_6.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_6.setObjectName("label_6")
        self.tabWidget_5.addTab(self.tab_11, "")
        self.tabWidget.addTab(self.tab_4, "")
        self.tabWidget_configuration.addTab(self.tab_add_object, "")
        self.tab_set_type = QtWidgets.QWidget()
        self.tab_set_type.setObjectName("tab_set_type")
        self.tabWidget_types = QtWidgets.QTabWidget(self.tab_set_type)
        self.tabWidget_types.setGeometry(QtCore.QRect(0, 0, 1100, 700))
        self.tabWidget_types.setFocusPolicy(QtCore.Qt.TabFocus)
        self.tabWidget_types.setObjectName("tabWidget_types")
        self.tab = QtWidgets.QWidget()
        self.tab.setObjectName("tab")
        self.label_configurations_4 = QtWidgets.QLabel(self.tab)
        self.label_configurations_4.setGeometry(QtCore.QRect(500, 240, 111, 17))
        self.label_configurations_4.setObjectName("label_configurations_4")
        self.label_configuration_select_first_comp = QtWidgets.QLabel(self.tab)
        self.label_configuration_select_first_comp.setGeometry(QtCore.QRect(490, 20, 171, 17))
        self.label_configuration_select_first_comp.setObjectName("label_configuration_select_first_comp")
        self.groupBox_result = QtWidgets.QGroupBox(self.tab)
        self.groupBox_result.setGeometry(QtCore.QRect(210, 340, 450, 80))
        self.groupBox_result.setTitle("")
        self.groupBox_result.setObjectName("groupBox_result")
        self.comboBox_configuration_select_second_comp = QtWidgets.QComboBox(self.tab)
        self.comboBox_configuration_select_second_comp.setGeometry(QtCore.QRect(780, 40, 161, 27))
        self.comboBox_configuration_select_second_comp.setObjectName("comboBox_configuration_select_second_comp")
        self.line_22 = QtWidgets.QFrame(self.tab)
        self.line_22.setGeometry(QtCore.QRect(360, 20, 20, 191))
        self.line_22.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_22.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_22.setObjectName("line_22")
        self.label_configuration_sub_module_list_3 = QtWidgets.QLabel(self.tab)
        self.label_configuration_sub_module_list_3.setGeometry(QtCore.QRect(70, 30, 111, 20))
        self.label_configuration_sub_module_list_3.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_configuration_sub_module_list_3.setObjectName("label_configuration_sub_module_list_3")
        self.groupBox_set_types_4 = QtWidgets.QGroupBox(self.tab)
        self.groupBox_set_types_4.setGeometry(QtCore.QRect(470, 110, 591, 41))
        self.groupBox_set_types_4.setTitle("")
        self.groupBox_set_types_4.setObjectName("groupBox_set_types_4")
        self.pushButton_configuration_set_comp_type = QtWidgets.QPushButton(self.groupBox_set_types_4)
        self.pushButton_configuration_set_comp_type.setGeometry(QtCore.QRect(480, 10, 99, 27))
        self.pushButton_configuration_set_comp_type.setObjectName("pushButton_configuration_set_comp_type")
        self.label_24 = QtWidgets.QLabel(self.groupBox_set_types_4)
        self.label_24.setGeometry(QtCore.QRect(187, 10, 16, 31))
        font = QtGui.QFont()
        font.setItalic(True)
        self.label_24.setFont(font)
        self.label_24.setObjectName("label_24")
        self.label_25 = QtWidgets.QLabel(self.groupBox_set_types_4)
        self.label_25.setGeometry(QtCore.QRect(290, 10, 16, 31))
        font = QtGui.QFont()
        font.setItalic(True)
        self.label_25.setFont(font)
        self.label_25.setObjectName("label_25")
        self.lineEdit_configuration_second_component = QtWidgets.QLineEdit(self.groupBox_set_types_4)
        self.lineEdit_configuration_second_component.setGeometry(QtCore.QRect(310, 10, 160, 27))
        self.lineEdit_configuration_second_component.setReadOnly(True)
        self.lineEdit_configuration_second_component.setObjectName("lineEdit_configuration_second_component")
        self.comboBox_configuration_type_of_component = QtWidgets.QComboBox(self.groupBox_set_types_4)
        self.comboBox_configuration_type_of_component.setGeometry(QtCore.QRect(205, 10, 80, 27))
        self.comboBox_configuration_type_of_component.setObjectName("comboBox_configuration_type_of_component")
        self.lineEdit_configuration_first_component = QtWidgets.QLineEdit(self.groupBox_set_types_4)
        self.lineEdit_configuration_first_component.setGeometry(QtCore.QRect(20, 10, 160, 27))
        self.lineEdit_configuration_first_component.setReadOnly(True)
        self.lineEdit_configuration_first_component.setObjectName("lineEdit_configuration_first_component")
        self.pushButton_delete_component_config = QtWidgets.QPushButton(self.tab)
        self.pushButton_delete_component_config.setGeometry(QtCore.QRect(800, 300, 101, 27))
        self.pushButton_delete_component_config.setAutoDefault(False)
        self.pushButton_delete_component_config.setDefault(False)
        self.pushButton_delete_component_config.setFlat(False)
        self.pushButton_delete_component_config.setObjectName("pushButton_delete_component_config")
        self.lineEdit_configuration_failure_rate_sub_module = QtWidgets.QLineEdit(self.tab)
        self.lineEdit_configuration_failure_rate_sub_module.setGeometry(QtCore.QRect(190, 119, 160, 27))
        self.lineEdit_configuration_failure_rate_sub_module.setReadOnly(True)
        self.lineEdit_configuration_failure_rate_sub_module.setObjectName("lineEdit_configuration_failure_rate_sub_module")
        self.listWidget_configurations_of_components = QtWidgets.QListWidget(self.tab)
        self.listWidget_configurations_of_components.setGeometry(QtCore.QRect(340, 260, 440, 371))
        self.listWidget_configurations_of_components.setDragDropMode(QtWidgets.QAbstractItemView.NoDragDrop)
        self.listWidget_configurations_of_components.setObjectName("listWidget_configurations_of_components")
        self.pushButton_add_component_config = QtWidgets.QPushButton(self.tab)
        self.pushButton_add_component_config.setGeometry(QtCore.QRect(800, 260, 99, 27))
        self.pushButton_add_component_config.setObjectName("pushButton_add_component_config")
        self.label_configuration_select_second_comp = QtWidgets.QLabel(self.tab)
        self.label_configuration_select_second_comp.setGeometry(QtCore.QRect(780, 20, 191, 17))
        self.label_configuration_select_second_comp.setObjectName("label_configuration_select_second_comp")
        self.comboBox_configuration_select_first_comp = QtWidgets.QComboBox(self.tab)
        self.comboBox_configuration_select_first_comp.setGeometry(QtCore.QRect(490, 40, 161, 27))
        self.comboBox_configuration_select_first_comp.setObjectName("comboBox_configuration_select_first_comp")
        self.pushButton_select_final_configuration_of_sub_module = QtWidgets.QPushButton(self.tab)
        self.pushButton_select_final_configuration_of_sub_module.setGeometry(QtCore.QRect(800, 600, 201, 27))
        self.pushButton_select_final_configuration_of_sub_module.setObjectName("pushButton_select_final_configuration_of_sub_module")
        self.lineEdit_configuration_failure_rate_first_component = QtWidgets.QLineEdit(self.tab)
        self.lineEdit_configuration_failure_rate_first_component.setGeometry(QtCore.QRect(490, 70, 160, 27))
        self.lineEdit_configuration_failure_rate_first_component.setReadOnly(True)
        self.lineEdit_configuration_failure_rate_first_component.setObjectName("lineEdit_configuration_failure_rate_first_component")
        self.label_configuration_failure_rate_7 = QtWidgets.QLabel(self.tab)
        self.label_configuration_failure_rate_7.setGeometry(QtCore.QRect(400, 70, 91, 20))
        self.label_configuration_failure_rate_7.setObjectName("label_configuration_failure_rate_7")
        self.label_configuration_sub_module_list_2 = QtWidgets.QLabel(self.tab)
        self.label_configuration_sub_module_list_2.setGeometry(QtCore.QRect(70, 59, 111, 20))
        self.label_configuration_sub_module_list_2.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_configuration_sub_module_list_2.setObjectName("label_configuration_sub_module_list_2")
        self.listWidget_components = QtWidgets.QListWidget(self.tab)
        self.listWidget_components.setGeometry(QtCore.QRect(50, 260, 256, 371))
        self.listWidget_components.setObjectName("listWidget_components")
        self.label_configuration_sub_module_list = QtWidgets.QLabel(self.tab)
        self.label_configuration_sub_module_list.setGeometry(QtCore.QRect(50, 89, 131, 20))
        self.label_configuration_sub_module_list.setObjectName("label_configuration_sub_module_list")
        self.comboBox_configuration_st_sc_select_system = QtWidgets.QComboBox(self.tab)
        self.comboBox_configuration_st_sc_select_system.setGeometry(QtCore.QRect(190, 30, 161, 27))
        self.comboBox_configuration_st_sc_select_system.setObjectName("comboBox_configuration_st_sc_select_system")
        self.label_components = QtWidgets.QLabel(self.tab)
        self.label_components.setGeometry(QtCore.QRect(120, 240, 91, 17))
        self.label_components.setObjectName("label_components")
        self.comboBox_configuration_st_sc_select_sub_module = QtWidgets.QComboBox(self.tab)
        self.comboBox_configuration_st_sc_select_sub_module.setGeometry(QtCore.QRect(190, 89, 161, 27))
        self.comboBox_configuration_st_sc_select_sub_module.setObjectName("comboBox_configuration_st_sc_select_sub_module")
        self.comboBox_configuration_st_sc_select_module = QtWidgets.QComboBox(self.tab)
        self.comboBox_configuration_st_sc_select_module.setGeometry(QtCore.QRect(190, 59, 161, 27))
        self.comboBox_configuration_st_sc_select_module.setObjectName("comboBox_configuration_st_sc_select_module")
        self.label_configuration_failure_rate_8 = QtWidgets.QLabel(self.tab)
        self.label_configuration_failure_rate_8.setGeometry(QtCore.QRect(20, 120, 171, 20))
        self.label_configuration_failure_rate_8.setObjectName("label_configuration_failure_rate_8")
        self.lineEdit_configuration_failure_rate_second_component = QtWidgets.QLineEdit(self.tab)
        self.lineEdit_configuration_failure_rate_second_component.setGeometry(QtCore.QRect(780, 70, 160, 27))
        self.lineEdit_configuration_failure_rate_second_component.setReadOnly(True)
        self.lineEdit_configuration_failure_rate_second_component.setObjectName("lineEdit_configuration_failure_rate_second_component")
        self.tabWidget_types.addTab(self.tab, "")
        self.tab_2 = QtWidgets.QWidget()
        self.tab_2.setObjectName("tab_2")
        self.pushButton_delete_sub_module_config = QtWidgets.QPushButton(self.tab_2)
        self.pushButton_delete_sub_module_config.setGeometry(QtCore.QRect(800, 300, 101, 27))
        self.pushButton_delete_sub_module_config.setAutoDefault(False)
        self.pushButton_delete_sub_module_config.setDefault(False)
        self.pushButton_delete_sub_module_config.setFlat(False)
        self.pushButton_delete_sub_module_config.setObjectName("pushButton_delete_sub_module_config")
        self.pushButton_add_sub_module_config = QtWidgets.QPushButton(self.tab_2)
        self.pushButton_add_sub_module_config.setGeometry(QtCore.QRect(800, 260, 99, 27))
        self.pushButton_add_sub_module_config.setObjectName("pushButton_add_sub_module_config")
        self.lineEdit_configuration_failure_rate_module = QtWidgets.QLineEdit(self.tab_2)
        self.lineEdit_configuration_failure_rate_module.setGeometry(QtCore.QRect(190, 89, 160, 27))
        self.lineEdit_configuration_failure_rate_module.setReadOnly(True)
        self.lineEdit_configuration_failure_rate_module.setObjectName("lineEdit_configuration_failure_rate_module")
        self.lineEdit_configuration_failure_rate_second_sub_module = QtWidgets.QLineEdit(self.tab_2)
        self.lineEdit_configuration_failure_rate_second_sub_module.setGeometry(QtCore.QRect(780, 70, 160, 27))
        self.lineEdit_configuration_failure_rate_second_sub_module.setReadOnly(True)
        self.lineEdit_configuration_failure_rate_second_sub_module.setObjectName("lineEdit_configuration_failure_rate_second_sub_module")
        self.groupBox_set_types_5 = QtWidgets.QGroupBox(self.tab_2)
        self.groupBox_set_types_5.setGeometry(QtCore.QRect(480, 110, 591, 41))
        self.groupBox_set_types_5.setTitle("")
        self.groupBox_set_types_5.setObjectName("groupBox_set_types_5")
        self.pushButton_configuration_set_sub_module_type = QtWidgets.QPushButton(self.groupBox_set_types_5)
        self.pushButton_configuration_set_sub_module_type.setGeometry(QtCore.QRect(470, 10, 99, 27))
        self.pushButton_configuration_set_sub_module_type.setObjectName("pushButton_configuration_set_sub_module_type")
        self.label_26 = QtWidgets.QLabel(self.groupBox_set_types_5)
        self.label_26.setGeometry(QtCore.QRect(177, 10, 16, 31))
        font = QtGui.QFont()
        font.setItalic(True)
        self.label_26.setFont(font)
        self.label_26.setObjectName("label_26")
        self.label_27 = QtWidgets.QLabel(self.groupBox_set_types_5)
        self.label_27.setGeometry(QtCore.QRect(280, 10, 16, 31))
        font = QtGui.QFont()
        font.setItalic(True)
        self.label_27.setFont(font)
        self.label_27.setObjectName("label_27")
        self.lineEdit_configuration_second_sub_module = QtWidgets.QLineEdit(self.groupBox_set_types_5)
        self.lineEdit_configuration_second_sub_module.setGeometry(QtCore.QRect(300, 10, 160, 27))
        self.lineEdit_configuration_second_sub_module.setReadOnly(True)
        self.lineEdit_configuration_second_sub_module.setObjectName("lineEdit_configuration_second_sub_module")
        self.comboBox_configuration_type_of_sub_module = QtWidgets.QComboBox(self.groupBox_set_types_5)
        self.comboBox_configuration_type_of_sub_module.setGeometry(QtCore.QRect(195, 10, 80, 27))
        self.comboBox_configuration_type_of_sub_module.setObjectName("comboBox_configuration_type_of_sub_module")
        self.lineEdit_configuration_first_sub_module = QtWidgets.QLineEdit(self.groupBox_set_types_5)
        self.lineEdit_configuration_first_sub_module.setGeometry(QtCore.QRect(10, 10, 160, 27))
        self.lineEdit_configuration_first_sub_module.setReadOnly(True)
        self.lineEdit_configuration_first_sub_module.setObjectName("lineEdit_configuration_first_sub_module")
        self.comboBox_configuration_st_ssm_select_module = QtWidgets.QComboBox(self.tab_2)
        self.comboBox_configuration_st_ssm_select_module.setGeometry(QtCore.QRect(190, 59, 161, 27))
        self.comboBox_configuration_st_ssm_select_module.setObjectName("comboBox_configuration_st_ssm_select_module")
        self.label_configuration_failure_rate_9 = QtWidgets.QLabel(self.tab_2)
        self.label_configuration_failure_rate_9.setGeometry(QtCore.QRect(50, 90, 141, 20))
        self.label_configuration_failure_rate_9.setObjectName("label_configuration_failure_rate_9")
        self.comboBox_configuration_select_first_sub_module = QtWidgets.QComboBox(self.tab_2)
        self.comboBox_configuration_select_first_sub_module.setGeometry(QtCore.QRect(490, 40, 161, 27))
        self.comboBox_configuration_select_first_sub_module.setObjectName("comboBox_configuration_select_first_sub_module")
        self.comboBox_configuration_st_ssm_select_system = QtWidgets.QComboBox(self.tab_2)
        self.comboBox_configuration_st_ssm_select_system.setGeometry(QtCore.QRect(190, 30, 161, 27))
        self.comboBox_configuration_st_ssm_select_system.setObjectName("comboBox_configuration_st_ssm_select_system")
        self.lineEdit_configuration_failure_rate_first_sub_module = QtWidgets.QLineEdit(self.tab_2)
        self.lineEdit_configuration_failure_rate_first_sub_module.setGeometry(QtCore.QRect(490, 70, 160, 27))
        self.lineEdit_configuration_failure_rate_first_sub_module.setReadOnly(True)
        self.lineEdit_configuration_failure_rate_first_sub_module.setObjectName("lineEdit_configuration_failure_rate_first_sub_module")
        self.label_configuration_select_first_sub_module = QtWidgets.QLabel(self.tab_2)
        self.label_configuration_select_first_sub_module.setGeometry(QtCore.QRect(490, 20, 171, 17))
        self.label_configuration_select_first_sub_module.setObjectName("label_configuration_select_first_sub_module")
        self.label_configuration_select_second_sub_module = QtWidgets.QLabel(self.tab_2)
        self.label_configuration_select_second_sub_module.setGeometry(QtCore.QRect(780, 20, 191, 17))
        self.label_configuration_select_second_sub_module.setObjectName("label_configuration_select_second_sub_module")
        self.listWidget_configurations_of_sub_modules = QtWidgets.QListWidget(self.tab_2)
        self.listWidget_configurations_of_sub_modules.setGeometry(QtCore.QRect(340, 260, 440, 371))
        self.listWidget_configurations_of_sub_modules.setDragDropMode(QtWidgets.QAbstractItemView.NoDragDrop)
        self.listWidget_configurations_of_sub_modules.setObjectName("listWidget_configurations_of_sub_modules")
        self.listWidget_sub_modules = QtWidgets.QListWidget(self.tab_2)
        self.listWidget_sub_modules.setGeometry(QtCore.QRect(50, 260, 256, 371))
        self.listWidget_sub_modules.setObjectName("listWidget_sub_modules")
        self.label_configuration_failure_rate_10 = QtWidgets.QLabel(self.tab_2)
        self.label_configuration_failure_rate_10.setGeometry(QtCore.QRect(400, 70, 91, 20))
        self.label_configuration_failure_rate_10.setObjectName("label_configuration_failure_rate_10")
        self.label_configuration_sub_module_list_4 = QtWidgets.QLabel(self.tab_2)
        self.label_configuration_sub_module_list_4.setGeometry(QtCore.QRect(70, 30, 111, 20))
        self.label_configuration_sub_module_list_4.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_configuration_sub_module_list_4.setObjectName("label_configuration_sub_module_list_4")
        self.label_configuration_module_list = QtWidgets.QLabel(self.tab_2)
        self.label_configuration_module_list.setGeometry(QtCore.QRect(80, 59, 101, 20))
        self.label_configuration_module_list.setObjectName("label_configuration_module_list")
        self.pushButton_select_final_configuration_of_module = QtWidgets.QPushButton(self.tab_2)
        self.pushButton_select_final_configuration_of_module.setGeometry(QtCore.QRect(800, 600, 201, 27))
        self.pushButton_select_final_configuration_of_module.setObjectName("pushButton_select_final_configuration_of_module")
        self.label_configurations_5 = QtWidgets.QLabel(self.tab_2)
        self.label_configurations_5.setGeometry(QtCore.QRect(500, 240, 111, 17))
        self.label_configurations_5.setObjectName("label_configurations_5")
        self.comboBox_configuration_select_second_sub_module = QtWidgets.QComboBox(self.tab_2)
        self.comboBox_configuration_select_second_sub_module.setGeometry(QtCore.QRect(780, 40, 161, 27))
        self.comboBox_configuration_select_second_sub_module.setObjectName("comboBox_configuration_select_second_sub_module")
        self.label_sub_module = QtWidgets.QLabel(self.tab_2)
        self.label_sub_module.setGeometry(QtCore.QRect(120, 240, 91, 17))
        self.label_sub_module.setObjectName("label_sub_module")
        self.line_32 = QtWidgets.QFrame(self.tab_2)
        self.line_32.setGeometry(QtCore.QRect(360, 20, 20, 191))
        self.line_32.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_32.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_32.setObjectName("line_32")
        self.tabWidget_types.addTab(self.tab_2, "")
        self.tab_set_module_types_2 = QtWidgets.QWidget()
        self.tab_set_module_types_2.setObjectName("tab_set_module_types_2")
        self.label_configurations_3 = QtWidgets.QLabel(self.tab_set_module_types_2)
        self.label_configurations_3.setGeometry(QtCore.QRect(500, 240, 111, 17))
        self.label_configurations_3.setObjectName("label_configurations_3")
        self.listWidget_configurations_of_modules = QtWidgets.QListWidget(self.tab_set_module_types_2)
        self.listWidget_configurations_of_modules.setGeometry(QtCore.QRect(340, 260, 440, 371))
        self.listWidget_configurations_of_modules.setDragDropMode(QtWidgets.QAbstractItemView.NoDragDrop)
        self.listWidget_configurations_of_modules.setObjectName("listWidget_configurations_of_modules")
        self.label_configuration_system_list = QtWidgets.QLabel(self.tab_set_module_types_2)
        self.label_configuration_system_list.setGeometry(QtCore.QRect(80, 30, 101, 20))
        self.label_configuration_system_list.setObjectName("label_configuration_system_list")
        self.listWidget_modules = QtWidgets.QListWidget(self.tab_set_module_types_2)
        self.listWidget_modules.setGeometry(QtCore.QRect(50, 260, 256, 371))
        self.listWidget_modules.setObjectName("listWidget_modules")
        self.label_modules = QtWidgets.QLabel(self.tab_set_module_types_2)
        self.label_modules.setGeometry(QtCore.QRect(140, 240, 61, 17))
        self.label_modules.setObjectName("label_modules")
        self.lineEdit_configuration_failure_rate_second_module = QtWidgets.QLineEdit(self.tab_set_module_types_2)
        self.lineEdit_configuration_failure_rate_second_module.setGeometry(QtCore.QRect(780, 70, 160, 27))
        self.lineEdit_configuration_failure_rate_second_module.setReadOnly(True)
        self.lineEdit_configuration_failure_rate_second_module.setObjectName("lineEdit_configuration_failure_rate_second_module")
        self.lineEdit_configuration_failure_rate_first_module = QtWidgets.QLineEdit(self.tab_set_module_types_2)
        self.lineEdit_configuration_failure_rate_first_module.setGeometry(QtCore.QRect(490, 70, 160, 27))
        self.lineEdit_configuration_failure_rate_first_module.setReadOnly(True)
        self.lineEdit_configuration_failure_rate_first_module.setObjectName("lineEdit_configuration_failure_rate_first_module")
        self.pushButton_delete_module_config = QtWidgets.QPushButton(self.tab_set_module_types_2)
        self.pushButton_delete_module_config.setGeometry(QtCore.QRect(800, 300, 101, 27))
        self.pushButton_delete_module_config.setAutoDefault(False)
        self.pushButton_delete_module_config.setDefault(False)
        self.pushButton_delete_module_config.setFlat(False)
        self.pushButton_delete_module_config.setObjectName("pushButton_delete_module_config")
        self.pushButton_add_module_config = QtWidgets.QPushButton(self.tab_set_module_types_2)
        self.pushButton_add_module_config.setGeometry(QtCore.QRect(800, 260, 99, 27))
        self.pushButton_add_module_config.setObjectName("pushButton_add_module_config")
        self.comboBox_configuration_st_sm_select_system = QtWidgets.QComboBox(self.tab_set_module_types_2)
        self.comboBox_configuration_st_sm_select_system.setGeometry(QtCore.QRect(190, 30, 161, 27))
        self.comboBox_configuration_st_sm_select_system.setObjectName("comboBox_configuration_st_sm_select_system")
        self.comboBox_configuration_select_second_module = QtWidgets.QComboBox(self.tab_set_module_types_2)
        self.comboBox_configuration_select_second_module.setGeometry(QtCore.QRect(780, 40, 161, 27))
        self.comboBox_configuration_select_second_module.setObjectName("comboBox_configuration_select_second_module")
        self.groupBox_set_types_3 = QtWidgets.QGroupBox(self.tab_set_module_types_2)
        self.groupBox_set_types_3.setGeometry(QtCore.QRect(480, 110, 591, 41))
        self.groupBox_set_types_3.setTitle("")
        self.groupBox_set_types_3.setObjectName("groupBox_set_types_3")
        self.pushButton_configuration_set_module_type = QtWidgets.QPushButton(self.groupBox_set_types_3)
        self.pushButton_configuration_set_module_type.setGeometry(QtCore.QRect(470, 10, 99, 27))
        self.pushButton_configuration_set_module_type.setObjectName("pushButton_configuration_set_module_type")
        self.label_22 = QtWidgets.QLabel(self.groupBox_set_types_3)
        self.label_22.setGeometry(QtCore.QRect(177, 10, 16, 31))
        font = QtGui.QFont()
        font.setItalic(True)
        self.label_22.setFont(font)
        self.label_22.setObjectName("label_22")
        self.label_23 = QtWidgets.QLabel(self.groupBox_set_types_3)
        self.label_23.setGeometry(QtCore.QRect(280, 10, 16, 31))
        font = QtGui.QFont()
        font.setItalic(True)
        self.label_23.setFont(font)
        self.label_23.setObjectName("label_23")
        self.lineEdit_configuration_second_module = QtWidgets.QLineEdit(self.groupBox_set_types_3)
        self.lineEdit_configuration_second_module.setGeometry(QtCore.QRect(300, 10, 160, 27))
        self.lineEdit_configuration_second_module.setReadOnly(True)
        self.lineEdit_configuration_second_module.setObjectName("lineEdit_configuration_second_module")
        self.comboBox_configuration_type_of_module = QtWidgets.QComboBox(self.groupBox_set_types_3)
        self.comboBox_configuration_type_of_module.setGeometry(QtCore.QRect(195, 10, 80, 27))
        self.comboBox_configuration_type_of_module.setObjectName("comboBox_configuration_type_of_module")
        self.lineEdit_configuration_first_module = QtWidgets.QLineEdit(self.groupBox_set_types_3)
        self.lineEdit_configuration_first_module.setGeometry(QtCore.QRect(10, 10, 160, 27))
        self.lineEdit_configuration_first_module.setReadOnly(True)
        self.lineEdit_configuration_first_module.setObjectName("lineEdit_configuration_first_module")
        self.label_configuration_select_second_module = QtWidgets.QLabel(self.tab_set_module_types_2)
        self.label_configuration_select_second_module.setGeometry(QtCore.QRect(780, 20, 191, 17))
        self.label_configuration_select_second_module.setObjectName("label_configuration_select_second_module")
        self.comboBox_configuration_select_first_module = QtWidgets.QComboBox(self.tab_set_module_types_2)
        self.comboBox_configuration_select_first_module.setGeometry(QtCore.QRect(490, 40, 161, 27))
        self.comboBox_configuration_select_first_module.setObjectName("comboBox_configuration_select_first_module")
        self.label_configuration_select_first_module = QtWidgets.QLabel(self.tab_set_module_types_2)
        self.label_configuration_select_first_module.setGeometry(QtCore.QRect(490, 20, 171, 17))
        self.label_configuration_select_first_module.setObjectName("label_configuration_select_first_module")
        self.label_configuration_failure_rate_5 = QtWidgets.QLabel(self.tab_set_module_types_2)
        self.label_configuration_failure_rate_5.setGeometry(QtCore.QRect(400, 70, 91, 21))
        self.label_configuration_failure_rate_5.setObjectName("label_configuration_failure_rate_5")
        self.line_21 = QtWidgets.QFrame(self.tab_set_module_types_2)
        self.line_21.setGeometry(QtCore.QRect(360, 20, 20, 191))
        self.line_21.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_21.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_21.setObjectName("line_21")
        self.lineEdit_configuration_failure_rate_system = QtWidgets.QLineEdit(self.tab_set_module_types_2)
        self.lineEdit_configuration_failure_rate_system.setGeometry(QtCore.QRect(190, 60, 160, 27))
        self.lineEdit_configuration_failure_rate_system.setReadOnly(True)
        self.lineEdit_configuration_failure_rate_system.setObjectName("lineEdit_configuration_failure_rate_system")
        self.label_configuration_system_failure_rate = QtWidgets.QLabel(self.tab_set_module_types_2)
        self.label_configuration_system_failure_rate.setGeometry(QtCore.QRect(50, 61, 141, 20))
        self.label_configuration_system_failure_rate.setObjectName("label_configuration_system_failure_rate")
        self.pushButton_select_final_configuration_of_system = QtWidgets.QPushButton(self.tab_set_module_types_2)
        self.pushButton_select_final_configuration_of_system.setGeometry(QtCore.QRect(800, 600, 201, 27))
        self.pushButton_select_final_configuration_of_system.setObjectName("pushButton_select_final_configuration_of_system")
        self.comboBox_configuration_reliability_model = QtWidgets.QComboBox(self.tab_set_module_types_2)
        self.comboBox_configuration_reliability_model.setGeometry(QtCore.QRect(190, 120, 160, 27))
        self.comboBox_configuration_reliability_model.setObjectName("comboBox_configuration_reliability_model")
        self.label_configuration_reliability_type = QtWidgets.QLabel(self.tab_set_module_types_2)
        self.label_configuration_reliability_type.setGeometry(QtCore.QRect(0, 120, 181, 20))
        self.label_configuration_reliability_type.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_configuration_reliability_type.setWordWrap(False)
        self.label_configuration_reliability_type.setObjectName("label_configuration_reliability_type")
        self.comboBox_configuration_unit = QtWidgets.QComboBox(self.tab_set_module_types_2)
        self.comboBox_configuration_unit.setGeometry(QtCore.QRect(190, 180, 160, 27))
        self.comboBox_configuration_unit.setObjectName("comboBox_configuration_unit")
        self.label_configuration_unit = QtWidgets.QLabel(self.tab_set_module_types_2)
        self.label_configuration_unit.setGeometry(QtCore.QRect(80, 180, 100, 20))
        self.label_configuration_unit.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_configuration_unit.setWordWrap(True)
        self.label_configuration_unit.setObjectName("label_configuration_unit")
        self.lineEdit_configuration_reliability_shape_parameter = QtWidgets.QLineEdit(self.tab_set_module_types_2)
        self.lineEdit_configuration_reliability_shape_parameter.setGeometry(QtCore.QRect(190, 150, 75, 27))
        self.lineEdit_configuration_reliability_shape_parameter.setReadOnly(False)
        self.lineEdit_configuration_reliability_shape_parameter.setObjectName("lineEdit_configuration_reliability_shape_parameter")
        self.label_configuration_shape_parameter = QtWidgets.QLabel(self.tab_set_module_types_2)
        self.label_configuration_shape_parameter.setGeometry(QtCore.QRect(47, 150, 130, 20))
        self.label_configuration_shape_parameter.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_configuration_shape_parameter.setWordWrap(True)
        self.label_configuration_shape_parameter.setObjectName("label_configuration_shape_parameter")
        self.tabWidget_types.addTab(self.tab_set_module_types_2, "")
        self.tabWidget_configuration.addTab(self.tab_set_type, "")
        self.pushButton_c_ao_start_analysis = QtWidgets.QPushButton(self.tab_configuration)
        self.pushButton_c_ao_start_analysis.setGeometry(QtCore.QRect(160, 740, 161, 50))
        self.pushButton_c_ao_start_analysis.setObjectName("pushButton_c_ao_start_analysis")
        self.pushButton_c_ao_save = QtWidgets.QPushButton(self.tab_configuration)
        self.pushButton_c_ao_save.setGeometry(QtCore.QRect(880, 740, 100, 50))
        self.pushButton_c_ao_save.setObjectName("pushButton_c_ao_save")
        self.pushButton_c_ao_load = QtWidgets.QPushButton(self.tab_configuration)
        self.pushButton_c_ao_load.setGeometry(QtCore.QRect(710, 740, 100, 50))
        self.pushButton_c_ao_load.setObjectName("pushButton_c_ao_load")
        self.tabWidget_general.addTab(self.tab_configuration, "")
        self.tab_monitoring = QtWidgets.QWidget()
        self.tab_monitoring.setObjectName("tab_monitoring")
        self.tabWidget_monitoring = QtWidgets.QTabWidget(self.tab_monitoring)
        self.tabWidget_monitoring.setGeometry(QtCore.QRect(0, 0, 1100, 900))
        self.tabWidget_monitoring.setObjectName("tabWidget_monitoring")
        self.tab_hazard_rate = QtWidgets.QWidget()
        self.tab_hazard_rate.setObjectName("tab_hazard_rate")
        self.groupBox_sys_failure_rate = QtWidgets.QGroupBox(self.tab_hazard_rate)
        self.groupBox_sys_failure_rate.setGeometry(QtCore.QRect(560, 300, 200, 70))
        self.groupBox_sys_failure_rate.setAlignment(QtCore.Qt.AlignCenter)
        self.groupBox_sys_failure_rate.setObjectName("groupBox_sys_failure_rate")
        self.lineEdit_m_sys_lambda = QtWidgets.QLineEdit(self.groupBox_sys_failure_rate)
        self.lineEdit_m_sys_lambda.setGeometry(QtCore.QRect(0, 30, 200, 27))
        self.lineEdit_m_sys_lambda.setReadOnly(True)
        self.lineEdit_m_sys_lambda.setObjectName("lineEdit_m_sys_lambda")
        self.widget_monitoring_hazard_rate = QtWidgets.QWidget(self.tab_hazard_rate)
        self.widget_monitoring_hazard_rate.setGeometry(QtCore.QRect(150, 360, 800, 425))
        self.widget_monitoring_hazard_rate.setObjectName("widget_monitoring_hazard_rate")
        self.groupBox_hazard_rate_sensor_list = QtWidgets.QGroupBox(self.tab_hazard_rate)
        self.groupBox_hazard_rate_sensor_list.setGeometry(QtCore.QRect(600, 20, 400, 290))
        self.groupBox_hazard_rate_sensor_list.setObjectName("groupBox_hazard_rate_sensor_list")
        self.pushButton_monitoring_deleted_sensor = QtWidgets.QPushButton(self.groupBox_hazard_rate_sensor_list)
        self.pushButton_monitoring_deleted_sensor.setGeometry(QtCore.QRect(300, 30, 100, 30))
        self.pushButton_monitoring_deleted_sensor.setAutoDefault(False)
        self.pushButton_monitoring_deleted_sensor.setDefault(False)
        self.pushButton_monitoring_deleted_sensor.setFlat(False)
        self.pushButton_monitoring_deleted_sensor.setObjectName("pushButton_monitoring_deleted_sensor")
        self.label_sensor_list = QtWidgets.QLabel(self.groupBox_hazard_rate_sensor_list)
        self.label_sensor_list.setGeometry(QtCore.QRect(110, 10, 111, 17))
        self.label_sensor_list.setObjectName("label_sensor_list")
        self.listWidget_monitoring_hazard_rate_sensor_list = QtWidgets.QListWidget(self.groupBox_hazard_rate_sensor_list)
        self.listWidget_monitoring_hazard_rate_sensor_list.setGeometry(QtCore.QRect(40, 30, 250, 250))
        self.listWidget_monitoring_hazard_rate_sensor_list.setDragDropMode(QtWidgets.QAbstractItemView.NoDragDrop)
        self.listWidget_monitoring_hazard_rate_sensor_list.setObjectName("listWidget_monitoring_hazard_rate_sensor_list")
        self.groupBox_hazard_rate_add_sensor = QtWidgets.QGroupBox(self.tab_hazard_rate)
        self.groupBox_hazard_rate_add_sensor.setGeometry(QtCore.QRect(100, 130, 440, 170))
        font = QtGui.QFont()
        font.setBold(False)
        font.setItalic(False)
        font.setUnderline(False)
        font.setWeight(50)
        font.setStrikeOut(False)
        font.setKerning(True)
        font.setStyleStrategy(QtGui.QFont.PreferAntialias)
        self.groupBox_hazard_rate_add_sensor.setFont(font)
        self.groupBox_hazard_rate_add_sensor.setFocusPolicy(QtCore.Qt.NoFocus)
        self.groupBox_hazard_rate_add_sensor.setContextMenuPolicy(QtCore.Qt.NoContextMenu)
        self.groupBox_hazard_rate_add_sensor.setAlignment(QtCore.Qt.AlignCenter)
        self.groupBox_hazard_rate_add_sensor.setFlat(False)
        self.groupBox_hazard_rate_add_sensor.setCheckable(False)
        self.groupBox_hazard_rate_add_sensor.setChecked(False)
        self.groupBox_hazard_rate_add_sensor.setObjectName("groupBox_hazard_rate_add_sensor")
        self.label_37 = QtWidgets.QLabel(self.groupBox_hazard_rate_add_sensor)
        self.label_37.setGeometry(QtCore.QRect(20, 30, 121, 21))
        self.label_37.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_37.setObjectName("label_37")
        self.comboBox_m_hazard_rate_select_sensor = QtWidgets.QComboBox(self.groupBox_hazard_rate_add_sensor)
        self.comboBox_m_hazard_rate_select_sensor.setGeometry(QtCore.QRect(150, 30, 151, 27))
        self.comboBox_m_hazard_rate_select_sensor.setObjectName("comboBox_m_hazard_rate_select_sensor")
        self.lineEdit_m_hazard_rate_select_sensor_value = QtWidgets.QLineEdit(self.groupBox_hazard_rate_add_sensor)
        self.lineEdit_m_hazard_rate_select_sensor_value.setGeometry(QtCore.QRect(150, 70, 151, 27))
        self.lineEdit_m_hazard_rate_select_sensor_value.setReadOnly(True)
        self.lineEdit_m_hazard_rate_select_sensor_value.setObjectName("lineEdit_m_hazard_rate_select_sensor_value")
        self.label_38 = QtWidgets.QLabel(self.groupBox_hazard_rate_add_sensor)
        self.label_38.setGeometry(QtCore.QRect(0, 70, 141, 21))
        self.label_38.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_38.setObjectName("label_38")
        self.pushButton_m_hazard_rate_refresh_sensor = QtWidgets.QPushButton(self.groupBox_hazard_rate_add_sensor)
        self.pushButton_m_hazard_rate_refresh_sensor.setGeometry(QtCore.QRect(310, 30, 125, 31))
        self.pushButton_m_hazard_rate_refresh_sensor.setObjectName("pushButton_m_hazard_rate_refresh_sensor")
        self.pushButton_m_hazard_rate_add_sensor = QtWidgets.QPushButton(self.groupBox_hazard_rate_add_sensor)
        self.pushButton_m_hazard_rate_add_sensor.setGeometry(QtCore.QRect(170, 110, 100, 50))
        self.pushButton_m_hazard_rate_add_sensor.setObjectName("pushButton_m_hazard_rate_add_sensor")
        self.groupBox_sys_select_failure_rate_type = QtWidgets.QGroupBox(self.tab_hazard_rate)
        self.groupBox_sys_select_failure_rate_type.setGeometry(QtCore.QRect(340, 300, 200, 70))
        self.groupBox_sys_select_failure_rate_type.setAlignment(QtCore.Qt.AlignCenter)
        self.groupBox_sys_select_failure_rate_type.setObjectName("groupBox_sys_select_failure_rate_type")
        self.comboBox_m_sys_select_failure_rate_type = QtWidgets.QComboBox(self.groupBox_sys_select_failure_rate_type)
        self.comboBox_m_sys_select_failure_rate_type.setGeometry(QtCore.QRect(0, 30, 200, 27))
        self.comboBox_m_sys_select_failure_rate_type.setObjectName("comboBox_m_sys_select_failure_rate_type")
        self.groupBox_sys_select_place_failure_rate = QtWidgets.QGroupBox(self.tab_hazard_rate)
        self.groupBox_sys_select_place_failure_rate.setGeometry(QtCore.QRect(100, 20, 440, 70))
        self.groupBox_sys_select_place_failure_rate.setAlignment(QtCore.Qt.AlignCenter)
        self.groupBox_sys_select_place_failure_rate.setObjectName("groupBox_sys_select_place_failure_rate")
        self.comboBox_m_select_place_failure_rate = QtWidgets.QComboBox(self.groupBox_sys_select_place_failure_rate)
        self.comboBox_m_select_place_failure_rate.setGeometry(QtCore.QRect(160, 30, 200, 27))
        self.comboBox_m_select_place_failure_rate.setObjectName("comboBox_m_select_place_failure_rate")
        self.label_39 = QtWidgets.QLabel(self.groupBox_sys_select_place_failure_rate)
        self.label_39.setGeometry(QtCore.QRect(0, 30, 151, 21))
        self.label_39.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_39.setObjectName("label_39")
        self.tabWidget_monitoring.addTab(self.tab_hazard_rate, "")
        self.tab_reliability = QtWidgets.QWidget()
        self.tab_reliability.setObjectName("tab_reliability")
        self.widget_monitoring_reliability = QtWidgets.QWidget(self.tab_reliability)
        self.widget_monitoring_reliability.setGeometry(QtCore.QRect(150, 360, 800, 425))
        self.widget_monitoring_reliability.setObjectName("widget_monitoring_reliability")
        self.groupBox_sys_reliability = QtWidgets.QGroupBox(self.tab_reliability)
        self.groupBox_sys_reliability.setGeometry(QtCore.QRect(560, 300, 200, 70))
        self.groupBox_sys_reliability.setAlignment(QtCore.Qt.AlignCenter)
        self.groupBox_sys_reliability.setObjectName("groupBox_sys_reliability")
        self.lineEdit_m_sys_reliability = QtWidgets.QLineEdit(self.groupBox_sys_reliability)
        self.lineEdit_m_sys_reliability.setGeometry(QtCore.QRect(0, 30, 200, 27))
        self.lineEdit_m_sys_reliability.setReadOnly(True)
        self.lineEdit_m_sys_reliability.setObjectName("lineEdit_m_sys_reliability")
        self.groupBox_sys_select_reliability_type = QtWidgets.QGroupBox(self.tab_reliability)
        self.groupBox_sys_select_reliability_type.setGeometry(QtCore.QRect(340, 300, 200, 70))
        self.groupBox_sys_select_reliability_type.setAlignment(QtCore.Qt.AlignCenter)
        self.groupBox_sys_select_reliability_type.setObjectName("groupBox_sys_select_reliability_type")
        self.comboBox_m_sys_select_reliability_type = QtWidgets.QComboBox(self.groupBox_sys_select_reliability_type)
        self.comboBox_m_sys_select_reliability_type.setGeometry(QtCore.QRect(0, 30, 200, 27))
        self.comboBox_m_sys_select_reliability_type.setObjectName("comboBox_m_sys_select_reliability_type")
        self.groupBox_6 = QtWidgets.QGroupBox(self.tab_reliability)
        self.groupBox_6.setGeometry(QtCore.QRect(330, 30, 470, 200))
        self.groupBox_6.setObjectName("groupBox_6")
        self.groupBox_select_reliability_module = QtWidgets.QGroupBox(self.groupBox_6)
        self.groupBox_select_reliability_module.setGeometry(QtCore.QRect(0, 30, 200, 60))
        self.groupBox_select_reliability_module.setAlignment(QtCore.Qt.AlignCenter)
        self.groupBox_select_reliability_module.setFlat(False)
        self.groupBox_select_reliability_module.setCheckable(False)
        self.groupBox_select_reliability_module.setObjectName("groupBox_select_reliability_module")
        self.comboBox_m_reliability_module = QtWidgets.QComboBox(self.groupBox_select_reliability_module)
        self.comboBox_m_reliability_module.setGeometry(QtCore.QRect(0, 30, 200, 27))
        self.comboBox_m_reliability_module.setCurrentText("")
        self.comboBox_m_reliability_module.setObjectName("comboBox_m_reliability_module")
        self.groupBox_module_reliability = QtWidgets.QGroupBox(self.groupBox_6)
        self.groupBox_module_reliability.setGeometry(QtCore.QRect(120, 120, 200, 70))
        self.groupBox_module_reliability.setAlignment(QtCore.Qt.AlignCenter)
        self.groupBox_module_reliability.setObjectName("groupBox_module_reliability")
        self.lineEdit_m_module_reliability = QtWidgets.QLineEdit(self.groupBox_module_reliability)
        self.lineEdit_m_module_reliability.setGeometry(QtCore.QRect(0, 30, 200, 27))
        self.lineEdit_m_module_reliability.setReadOnly(True)
        self.lineEdit_m_module_reliability.setObjectName("lineEdit_m_module_reliability")
        self.groupBox_sys_select_module_reliability_type = QtWidgets.QGroupBox(self.groupBox_6)
        self.groupBox_sys_select_module_reliability_type.setGeometry(QtCore.QRect(220, 30, 230, 70))
        self.groupBox_sys_select_module_reliability_type.setAlignment(QtCore.Qt.AlignCenter)
        self.groupBox_sys_select_module_reliability_type.setObjectName("groupBox_sys_select_module_reliability_type")
        self.comboBox_m_sys_select_module_reliability_type = QtWidgets.QComboBox(self.groupBox_sys_select_module_reliability_type)
        self.comboBox_m_sys_select_module_reliability_type.setGeometry(QtCore.QRect(0, 30, 200, 27))
        self.comboBox_m_sys_select_module_reliability_type.setObjectName("comboBox_m_sys_select_module_reliability_type")
        self.tabWidget_monitoring.addTab(self.tab_reliability, "")
        self.tab_potc = QtWidgets.QWidget()
        self.tab_potc.setObjectName("tab_potc")
        self.tabWidget_2 = QtWidgets.QTabWidget(self.tab_potc)
        self.tabWidget_2.setGeometry(QtCore.QRect(0, 0, 1101, 900))
        self.tabWidget_2.setObjectName("tabWidget_2")
        self.tab_6 = QtWidgets.QWidget()
        self.tab_6.setObjectName("tab_6")
        self.widget_monitoring_potc = QtWidgets.QWidget(self.tab_6)
        self.widget_monitoring_potc.setGeometry(QtCore.QRect(160, 310, 800, 425))
        self.widget_monitoring_potc.setObjectName("widget_monitoring_potc")
        self.groupBox_m_potc_actual_group = QtWidgets.QGroupBox(self.tab_6)
        self.groupBox_m_potc_actual_group.setGeometry(QtCore.QRect(80, 30, 500, 275))
        self.groupBox_m_potc_actual_group.setAlignment(QtCore.Qt.AlignCenter)
        self.groupBox_m_potc_actual_group.setObjectName("groupBox_m_potc_actual_group")
        self.groupBox_sys_potc_reliability = QtWidgets.QGroupBox(self.groupBox_m_potc_actual_group)
        self.groupBox_sys_potc_reliability.setGeometry(QtCore.QRect(0, 190, 200, 70))
        self.groupBox_sys_potc_reliability.setAlignment(QtCore.Qt.AlignCenter)
        self.groupBox_sys_potc_reliability.setObjectName("groupBox_sys_potc_reliability")
        self.lineEdit_m_sys_potc_reliability = QtWidgets.QLineEdit(self.groupBox_sys_potc_reliability)
        self.lineEdit_m_sys_potc_reliability.setGeometry(QtCore.QRect(0, 30, 200, 27))
        self.lineEdit_m_sys_potc_reliability.setReadOnly(True)
        self.lineEdit_m_sys_potc_reliability.setObjectName("lineEdit_m_sys_potc_reliability")
        self.groupBox_sys_potc_sb_reliability = QtWidgets.QGroupBox(self.groupBox_m_potc_actual_group)
        self.groupBox_sys_potc_sb_reliability.setGeometry(QtCore.QRect(250, 190, 200, 70))
        self.groupBox_sys_potc_sb_reliability.setAlignment(QtCore.Qt.AlignCenter)
        self.groupBox_sys_potc_sb_reliability.setObjectName("groupBox_sys_potc_sb_reliability")
        self.lineEdit_m_sys_potc_sb_reliability = QtWidgets.QLineEdit(self.groupBox_sys_potc_sb_reliability)
        self.lineEdit_m_sys_potc_sb_reliability.setGeometry(QtCore.QRect(0, 30, 200, 27))
        self.lineEdit_m_sys_potc_sb_reliability.setReadOnly(True)
        self.lineEdit_m_sys_potc_sb_reliability.setObjectName("lineEdit_m_sys_potc_sb_reliability")
        self.groupBox_m_potc_actual_time = QtWidgets.QGroupBox(self.groupBox_m_potc_actual_group)
        self.groupBox_m_potc_actual_time.setGeometry(QtCore.QRect(0, 50, 200, 70))
        self.groupBox_m_potc_actual_time.setAlignment(QtCore.Qt.AlignCenter)
        self.groupBox_m_potc_actual_time.setObjectName("groupBox_m_potc_actual_time")
        self.lineEdit_m_potc_actual_time = QtWidgets.QLineEdit(self.groupBox_m_potc_actual_time)
        self.lineEdit_m_potc_actual_time.setGeometry(QtCore.QRect(0, 30, 200, 27))
        self.lineEdit_m_potc_actual_time.setReadOnly(True)
        self.lineEdit_m_potc_actual_time.setObjectName("lineEdit_m_potc_actual_time")
        self.groupBox_m_potc_actual_distance = QtWidgets.QGroupBox(self.groupBox_m_potc_actual_group)
        self.groupBox_m_potc_actual_distance.setGeometry(QtCore.QRect(250, 50, 200, 70))
        self.groupBox_m_potc_actual_distance.setAlignment(QtCore.Qt.AlignCenter)
        self.groupBox_m_potc_actual_distance.setObjectName("groupBox_m_potc_actual_distance")
        self.lineEdit_m_potc_actual_distance = QtWidgets.QLineEdit(self.groupBox_m_potc_actual_distance)
        self.lineEdit_m_potc_actual_distance.setGeometry(QtCore.QRect(0, 30, 200, 27))
        self.lineEdit_m_potc_actual_distance.setReadOnly(True)
        self.lineEdit_m_potc_actual_distance.setObjectName("lineEdit_m_potc_actual_distance")
        self.line_6 = QtWidgets.QFrame(self.groupBox_m_potc_actual_group)
        self.line_6.setGeometry(QtCore.QRect(0, 130, 500, 30))
        self.line_6.setFrameShadow(QtWidgets.QFrame.Plain)
        self.line_6.setLineWidth(3)
        self.line_6.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_6.setObjectName("line_6")
        self.groupBox_m_potc_predict_group = QtWidgets.QGroupBox(self.tab_6)
        self.groupBox_m_potc_predict_group.setGeometry(QtCore.QRect(580, 30, 450, 275))
        self.groupBox_m_potc_predict_group.setAlignment(QtCore.Qt.AlignCenter)
        self.groupBox_m_potc_predict_group.setObjectName("groupBox_m_potc_predict_group")
        self.groupBox_sys_potc_predict_reliability = QtWidgets.QGroupBox(self.groupBox_m_potc_predict_group)
        self.groupBox_sys_potc_predict_reliability.setGeometry(QtCore.QRect(0, 190, 200, 70))
        self.groupBox_sys_potc_predict_reliability.setAlignment(QtCore.Qt.AlignCenter)
        self.groupBox_sys_potc_predict_reliability.setObjectName("groupBox_sys_potc_predict_reliability")
        self.lineEdit_m_sys_potc_predict_reliability = QtWidgets.QLineEdit(self.groupBox_sys_potc_predict_reliability)
        self.lineEdit_m_sys_potc_predict_reliability.setGeometry(QtCore.QRect(0, 30, 200, 27))
        self.lineEdit_m_sys_potc_predict_reliability.setReadOnly(True)
        self.lineEdit_m_sys_potc_predict_reliability.setObjectName("lineEdit_m_sys_potc_predict_reliability")
        self.groupBox_sys_potc_predict_sb_reliability = QtWidgets.QGroupBox(self.groupBox_m_potc_predict_group)
        self.groupBox_sys_potc_predict_sb_reliability.setGeometry(QtCore.QRect(250, 190, 200, 70))
        self.groupBox_sys_potc_predict_sb_reliability.setAlignment(QtCore.Qt.AlignCenter)
        self.groupBox_sys_potc_predict_sb_reliability.setObjectName("groupBox_sys_potc_predict_sb_reliability")
        self.lineEdit_m_sys_potc_predict_sb_reliability = QtWidgets.QLineEdit(self.groupBox_sys_potc_predict_sb_reliability)
        self.lineEdit_m_sys_potc_predict_sb_reliability.setGeometry(QtCore.QRect(0, 30, 200, 27))
        self.lineEdit_m_sys_potc_predict_sb_reliability.setReadOnly(True)
        self.lineEdit_m_sys_potc_predict_sb_reliability.setObjectName("lineEdit_m_sys_potc_predict_sb_reliability")
        self.groupBox_m_potc_predict_time = QtWidgets.QGroupBox(self.groupBox_m_potc_predict_group)
        self.groupBox_m_potc_predict_time.setGeometry(QtCore.QRect(0, 50, 200, 70))
        self.groupBox_m_potc_predict_time.setAlignment(QtCore.Qt.AlignCenter)
        self.groupBox_m_potc_predict_time.setObjectName("groupBox_m_potc_predict_time")
        self.lineEdit_m_potc_predict_time = QtWidgets.QLineEdit(self.groupBox_m_potc_predict_time)
        self.lineEdit_m_potc_predict_time.setGeometry(QtCore.QRect(0, 30, 200, 27))
        self.lineEdit_m_potc_predict_time.setReadOnly(True)
        self.lineEdit_m_potc_predict_time.setObjectName("lineEdit_m_potc_predict_time")
        self.groupBox_m_potc_predict_distance = QtWidgets.QGroupBox(self.groupBox_m_potc_predict_group)
        self.groupBox_m_potc_predict_distance.setGeometry(QtCore.QRect(250, 50, 200, 70))
        self.groupBox_m_potc_predict_distance.setAlignment(QtCore.Qt.AlignCenter)
        self.groupBox_m_potc_predict_distance.setObjectName("groupBox_m_potc_predict_distance")
        self.lineEdit_m_potc_predict_distance = QtWidgets.QLineEdit(self.groupBox_m_potc_predict_distance)
        self.lineEdit_m_potc_predict_distance.setGeometry(QtCore.QRect(0, 30, 200, 27))
        self.lineEdit_m_potc_predict_distance.setReadOnly(True)
        self.lineEdit_m_potc_predict_distance.setObjectName("lineEdit_m_potc_predict_distance")
        self.line_7 = QtWidgets.QFrame(self.groupBox_m_potc_predict_group)
        self.line_7.setGeometry(QtCore.QRect(0, 130, 450, 30))
        self.line_7.setFrameShadow(QtWidgets.QFrame.Plain)
        self.line_7.setLineWidth(3)
        self.line_7.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_7.setObjectName("line_7")
        self.line_8 = QtWidgets.QFrame(self.tab_6)
        self.line_8.setGeometry(QtCore.QRect(540, 70, 30, 210))
        self.line_8.setFrameShadow(QtWidgets.QFrame.Plain)
        self.line_8.setLineWidth(3)
        self.line_8.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_8.setObjectName("line_8")
        self.tabWidget_2.addTab(self.tab_6, "")
        self.tab_7 = QtWidgets.QWidget()
        self.tab_7.setObjectName("tab_7")
        self.groupBox_potc_predict = QtWidgets.QGroupBox(self.tab_7)
        self.groupBox_potc_predict.setGeometry(QtCore.QRect(60, 30, 800, 600))
        self.groupBox_potc_predict.setObjectName("groupBox_potc_predict")
        self.groupBox_m_prognostic_potc_failure_rate = QtWidgets.QGroupBox(self.groupBox_potc_predict)
        self.groupBox_m_prognostic_potc_failure_rate.setGeometry(QtCore.QRect(510, 260, 200, 70))
        self.groupBox_m_prognostic_potc_failure_rate.setAlignment(QtCore.Qt.AlignCenter)
        self.groupBox_m_prognostic_potc_failure_rate.setObjectName("groupBox_m_prognostic_potc_failure_rate")
        self.lineEdit_m_prognostic_potc = QtWidgets.QLineEdit(self.groupBox_m_prognostic_potc_failure_rate)
        self.lineEdit_m_prognostic_potc.setGeometry(QtCore.QRect(0, 30, 200, 27))
        self.lineEdit_m_prognostic_potc.setReadOnly(True)
        self.lineEdit_m_prognostic_potc.setObjectName("lineEdit_m_prognostic_potc")
        self.groupBox_m_prognostic_potc_simulation_time = QtWidgets.QGroupBox(self.groupBox_potc_predict)
        self.groupBox_m_prognostic_potc_simulation_time.setGeometry(QtCore.QRect(220, 260, 200, 70))
        self.groupBox_m_prognostic_potc_simulation_time.setAlignment(QtCore.Qt.AlignCenter)
        self.groupBox_m_prognostic_potc_simulation_time.setObjectName("groupBox_m_prognostic_potc_simulation_time")
        self.lineEdit_m_prognostic_potc_simulation_time = QtWidgets.QLineEdit(self.groupBox_m_prognostic_potc_simulation_time)
        self.lineEdit_m_prognostic_potc_simulation_time.setGeometry(QtCore.QRect(0, 30, 200, 27))
        self.lineEdit_m_prognostic_potc_simulation_time.setReadOnly(True)
        self.lineEdit_m_prognostic_potc_simulation_time.setObjectName("lineEdit_m_prognostic_potc_simulation_time")
        self.groupBox_general = QtWidgets.QGroupBox(self.groupBox_potc_predict)
        self.groupBox_general.setGeometry(QtCore.QRect(0, 0, 800, 231))
        self.groupBox_general.setTitle("")
        self.groupBox_general.setObjectName("groupBox_general")
        self.groupBox_m_prognostic_potc_select_task = QtWidgets.QGroupBox(self.groupBox_general)
        self.groupBox_m_prognostic_potc_select_task.setGeometry(QtCore.QRect(220, 60, 200, 60))
        self.groupBox_m_prognostic_potc_select_task.setAlignment(QtCore.Qt.AlignCenter)
        self.groupBox_m_prognostic_potc_select_task.setFlat(False)
        self.groupBox_m_prognostic_potc_select_task.setCheckable(False)
        self.groupBox_m_prognostic_potc_select_task.setObjectName("groupBox_m_prognostic_potc_select_task")
        self.comboBox_m_prognostic_potc_select_task = QtWidgets.QComboBox(self.groupBox_m_prognostic_potc_select_task)
        self.comboBox_m_prognostic_potc_select_task.setGeometry(QtCore.QRect(5, 25, 190, 27))
        self.comboBox_m_prognostic_potc_select_task.setCurrentText("")
        self.comboBox_m_prognostic_potc_select_task.setObjectName("comboBox_m_prognostic_potc_select_task")
        self.pushButton_m_prognostic_potc_start_simulation = QtWidgets.QPushButton(self.groupBox_general)
        self.pushButton_m_prognostic_potc_start_simulation.setGeometry(QtCore.QRect(380, 170, 150, 50))
        self.pushButton_m_prognostic_potc_start_simulation.setObjectName("pushButton_m_prognostic_potc_start_simulation")
        self.groupBox_m_prognostic_potc_simulation_count = QtWidgets.QGroupBox(self.groupBox_general)
        self.groupBox_m_prognostic_potc_simulation_count.setGeometry(QtCore.QRect(510, 60, 200, 60))
        self.groupBox_m_prognostic_potc_simulation_count.setAlignment(QtCore.Qt.AlignCenter)
        self.groupBox_m_prognostic_potc_simulation_count.setFlat(False)
        self.groupBox_m_prognostic_potc_simulation_count.setCheckable(False)
        self.groupBox_m_prognostic_potc_simulation_count.setObjectName("groupBox_m_prognostic_potc_simulation_count")
        self.lineEdit_m_prognostic_potc_simulation_count = QtWidgets.QLineEdit(self.groupBox_m_prognostic_potc_simulation_count)
        self.lineEdit_m_prognostic_potc_simulation_count.setGeometry(QtCore.QRect(5, 25, 190, 27))
        self.lineEdit_m_prognostic_potc_simulation_count.setObjectName("lineEdit_m_prognostic_potc_simulation_count")
        self.tabWidget_2.addTab(self.tab_7, "")
        self.tabWidget_monitoring.addTab(self.tab_potc, "")
        self.tabWidget_general.addTab(self.tab_monitoring, "")
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
        self.tabWidget_general.setCurrentIndex(0)
        self.tabWidget_configuration.setCurrentIndex(0)
        self.tabWidget.setCurrentIndex(0)
        self.tabWidget_5.setCurrentIndex(0)
        self.tabWidget_types.setCurrentIndex(0)
        self.tabWidget_monitoring.setCurrentIndex(0)
        self.tabWidget_2.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)


    def retranslateUi(self, MainWindow):
        """
            PHM Gui Retranslate Ui
        """
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "PHM GUI"))
        self.groupBox_c_ao_am_add_module.setTitle(_translate("MainWindow", "Add Module"))
        self.label_12.setText(_translate("MainWindow", "Set module name:"))
        self.groupBox_c_ao_am_module_fr.setTitle(_translate("MainWindow", "Set failure rate"))
        self.label_13.setText(_translate("MainWindow", "Module failure rate:"))
        self.label_42.setText(_translate("MainWindow", "Type : "))
        self.label_43.setText(_translate("MainWindow", "Total failure rate:"))
        self.label_44.setText(_translate("MainWindow", "Quantity : "))
        self.pushButton_c_ao_am_add_module.setText(_translate("MainWindow", "Add"))
        self.groupBox_c_ao_am_list_of_modules.setTitle(_translate("MainWindow", "List of modules"))
        self.pushButton_c_ao_am_delete_module.setText(_translate("MainWindow", "Delete"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_3), _translate("MainWindow", "Add Module"))
        self.groupBox_c_ao_asm_sub_module_fr.setTitle(_translate("MainWindow", "Set failure rate"))
        self.label_18.setText(_translate("MainWindow", "Sub-Module failure rate:"))
        self.label_36.setText(_translate("MainWindow", "Quantity : "))
        self.label_40.setText(_translate("MainWindow", "Total failure rate:"))
        self.label_41.setText(_translate("MainWindow", "Type : "))
        self.groupBox_c_ao_asm_default_fr.setTitle(_translate("MainWindow", "Set failure rate from default equipments"))
        self.label_19.setText(_translate("MainWindow", "Select the equipment :"))
        self.label_20.setText(_translate("MainWindow", "Type : "))
        self.label_21.setText(_translate("MainWindow", "Total failure rate:"))
        self.label_30.setText(_translate("MainWindow", "Quantity : "))
        self.pushButton_c_ao_asm_set_equipment.setText(_translate("MainWindow", "Set Equipment"))
        self.label_33.setText(_translate("MainWindow", "Equipment failure rate :  "))
        self.label_28.setText(_translate("MainWindow", "Select module : "))
        self.pushButton_c_ao_asm_add_sub_module.setText(_translate("MainWindow", "Add"))
        self.label_29.setText(_translate("MainWindow", "Set sub-module name"))
        self.groupBox_c_ao_asm_list_of_sub_modules.setTitle(_translate("MainWindow", "List of sub-modules"))
        self.pushButton_c_ao_asm_delete_sub_module.setText(_translate("MainWindow", "Delete"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_5), _translate("MainWindow", "Add Sub-Module"))
        self.pushButton_c_ao_ac_add_component.setText(_translate("MainWindow", "Add"))
        self.label_9.setText(_translate("MainWindow", "Select module : "))
        self.label_14.setText(_translate("MainWindow", "Select sub - module : "))
        self.groupBox_c_ao_ac_list_of_components.setTitle(_translate("MainWindow", "List of components"))
        self.pushButton_c_ao_ac_delete_component.setText(_translate("MainWindow", "Delete"))
        self.groupBox_c_ao_ac_default_fr.setTitle(_translate("MainWindow", "Set default component from equipments"))
        self.label_7.setText(_translate("MainWindow", "Select the equipment :"))
        self.label_8.setText(_translate("MainWindow", "Quantity : "))
        self.label_11.setText(_translate("MainWindow", "Equipment failure rate : "))
        self.pushButton_c_ao_ac_set_equipment.setText(_translate("MainWindow", "Set Equipment"))
        self.label_34.setText(_translate("MainWindow", "Total failure rate : "))
        self.label_17.setText(_translate("MainWindow", "Set component name"))
        self.groupBox_c_ao_ac_component_fr.setTitle(_translate("MainWindow", "Set failure rate"))
        self.label_10.setText(_translate("MainWindow", "Component failure rate:"))
        self.label_31.setText(_translate("MainWindow", "Quantity : "))
        self.label_35.setText(_translate("MainWindow", "Total failure rate : "))
        self.tabWidget_5.setTabText(self.tabWidget_5.indexOf(self.tab_10), _translate("MainWindow", "Default Components"))
        self.label.setText(_translate("MainWindow", "Select module : "))
        self.label_2.setText(_translate("MainWindow", "Select sub - module : "))
        self.label_4.setText(_translate("MainWindow", "Set element name"))
        self.label_5.setText(_translate("MainWindow", "Set variable name of the element"))
        self.pushButton_c_ao_ace_add_component_element.setText(_translate("MainWindow", "Add"))
        self.groupBox_c_ao_ace_set_element_value.setTitle(_translate("MainWindow", "Set element value:"))
        self.label_32.setText(_translate("MainWindow", "Set component name"))
        self.groupBox_c_ao_ace_list_of_elements.setTitle(_translate("MainWindow", "List of elements"))
        self.pushButton_c_ao_ace_delete_element.setText(_translate("MainWindow", "Delete"))
        self.label_3.setText(_translate("MainWindow", "Select component : "))
        self.pushButton_c_ao_ac_set_formula.setText(_translate("MainWindow", "Set Formula"))
        self.label_15.setText(_translate("MainWindow", "Set formula of component"))
        self.label_16.setText(_translate("MainWindow", "Failure Rate: "))
        self.label_6.setText(_translate("MainWindow", "Select component : "))
        self.tabWidget_5.setTabText(self.tabWidget_5.indexOf(self.tab_11), _translate("MainWindow", "Custom Components"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_4), _translate("MainWindow", "Add Component"))
        self.tabWidget_configuration.setTabText(self.tabWidget_configuration.indexOf(self.tab_add_object), _translate("MainWindow", "Robot Design"))
        self.label_configurations_4.setText(_translate("MainWindow", "Configurations"))
        self.label_configuration_select_first_comp.setText(_translate("MainWindow", "Select first component"))
        self.label_configuration_sub_module_list_3.setText(_translate("MainWindow", "Select system:"))
        self.pushButton_configuration_set_comp_type.setText(_translate("MainWindow", "Set"))
        self.label_24.setText(_translate("MainWindow", "is"))
        self.label_25.setText(_translate("MainWindow", "to"))
        self.pushButton_delete_component_config.setText(_translate("MainWindow", "Delete"))
        self.pushButton_add_component_config.setText(_translate("MainWindow", "Add"))
        self.label_configuration_select_second_comp.setText(_translate("MainWindow", "Select second component"))
        self.pushButton_select_final_configuration_of_sub_module.setText(_translate("MainWindow", "Select as final configuraiton"))
        self.label_configuration_failure_rate_7.setText(_translate("MainWindow", "Failure rate:"))
        self.label_configuration_sub_module_list_2.setText(_translate("MainWindow", "Select module:"))
        self.label_configuration_sub_module_list.setText(_translate("MainWindow", "Select sub-module:"))
        self.label_components.setText(_translate("MainWindow", "Components"))
        self.label_configuration_failure_rate_8.setText(_translate("MainWindow", "Sub-module failure rate:"))
        self.tabWidget_types.setTabText(self.tabWidget_types.indexOf(self.tab), _translate("MainWindow", "Sub Module Configuration"))
        self.pushButton_delete_sub_module_config.setText(_translate("MainWindow", "Delete"))
        self.pushButton_add_sub_module_config.setText(_translate("MainWindow", "Add"))
        self.pushButton_configuration_set_sub_module_type.setText(_translate("MainWindow", "Set"))
        self.label_26.setText(_translate("MainWindow", "is"))
        self.label_27.setText(_translate("MainWindow", "to"))
        self.label_configuration_failure_rate_9.setText(_translate("MainWindow", "Module failure rate:"))
        self.label_configuration_select_first_sub_module.setText(_translate("MainWindow", "Select first sub-module"))
        self.label_configuration_select_second_sub_module.setText(_translate("MainWindow", "Select second sub-module"))
        self.label_configuration_failure_rate_10.setText(_translate("MainWindow", "Failure rate:"))
        self.label_configuration_sub_module_list_4.setText(_translate("MainWindow", "Select system:"))
        self.label_configuration_module_list.setText(_translate("MainWindow", "Select module:"))
        self.pushButton_select_final_configuration_of_module.setText(_translate("MainWindow", "Select as final configuraiton"))
        self.label_configurations_5.setText(_translate("MainWindow", "Configurations"))
        self.label_sub_module.setText(_translate("MainWindow", "Sub-modules"))
        self.tabWidget_types.setTabText(self.tabWidget_types.indexOf(self.tab_2), _translate("MainWindow", "Module Configuration"))
        self.label_configurations_3.setText(_translate("MainWindow", "Configurations"))
        self.label_configuration_system_list.setText(_translate("MainWindow", "Select system:"))
        self.label_modules.setText(_translate("MainWindow", "Modules"))
        self.pushButton_delete_module_config.setText(_translate("MainWindow", "Delete"))
        self.pushButton_add_module_config.setText(_translate("MainWindow", "Add"))
        self.pushButton_configuration_set_module_type.setText(_translate("MainWindow", "Set"))
        self.label_22.setText(_translate("MainWindow", "is"))
        self.label_23.setText(_translate("MainWindow", "to"))
        self.label_configuration_select_second_module.setText(_translate("MainWindow", "Select second module"))
        self.label_configuration_select_first_module.setText(_translate("MainWindow", "Select first module"))
        self.label_configuration_failure_rate_5.setText(_translate("MainWindow", "Failure rate:"))
        self.label_configuration_system_failure_rate.setText(_translate("MainWindow", "System failure rate:"))
        self.pushButton_select_final_configuration_of_system.setText(_translate("MainWindow", "Select as final configuraiton"))
        self.label_configuration_reliability_type.setText(_translate("MainWindow", "Select reliability model : "))
        self.label_configuration_unit.setText(_translate("MainWindow", "Select unit : "))
        self.label_configuration_shape_parameter.setText(_translate("MainWindow", "Shape Parameter :"))
        self.tabWidget_types.setTabText(self.tabWidget_types.indexOf(self.tab_set_module_types_2), _translate("MainWindow", "System Configuration"))
        self.tabWidget_configuration.setTabText(self.tabWidget_configuration.indexOf(self.tab_set_type), _translate("MainWindow", "Configuration Setup"))
        self.pushButton_c_ao_start_analysis.setText(_translate("MainWindow", "START ANALYSIS"))
        self.pushButton_c_ao_save.setText(_translate("MainWindow", "SAVE"))
        self.pushButton_c_ao_load.setText(_translate("MainWindow", "LOAD"))
        self.tabWidget_general.setTabText(self.tabWidget_general.indexOf(self.tab_configuration), _translate("MainWindow", "Robot Configuration Setup"))
        self.groupBox_sys_failure_rate.setTitle(_translate("MainWindow", "System Failure Rate - λ"))
        self.groupBox_hazard_rate_sensor_list.setTitle(_translate("MainWindow", "Sensors"))
        self.pushButton_monitoring_deleted_sensor.setText(_translate("MainWindow", "Delete"))
        self.label_sensor_list.setText(_translate("MainWindow", "Sensor List"))
        self.groupBox_hazard_rate_add_sensor.setTitle(_translate("MainWindow", "Add Sensor"))
        self.label_37.setText(_translate("MainWindow", "Select a sensor:"))
        self.label_38.setText(_translate("MainWindow", "Sensor value:"))
        self.pushButton_m_hazard_rate_refresh_sensor.setText(_translate("MainWindow", "Refresh"))
        self.pushButton_m_hazard_rate_add_sensor.setText(_translate("MainWindow", "Add Sensor"))
        self.groupBox_sys_select_failure_rate_type.setTitle(_translate("MainWindow", "Select Failure Rate"))
        self.groupBox_sys_select_place_failure_rate.setTitle(_translate("MainWindow", "Sensor"))
        self.label_39.setText(_translate("MainWindow", "Where is the sensor? "))
        self.tabWidget_monitoring.setTabText(self.tabWidget_monitoring.indexOf(self.tab_hazard_rate), _translate("MainWindow", "Hazard Rate Analysis"))
        self.groupBox_sys_reliability.setTitle(_translate("MainWindow", "System Reliability - R"))
        self.groupBox_sys_select_reliability_type.setTitle(_translate("MainWindow", "Select Reliability"))
        self.groupBox_6.setTitle(_translate("MainWindow", "Module Reliability"))
        self.groupBox_select_reliability_module.setTitle(_translate("MainWindow", "Select Module"))
        self.groupBox_module_reliability.setTitle(_translate("MainWindow", "Module Reliability - R"))
        self.groupBox_sys_select_module_reliability_type.setTitle(_translate("MainWindow", "Select Module Reliability Type"))
        self.tabWidget_monitoring.setTabText(self.tabWidget_monitoring.indexOf(self.tab_reliability), _translate("MainWindow", "Reliability Analysis"))
        self.groupBox_m_potc_actual_group.setTitle(_translate("MainWindow", "Actual Analysis"))
        self.groupBox_sys_potc_reliability.setTitle(_translate("MainWindow", "Nominal PoTC"))
        self.groupBox_sys_potc_sb_reliability.setTitle(_translate("MainWindow", "Sensor Based PoTC"))
        self.groupBox_m_potc_actual_time.setTitle(_translate("MainWindow", "Actual Time"))
        self.groupBox_m_potc_actual_distance.setTitle(_translate("MainWindow", "Actual Distance"))
        self.groupBox_m_potc_predict_group.setTitle(_translate("MainWindow", "Predict Analysis"))
        self.groupBox_sys_potc_predict_reliability.setTitle(_translate("MainWindow", "Nominal PoTC"))
        self.groupBox_sys_potc_predict_sb_reliability.setTitle(_translate("MainWindow", "Sensor Based PoTC"))
        self.groupBox_m_potc_predict_time.setTitle(_translate("MainWindow", "Predicted Time"))
        self.groupBox_m_potc_predict_distance.setTitle(_translate("MainWindow", "Predicted Distance"))
        self.tabWidget_2.setTabText(self.tabWidget_2.indexOf(self.tab_6), _translate("MainWindow", "Real Time Analysis"))
        self.groupBox_potc_predict.setTitle(_translate("MainWindow", "Predict PoTC"))
        self.groupBox_m_prognostic_potc_failure_rate.setTitle(_translate("MainWindow", "Predicted PoTC"))
        self.groupBox_m_prognostic_potc_simulation_time.setTitle(_translate("MainWindow", "Simulation Time"))
        self.groupBox_m_prognostic_potc_select_task.setTitle(_translate("MainWindow", "Select Task"))
        self.pushButton_m_prognostic_potc_start_simulation.setText(_translate("MainWindow", "Predict Simulation"))
        self.groupBox_m_prognostic_potc_simulation_count.setTitle(_translate("MainWindow", "Simulation Count"))
        self.tabWidget_2.setTabText(self.tabWidget_2.indexOf(self.tab_7), _translate("MainWindow", "Prognostic Analysis"))
        self.tabWidget_monitoring.setTabText(self.tabWidget_monitoring.indexOf(self.tab_potc), _translate("MainWindow", "Task Completion Analysis"))
        self.tabWidget_general.setTabText(self.tabWidget_general.indexOf(self.tab_monitoring), _translate("MainWindow", "Monitoring and Analysis"))


# ------------------------------------------------------------------------------------------------

# "PHM MAIN FUNCTIONS" - START -

    def gui_main(self):
        """
            PHM Gui Main Function
        """
        app.aboutToQuit.connect(self.close_event)

        self.pushButton_c_ao_start_analysis.clicked.connect(self.click_start_analysis_func)

        self.start_phm_gui_main_dict_func()
        self.start_set_reliability_model_and_unit_func()

     # CONFIGURATION // SET TYPE
        self.configuration_set_type_events_func()

     # CONFIGURATION // ADD OBJECT 2
        self.configuration_add_object_2_tab_events_func()

     # MONITORING
        self.monitoring_events_func()
        self.monitoring_general_tab_func()

        self.pushButton_m_prognostic_potc_start_simulation.clicked.connect(self.monitoring_prognostic_robot_task_completion_func)
        self.gui_main_plot()
        self.general_set_result_func()

     # PHM START - SAVE AND LOAD
        self.start_phm_save_and_load_buttons_control_func()


    def gui_main_plot(self):
        """
            PHM Gui Main Plot Function
        """
     # MONITORING
        # Gui Graphs
        layout = QtWidgets.QVBoxLayout(self.widget_monitoring_hazard_rate)
        monitoring_hazard_rate_dinamic_canvas = DynamicPlot(self, "lineEdit_m_sys_lambda", "phm_gui_time", self.widget_monitoring_hazard_rate, width=5, height=4, dpi=100)
        layout.addWidget(monitoring_hazard_rate_dinamic_canvas)
        self.widget_monitoring_hazard_rate.setFocus()

        layout = QtWidgets.QVBoxLayout(self.widget_monitoring_reliability)
        reliability_dinamic_canvas = ReliabilityPlot(self, "lineEdit_m_sys_reliability", "phm_gui_time", self.reliability_plot_scale_list, self.widget_monitoring_reliability, width=5, height=4, dpi=100)
        layout.addWidget(reliability_dinamic_canvas)
        self.widget_monitoring_reliability.setFocus()

        layout = QtWidgets.QVBoxLayout(self.widget_monitoring_potc)
        potc_static_canvas = POTCPlot(self, "potc_main_dict", width=5, height=4, dpi=100)
        layout.addWidget(potc_static_canvas)
        self.widget_monitoring_potc.setFocus()


    def close_event(self):
        """
            PHM Gui Close Event Function
        """
        rospy.signal_shutdown("ROS Close")
        print("\n\nSystem is closing...\n\n")

        file_dir = "phm_start/params/"
        temp_file_name = "last_configurations"
        temp_data = self.main_dict

        self.fill_the_file_with_data(file_dir, temp_file_name, temp_data)

        print("\n\nYour files are saving...\n\n")


    def click_start_analysis_func(self):
        """
            Start Anaylsis Function
        """
        self.label_configuration_reliability_type.setEnabled(False)
        self.comboBox_configuration_reliability_model.setEnabled(False)
        self.label_configuration_shape_parameter.setEnabled(False)
        self.lineEdit_configuration_reliability_shape_parameter.setEnabled(False)
        self.label_configuration_unit.setEnabled(False)
        self.comboBox_configuration_unit.setEnabled(False)
        self.pushButton_c_ao_start_analysis.setEnabled(False)

        self.general_threads_func()


    def start_set_reliability_model_and_unit_func(self):
        """
            Start Set Reliability Model and Unit Function
        """
        self.comboBox_configuration_reliability_model.addItems(self.configuration_reliability_model)
        self.comboBox_configuration_unit.addItems(self.configuration_reliability_unit)
        self.label_configuration_shape_parameter.setEnabled(False)
        self.lineEdit_configuration_reliability_shape_parameter.setEnabled(False)


# "PHM MAIN FUNCTIONS" - END -

# "PHM GENERAL THREADS FUNCTIONS" - START -

    def general_threads_func(self):
        """
            PHM General Threads Function
        """
        self.thread_calculation_system_main_dict = Thread(target=self.thread_calculation_system_main_dict_func)
        self.thread_calculation_system_main_dict.start()

        self.thread_monitoring_system_potc = Thread(target=self.thread_monitoring_system_potc_func)
        self.thread_monitoring_system_potc.start()

        self.thread_temperature_sensor_dict = Thread(target=self.thread_temperature_sensors_func)
        self.thread_temperature_sensor_dict.start()


    def thread_calculation_system_main_dict_func(self):
        """
            Calculation System Main Dict Thread Function
        """
        try:
            while not rospy.is_shutdown():
                self.main_dict_selected_system_calculation_func()
                self.general_set_result_func()

                self.publisher_gui_hazard_rate_func()
                self.publisher_gui_reliability_func()

                self.configuration_setup_set_result_func()
                #print("\n\n")
                #print(self.main_dict)
                #print("\n\n")

                self.thread_rate.sleep()

        except Exception as err:
            print("\nError: thread_calculation_system_main_dict_func\n")
            print(err)


    def thread_monitoring_system_potc_func(self):
        """
            Monitoring System POTC Thread Function
        """
        try:
            while not rospy.is_shutdown():
                self.monitoring_system_robot_task_completion_func()

                self.thread_rate.sleep()

        except Exception as err:
            print("\nError: thread_monitoring_system_potc_func\n")
            print(err)


    def thread_temperature_sensors_func(self):
        """
            Temperature Sensors Thread Function
        """
        try:
            while not rospy.is_shutdown():
                self.phm_temperature_sensor_thread_func()

                self.thread_rate.sleep()

        except Exception as err:
            print("\nError: thread_temperature_sensors_func\n")
            print(err)

# "PHM GENERAL THREADS FUNCTIONS" - END -

# "PHM GENERAL PUBLISHER FUNCTIONS" - START -

    def publisher_gui_hazard_rate_func(self):
        """
            /gui_hazard_rate Publisher Function
        """
        phm_msg = String()

        filtered_hazard_rate_dict = self.filter_hazard_rate_func(self.main_dict)
        phm_msg.data = str(filtered_hazard_rate_dict)

        self.publisher_gui_hazard_rate.publish(phm_msg)


    def filter_hazard_rate_func(self, main_dict):
        """
            Hazard Rate Filter Function
        """
        filtered_dict = dict()
        system_dict_keys = list(main_dict["System"].keys())
        filtered_dict["System"] = {"Failure Rate": main_dict["System"]["Failure Rate"]}
        self.remove_key(system_dict_keys)

        for item in system_dict_keys:
            filtered_dict["System"][str(item)] = {"Failure Rate": main_dict["System"][str(item)]["Failure Rate"]}

        return filtered_dict


    def publisher_gui_reliability_func(self):
        """
            /gui_reliability Publisher Function
        """
        phm_msg = String()

        filtered_reliability_dict = self.filter_reliability_func(self.main_dict)
        phm_msg.data = str(filtered_reliability_dict)

        self.publisher_gui_reliability.publish(phm_msg)


    def filter_reliability_func(self, main_dict):
        """
            Reliability Filter Function
        """
        filtered_dict = dict()
        system_dict_keys = list(main_dict["System"].keys())
        filtered_dict["System"] = {"Reliability": main_dict["System"]["Reliability"]}
        self.remove_key(system_dict_keys)

        for item in system_dict_keys:
            filtered_dict["System"][str(item)] = {"Reliability": main_dict["System"][str(item)]["Reliability"]}

        return filtered_dict


    def publisher_gui_actual_potc_func(self, publish_data):
        """
            /gui_actual_potc Publisher Function
        """
        phm_msg = String()
        phm_msg.data = str(publish_data)

        self.publisher_gui_actual_potc.publish(phm_msg)


    def publisher_gui_predict_potc_func(self, publish_data):
        """
            /gui_predict_potc Publisher Function
        """
        phm_msg = String()
        phm_msg.data = str(publish_data)

        self.publisher_gui_predict_potc.publish(phm_msg)


# "PHM GENERAL PUBLISHER FUNCTIONS" - END -

# "PHM SYSTEM GENERAL CALCULATION FUNCTIONS" - START -

    def main_dict_selected_system_calculation_func(self):
        """
            Selected System Calculation Function
        """
        try:
            selected_system = "System"
            start = time.time()
            self.main_dict_module_calculate_func(selected_system)
            done = time.time()
            time_diff = done - start
            #print("\n\n\nTime diff")
            #print(time_diff)

        except Exception as err:
            print("\nError: main_dict_selected_system_calculation_func\n")
            print(err)


    def main_dict_module_calculate_func(self, selected_system):
        """
            Module Calculation Function
        """
        try:
            selected_system_dict = self.main_dict[str(selected_system)]
            selected_configuration = dict(selected_system_dict["Configurations"])
            del selected_configuration["Configuration Count"]
            control_dict = dict()

            if str(selected_configuration) != str(control_dict):
                system_depth_level = selected_configuration["Depth Level"]
                selected_final_configuration = selected_system_dict["Final Configuration"]

                configuration_list = list(selected_configuration.keys())
                self.remove_key(configuration_list)

                module_list = list(selected_system_dict.keys())
                self.remove_key(module_list)

                sorted_list = self.sorted_list_for_depth_level(selected_configuration, configuration_list, system_depth_level)

                # Alt katmandakilerin hesabi yapildi
                for module in module_list:
                    self.main_dict_sub_module_calculate_func(selected_system, module)

                # Mevcut katmanin isi yapiliyor
                for item in sorted_list:
                    item_module_list = selected_system_dict["Configurations"][str(item)]["Components"]
                    item_type = selected_system_dict["Configurations"][str(item)]["Type"]

                    calculate_fr = self.calculate_selected_module_fr_func(selected_system, configuration_list, item_module_list, item_type, True)
                    sb_calculate_fr = self.calculate_selected_module_fr_func(selected_system, configuration_list, item_module_list, item_type, False)

                    self.main_dict[str(selected_system)]["Configurations"][str(item)]["Failure Rate"]["Nominal"] = calculate_fr
                    self.main_dict[str(selected_system)]["Configurations"][str(item)]["Failure Rate"]["Sensor Based"] = sb_calculate_fr

                configuration_keys = list(self.main_dict[str(selected_system)]["Configurations"].keys())
                self.remove_key(configuration_keys)

                if selected_final_configuration not in configuration_keys:
                    if selected_final_configuration not in module_list:
                        self.main_dict[str(selected_system)]["Final Configuration"] = ""
                        selected_final_configuration == ""

                if selected_final_configuration == "":
                    selected_system_fr = 0.0
                    selected_system_sb_fr = 0.0

                else:
                    if selected_final_configuration in configuration_keys:
                        selected_system_fr = self.main_dict[str(selected_system)]["Configurations"][str(selected_final_configuration)]["Failure Rate"]["Nominal"]
                        selected_system_sb_fr = self.main_dict[str(selected_system)]["Configurations"][str(selected_final_configuration)]["Failure Rate"]["Sensor Based"]

                    else:
                        selected_system_fr = self.main_dict[str(selected_system)][str(selected_final_configuration)]["Failure Rate"]["Nominal"]
                        selected_system_sb_fr = self.main_dict[str(selected_system)][str(selected_final_configuration)]["Failure Rate"]["Sensor Based"]

                self.main_dict[str(selected_system)]["Failure Rate"]["Nominal"] = selected_system_fr
                self.main_dict[str(selected_system)]["Failure Rate"]["Sensor Based"] = selected_system_sb_fr

            selected_system_fr = self.main_dict[str(selected_system)]["Failure Rate"]["Nominal"]
            selected_system_reliability = self.r_calculation_class.reliability_calculate_func(self.phm_gui_time, selected_system_fr, self.selected_reliability_model, self.selected_reliability_unit, self.shape_parameter)
            self.main_dict[str(selected_system)]["Reliability"]["Nominal"] = selected_system_reliability

            selected_system_sb_fr = self.main_dict[str(selected_system)]["Failure Rate"]["Sensor Based"]
            selected_system_sb_reliability = self.r_calculation_class.reliability_calculate_func(self.phm_gui_time, selected_system_sb_fr, self.selected_reliability_model, self.selected_reliability_unit, self.shape_parameter)
            self.main_dict[str(selected_system)]["Reliability"]["Sensor Based"] = selected_system_sb_reliability

        except Exception as err:
            print("\nError: main_dict_module_calculate_func\n")
            print(err)


    def calculate_selected_module_fr_func(self, selected_system, object_keys, select_module_list, object_type, fr_type):
        """
            Selected Module Failure Rate Calculation Function
        """
        try:
            fr_list = list()
            calculate_fr = 0.0

            if fr_type:
                failure_rate_type = "Nominal"

            else:
                failure_rate_type = "Sensor Based"

            for item in select_module_list:
                if item in object_keys:
                    selected_fr = self.main_dict[str(selected_system)]['Configurations'][str(item)]['Failure Rate'][str(failure_rate_type)]

                    if selected_fr != None and selected_fr != "None":
                        fr_list.append(selected_fr)

                else:
                    fr_type_list = list(self.main_dict[str(selected_system)][str(item)]['Failure Rate'].keys())
                    fr_type_list.remove("Nominal")

                    if len(fr_type_list) > 0 and not fr_type:
                        selected_fr = self.main_dict[str(selected_system)][str(item)]['Failure Rate'][str(failure_rate_type)]

                    else:
                        selected_fr = self.main_dict[str(selected_system)][str(item)]['Failure Rate']["Nominal"]

                    if selected_fr != None and selected_fr != "None":
                        fr_list.append(selected_fr)

            if object_type == "Serial":
                calculate_fr = self.fr_calculation_class.component_serial_failure_rate_calculation(fr_list)

            else:
                calculate_fr = self.fr_calculation_class.component_parallel_failure_rate_calculation(fr_list)

            return calculate_fr

        except Exception as err:
            print("\nError: calculate_selected_module_fr_func\n")
            print(err)


    def main_dict_sub_module_calculate_func(self, selected_system, selected_module):
        """
            Sub Module Calculation Function
        """
        try:
            selected_module_dict = self.main_dict[str(selected_system)][str(selected_module)]
            selected_configuration = dict(selected_module_dict["Configurations"])
            del selected_configuration["Configuration Count"]
            control_dict = dict()

            if str(selected_configuration) != str(control_dict):
                module_depth_level = selected_configuration["Depth Level"]
                selected_final_configuration = selected_module_dict["Final Configuration"]

                configuration_list = list(selected_configuration.keys())
                self.remove_key(configuration_list)

                sub_module_list = list(selected_module_dict.keys())
                self.remove_key(sub_module_list)

                sorted_list = self.sorted_list_for_depth_level(selected_configuration, configuration_list, module_depth_level)

                # Alt katmandakilerin hesabi yapildi
                for sub_module in sub_module_list:
                    self.main_dict_component_calculate_func(selected_system, selected_module, sub_module)

                # Mevcut katmanin isi yapiliyor
                for item in sorted_list:
                    item_sub_module_list = selected_module_dict["Configurations"][str(item)]["Components"]
                    item_type = selected_module_dict["Configurations"][str(item)]["Type"]
                    calculate_fr = self.calculate_selected_sub_module_fr_func(selected_system, selected_module, configuration_list, item_sub_module_list, item_type)

                    self.main_dict[str(selected_system)][str(selected_module)]["Configurations"][str(item)]["Failure Rate"] = calculate_fr

                configuration_keys = list(self.main_dict[str(selected_system)][str(selected_module)]["Configurations"].keys())
                self.remove_key(configuration_keys)

                if selected_final_configuration not in configuration_keys:
                    if selected_final_configuration not in module_list:
                        self.main_dict[str(selected_system)][str(selected_module)]["Final Configuration"] = ""
                        selected_final_configuration == ""

                if selected_final_configuration == "":
                    selected_module_fr = 0.0

                else:
                    if selected_final_configuration in configuration_keys:
                        selected_module_fr = self.main_dict[str(selected_system)][str(selected_module)]["Configurations"][str(selected_final_configuration)]["Failure Rate"]

                    else:
                        selected_module_fr = self.main_dict[str(selected_system)][str(selected_module)][str(selected_final_configuration)]["Failure Rate"]

                self.main_dict[str(selected_system)][str(selected_module)]["Failure Rate"]["Nominal"] = selected_module_fr

            selected_module_fr = self.main_dict[str(selected_system)][str(selected_module)]["Failure Rate"]["Nominal"]
            selected_module_reliability = self.r_calculation_class.reliability_calculate_func(self.phm_gui_time, selected_module_fr, self.selected_reliability_model, self.selected_reliability_unit, self.shape_parameter)
            self.main_dict[str(selected_system)][str(selected_module)]["Reliability"]["Nominal"] = selected_module_reliability

            sensor_list = list(self.temperature_sensor_dict.keys())
            sensor_list.remove("System")

            if len(sensor_list) > 0 and str(selected_module) in sensor_list:
                module_temperature = float(self.temperature_sensor_dict[str(selected_module)]["Avarage"])
                selected_module_temperature_fr = self.fr_calculation_class.failure_rate_calculation_using_temperature_func(selected_module_fr, module_temperature, self.temperature_t0)
                self.main_dict[str(selected_system)][str(selected_module)]["Failure Rate"]["Sensor Based"] = selected_module_temperature_fr
                selected_module_temperature_reliability = self.r_calculation_class.reliability_calculate_func(self.phm_gui_time, selected_module_temperature_fr, self.selected_reliability_model, self.selected_reliability_unit, self.shape_parameter)
                self.main_dict[str(selected_system)][str(selected_module)]["Reliability"]["Sensor Based"] = selected_module_temperature_reliability
            else:
                try:
                    del self.main_dict[str(selected_system)][str(selected_module)]["Failure Rate"]["Sensor Based"]
                    del self.main_dict[str(selected_system)][str(selected_module)]["Reliability"]["Sensor Based"]

                except KeyError:
                    pass

        except Exception as err:
            print("\nError: main_dict_sub_module_calculate_func\n")
            print(err)


    def calculate_selected_sub_module_fr_func(self, selected_system, selected_module, object_keys, select_sub_module_list, object_type):
        """
            Selected Sub Module Failure Rate Calculation Function
        """
        try:
            fr_list = list()
            calculate_fr = 0.0

            for item in select_sub_module_list:
                if item in object_keys:
                    selected_fr = self.main_dict[str(selected_system)][str(selected_module)]['Configurations'][str(item)]['Failure Rate']

                    if selected_fr != None and selected_fr != "None":
                        fr_list.append(selected_fr)

                else:
                    selected_fr = self.main_dict[str(selected_system)][str(selected_module)][str(item)]['Failure Rate']

                    if selected_fr != None and selected_fr != "None":
                        fr_list.append(selected_fr)

            if object_type == "Serial":
                calculate_fr = self.fr_calculation_class.component_serial_failure_rate_calculation(fr_list)

            else:
                calculate_fr = self.fr_calculation_class.component_parallel_failure_rate_calculation(fr_list)

            return calculate_fr

        except Exception as err:
            print("\nError: calculate_selected_sub_module_fr_func\n")
            print(err)


    def main_dict_component_calculate_func(self, selected_system, selected_module, selected_sub_module):
        """
            Component Calculation Function
        """
        try:
            selected_sub_module_dict = self.main_dict[str(selected_system)][str(selected_module)][str(selected_sub_module)]
            selected_configuration = dict(selected_sub_module_dict["Configurations"])
            del selected_configuration["Configuration Count"]
            control_dict = dict()

            if str(selected_configuration) != str(control_dict):
                sub_module_depth_level = selected_configuration["Depth Level"]
                selected_final_configuration = selected_sub_module_dict["Final Configuration"]

                configuration_list = list(selected_configuration.keys())
                self.remove_key(configuration_list)

                sorted_list = self.sorted_list_for_depth_level(selected_configuration, configuration_list, sub_module_depth_level)

                for item in sorted_list:
                    item_component_list = selected_sub_module_dict["Configurations"][str(item)]["Components"]
                    item_type = selected_sub_module_dict["Configurations"][str(item)]["Type"]
                    calculate_fr = self.calculate_selected_component_fr_func(selected_system, selected_module, selected_sub_module, configuration_list, item_component_list, item_type)

                    self.main_dict[str(selected_system)][str(selected_module)][str(selected_sub_module)]["Configurations"][str(item)]["Failure Rate"] = calculate_fr

                configuration_keys = list(self.main_dict[str(selected_system)][str(selected_module)][str(selected_sub_module)]["Configurations"].keys())
                self.remove_key(configuration_keys)

                if selected_final_configuration not in configuration_keys:
                    if selected_final_configuration not in module_list:
                        self.main_dict[str(selected_system)][str(selected_module)][str(selected_sub_module)]["Final Configuration"] = ""
                        selected_final_configuration == ""

                if selected_final_configuration == "":
                    selected_sub_module_fr = 0.0

                else:
                    if selected_final_configuration in configuration_keys:
                        selected_sub_module_fr = self.main_dict[str(selected_system)][str(selected_module)][str(selected_sub_module)]["Configurations"][str(selected_final_configuration)]["Failure Rate"]

                    else:
                        selected_sub_module_fr = self.main_dict[str(selected_system)][str(selected_module)][str(selected_sub_module)][str(selected_final_configuration)]["Failure Rate"]

                self.main_dict[str(selected_system)][str(selected_module)][str(selected_sub_module)]["Failure Rate"] = selected_sub_module_fr

            selected_sub_module_fr = self.main_dict[str(selected_system)][str(selected_module)][str(selected_sub_module)]["Failure Rate"]
            selected_sub_module_reliability = self.r_calculation_class.reliability_calculate_func(self.phm_gui_time, selected_sub_module_fr, self.selected_reliability_model, self.selected_reliability_unit, self.shape_parameter)
            self.main_dict[str(selected_system)][str(selected_module)][str(selected_sub_module)]["Reliability"] = selected_sub_module_reliability

        except Exception as err:
            print("\nError: main_dict_component_calculate_func\n")
            print(err)


    def calculate_selected_component_fr_func(self, selected_system, selected_module, selected_sub_module, object_keys, select_component_list, object_type):
        """
            Selected Component Failure Rate Calculation Function
        """
        try:
            fr_list = list()
            calculate_fr = 0.0

            for item in select_component_list:
                if item in object_keys:
                    selected_fr = self.main_dict[str(selected_system)][str(selected_module)][str(selected_sub_module)]['Configurations'][str(item)]['Failure Rate']

                    if selected_fr != None and selected_fr != "None":
                        fr_list.append(selected_fr)

                else:
                    selected_fr = self.main_dict[str(selected_system)][str(selected_module)][str(selected_sub_module)][str(item)]['Failure Rate']

                    if selected_fr != None and selected_fr != "None":
                        fr_list.append(selected_fr)

            if object_type == "Serial":
                calculate_fr = self.fr_calculation_class.component_serial_failure_rate_calculation(fr_list)

            else:
                calculate_fr = self.fr_calculation_class.component_parallel_failure_rate_calculation(fr_list)

            return calculate_fr

        except Exception as err:
            print("\nError: calculate_selected_component_fr_func\n")
            print(err)

    @classmethod
    def sorted_list_for_depth_level(cls, configuration_dict, configuration_list, depth_level):
        """
            Sorted List for Depth Level Function
        """
        try:
            sorted_list = list()

            for i in range(depth_level + 1):
                for item in configuration_list:
                    depth = configuration_dict[str(item)]["Depth Level"]

                    if depth == i:
                        sorted_list.append(item)

            return sorted_list

        except Exception as err:
            print("\nError: sorted_list_for_depth_level\n")
            print(err)

# "PHM SYSTEM GENERAL CALCULATION FUNCTIONS" - END -

# "PHM GENERAL SET RESULT FUNCTIONS" - START -

    def general_set_result_func(self):
        """
            General Set Result Function
        """
        try:
            selected_failure_rate_type = str(self.comboBox_m_sys_select_failure_rate_type.currentText())
            selected_reliability_type = str(self.comboBox_m_sys_select_reliability_type.currentText())
            selected_module_reliability = str(self.comboBox_m_reliability_module.currentText())
            selected_module_reliability_type = str(self.comboBox_m_sys_select_module_reliability_type.currentText())

            if selected_failure_rate_type != "" and selected_failure_rate_type != "None":
                system_failure_rate = self.main_dict["System"]["Failure Rate"][str(selected_failure_rate_type)]
                self.lineEdit_m_sys_lambda.setText(str(system_failure_rate))

            if selected_reliability_type != "" and selected_reliability_type != "None":
                system_reliability = self.main_dict["System"]["Reliability"][str(selected_reliability_type)]
                self.lineEdit_m_sys_reliability.setText(str(system_reliability))

            if selected_module_reliability != "" and selected_module_reliability != "None" and selected_module_reliability_type != "" and selected_module_reliability_type != "None":
                module_reliability = self.main_dict["System"][str(selected_module_reliability)]['Reliability'][str(selected_module_reliability_type)]
                self.lineEdit_m_module_reliability.setText(str(module_reliability))

            else:
                module_reliability = 1.0
                self.lineEdit_m_module_reliability.setText(str(module_reliability))

            self.phm_gui_time += 1

        except Exception as err:
            print("\nError: general_set_result_func\n")
            print(err)


    def configuration_setup_set_result_func(self):
        """
            Configuration Setup Set Result Function
        """
        self.configuration_setup_sub_module_set_result_func()
        self.configuration_setup_module_set_result_func()
        self.configuration_setup_system_set_result_func()


    def configuration_setup_sub_module_set_result_func(self):
        """
            Configuration Setup Sub Module Set Result Function
        """
        try:
            selected_system = str(self.comboBox_configuration_st_sc_select_system.currentText())
            selected_module = str(self.comboBox_configuration_st_sc_select_module.currentText())
            selected_sub_module = str(self.comboBox_configuration_st_sc_select_sub_module.currentText())

            selected_first_component = str(self.comboBox_configuration_select_first_comp.currentText())
            selected_second_component = str(self.comboBox_configuration_select_second_comp.currentText())

            if selected_system != "" and selected_system != "None" and selected_module != "" and selected_module != "None" and selected_sub_module != "" and selected_sub_module != "None":
                selected_fr = self.main_dict[str(selected_system)][str(selected_module)][selected_sub_module]["Failure Rate"]
                self.lineEdit_configuration_failure_rate_sub_module.setText(str(selected_fr))

            if selected_first_component != "" and selected_first_component != "None":
                configuration_list = self.control_of_configuration_list(selected_system, selected_module, selected_sub_module)

                if selected_first_component in configuration_list:
                    selected_fr = self.main_dict[str(selected_system)][str(selected_module)][selected_sub_module]["Configurations"][str(selected_first_component)]["Failure Rate"]

                else:
                    selected_fr = self.main_dict[str(selected_system)][str(selected_module)][selected_sub_module][str(selected_first_component)]["Failure Rate"]

                self.lineEdit_configuration_failure_rate_first_component.setText(str(selected_fr))

            if selected_second_component != "" and selected_second_component != "None":
                configuration_list = self.control_of_configuration_list(selected_system, selected_module, selected_sub_module)

                if selected_second_component in configuration_list:
                    selected_fr = self.main_dict[str(selected_system)][str(selected_module)][selected_sub_module]["Configurations"][str(selected_second_component)]["Failure Rate"]

                else:
                    selected_fr = self.main_dict[str(selected_system)][str(selected_module)][selected_sub_module][str(selected_second_component)]["Failure Rate"]

                self.lineEdit_configuration_failure_rate_second_component.setText(str(selected_fr))

        except Exception as err:
            print("\nError: configuration_setup_sub_module_set_result_func\n")
            print(err)


    def configuration_setup_module_set_result_func(self):
        """
            Configuration Setup Module Set Result Function
        """
        try:
            selected_system = str(self.comboBox_configuration_st_ssm_select_system.currentText())
            selected_module = str(self.comboBox_configuration_st_ssm_select_module.currentText())

            selected_first_sub_module = str(self.comboBox_configuration_select_first_sub_module.currentText())
            selected_second_sub_module = str(self.comboBox_configuration_select_second_sub_module.currentText())

            if selected_system != "" and selected_system != "None" and selected_module != "" and selected_module != "None":
                selected_fr = self.main_dict[str(selected_system)][str(selected_module)]["Failure Rate"]["Nominal"]
                self.lineEdit_configuration_failure_rate_module.setText(str(selected_fr))

            if selected_first_sub_module != "" and selected_first_sub_module != "None":
                configuration_list = self.control_of_configuration_list(selected_system, selected_module)

                if selected_first_sub_module in configuration_list:
                    selected_fr = self.main_dict[str(selected_system)][str(selected_module)]["Configurations"][str(selected_first_sub_module)]["Failure Rate"]

                else:
                    selected_fr = self.main_dict[str(selected_system)][str(selected_module)][str(selected_first_sub_module)]["Failure Rate"]

                self.lineEdit_configuration_failure_rate_first_sub_module.setText(str(selected_fr))

            if selected_second_sub_module != "" and selected_second_sub_module != "None":
                configuration_list = self.control_of_configuration_list(selected_system, selected_module)

                if selected_second_sub_module in configuration_list:
                    selected_fr = self.main_dict[str(selected_system)][str(selected_module)]["Configurations"][str(selected_second_sub_module)]["Failure Rate"]

                else:
                    selected_fr = self.main_dict[str(selected_system)][str(selected_module)][str(selected_second_sub_module)]["Failure Rate"]

                self.lineEdit_configuration_failure_rate_second_sub_module.setText(str(selected_fr))

        except Exception as err:
            print("\nError: configuration_setup_module_set_result_func\n")
            print(err)


    def configuration_setup_system_set_result_func(self):
        """
            Configuration Setup System Set Result Function
        """
        try:
            selected_system = str(self.comboBox_configuration_st_sm_select_system.currentText())

            selected_first_module = str(self.comboBox_configuration_select_first_module.currentText())
            selected_second_module = str(self.comboBox_configuration_select_second_module.currentText())

            if selected_system != "" and selected_system != "None":
                selected_fr = self.main_dict[str(selected_system)]["Failure Rate"]["Nominal"]
                self.lineEdit_configuration_failure_rate_system.setText(str(selected_fr))

            if selected_first_module != "" and selected_first_module != "None":
                configuration_list = self.control_of_configuration_list(selected_system)

                if selected_first_module in configuration_list:
                    selected_fr = self.main_dict[str(selected_system)]["Configurations"][str(selected_first_module)]["Failure Rate"]["Nominal"]

                else:
                    selected_fr = self.main_dict[str(selected_system)][str(selected_first_module)]["Failure Rate"]["Nominal"]

                self.lineEdit_configuration_failure_rate_first_module.setText(str(selected_fr))

            if selected_second_module != "" and selected_second_module != "None":
                configuration_list = self.control_of_configuration_list(selected_system)

                if selected_second_module in configuration_list:
                    selected_fr = self.main_dict[str(selected_system)]["Configurations"][str(selected_second_module)]["Failure Rate"]["Nominal"]

                else:
                    selected_fr = self.main_dict[str(selected_system)][str(selected_second_module)]["Failure Rate"]["Nominal"]

                self.lineEdit_configuration_failure_rate_second_module.setText(str(selected_fr))

        except Exception as err:
            print("\nError: configuration_setup_system_set_result_func\n")
            print(err)

# "PHM GENERAL SET RESULT FUNCTIONS" - END -

# "PHM GENERAL SENSORS FUNCTIONS" - START -

    def phm_temperature_sensor_thread_func(self):
        """
            Phm Temperature Sensor Thread Function
        """
        general_key_list = list(self.temperature_sensor_dict.keys())

        for item in general_key_list:
            selected_sensors_number = len(list(self.temperature_sensor_dict[str(item)]["Sensors"].keys()))
            result = 0

            if selected_sensors_number > 0:
                for key, value in self.temperature_sensor_dict[str(item)]["Sensors"].items():
                    if str(value["Thread"].data) != "None":
                        self.temperature_sensor_dict[str(item)]["Sensors"][str(key)]["Value"] = float(value["Thread"].data)
                        result += float(value["Thread"].data)

                if selected_sensors_number > 1:
                    self.temperature_sensor_dict[str(item)]["Avarage"] = float(result / selected_sensors_number)

                else:
                    self.temperature_sensor_dict[str(item)]["Avarage"] = float(result)

# "PHM GENERAL SENSORS FUNCTIONS" - END -

# "CONTROL FUNCTIONS OF INDEX CHANGES" - START -

  # CONFIGURATION // SET TYPE

    def configuration_set_type_events_func(self):
        """
            Configuration Set Type Events Func
        """
     # GENERAL FUNCTIONS
        self.system_list_tab_func()
        self.module_list_tab_func()
        self.sub_module_list_tab_func()
        self.combobox_configuration_type()

     # SET MODULE TYPES TAB CONFIGURATION
        self.first_module_list_tab_func()
        self.second_module_list_tab_func()
        self.module_list_widget_func()

        self.comboBox_configuration_st_sm_select_system.currentIndexChanged.connect(self.system_list_control_of_set_module_types_tab)
        self.comboBox_configuration_st_sm_select_system.currentIndexChanged.connect(self.system_fr_control)
        self.comboBox_configuration_st_sm_select_system.currentIndexChanged.connect(self.convert_configurations_of_modules_into_syntax)
        self.comboBox_configuration_select_first_module.currentIndexChanged.connect(self.module_list_control_of_set_module_types_tab)
        self.comboBox_configuration_select_second_module.currentIndexChanged.connect(self.module_list_control_of_set_module_types_tab)
        self.comboBox_configuration_reliability_model.currentIndexChanged.connect(self.select_reliability_model_func)
        self.comboBox_configuration_unit.currentIndexChanged.connect(self.select_phm_unit_func)

        self.pushButton_configuration_set_module_type.clicked.connect(self.create_syntax_of_modules)
        self.pushButton_delete_module_config.clicked.connect(self.delete_module_configuration)
        self.pushButton_add_module_config.clicked.connect(self.add_module_into_config)
        self.pushButton_select_final_configuration_of_system.clicked.connect(self.select_final_configuration_of_system)
        self.lineEdit_configuration_reliability_shape_parameter.textChanged.connect(self.set_shape_parameter_func)

     # SET SUB-MODULE TYPES TAB CONFIGURATION
        self.first_sub_module_list_tab_func()
        self.second_sub_module_list_tab_func()
        self.sub_module_list_widget_func()

        # Robot Design Set failure rate from default equipments type
        self.comboBox_c_ao_am_select_type.addItems(self.type_list)
        self.comboBox_c_ao_asm_sfr_select_type.addItems(self.type_list)
        self.comboBox_c_ao_asm_select_type.addItems(self.type_list)
        self.comboBox_configuration_st_ssm_select_system.currentIndexChanged.connect(self.module_list_control_of_set_sub_module_types_tab)
        self.comboBox_configuration_st_ssm_select_module.currentIndexChanged.connect(self.sub_module_list_control_of_set_sub_module_types_tab)
        self.comboBox_configuration_st_ssm_select_module.currentIndexChanged.connect(self.module_fr_control)
        self.comboBox_configuration_st_ssm_select_module.currentIndexChanged.connect(self.convert_configurations_of_sub_modules_into_syntax)
        self.comboBox_configuration_select_first_sub_module.currentIndexChanged.connect(self.first_sub_module_list_control_of_set_sub_module_types_tab)
        self.comboBox_configuration_select_second_sub_module.currentIndexChanged.connect(self.second_sub_module_list_control_of_set_sub_module_types_tab)

        self.pushButton_configuration_set_sub_module_type.clicked.connect(self.create_syntax_of_sub_modules)
        self.pushButton_delete_sub_module_config.clicked.connect(self.delete_sub_module_configuration)
        self.pushButton_add_sub_module_config.clicked.connect(self.add_sub_module_into_config)
        self.pushButton_select_final_configuration_of_module.clicked.connect(self.select_final_configuration_of_module)

     # SET COMPONENT TYPES TAB CONFIGURATION
        self.first_component_list_tab_func()
        self.second_component_list_tab_func()
        self.component_list_widget_func()

        self.comboBox_configuration_st_sc_select_system.currentIndexChanged.connect(self.module_list_control_of_set_component_types_tab)
        self.comboBox_configuration_st_sc_select_module.currentIndexChanged.connect(self.sub_module_list_control_of_set_component_types_tab)
        self.comboBox_configuration_st_sc_select_sub_module.currentIndexChanged.connect(self.component_list_control_of_set_component_types_tab)
        self.comboBox_configuration_st_sc_select_sub_module.currentIndexChanged.connect(self.sub_module_fr_control)
        self.comboBox_configuration_st_sc_select_sub_module.currentIndexChanged.connect(self.convert_configurations_of_components_into_syntax)
        self.comboBox_configuration_select_first_comp.currentIndexChanged.connect(self.first_component_list_control_of_set_component_types_tab)
        self.comboBox_configuration_select_second_comp.currentIndexChanged.connect(self.second_component_list_control_of_set_component_types_tab)

        self.pushButton_configuration_set_comp_type.clicked.connect(self.create_syntax_of_components)
        self.pushButton_delete_component_config.clicked.connect(self.delete_component_configuration)
        self.pushButton_add_component_config.clicked.connect(self.add_component_into_config)
        self.pushButton_select_final_configuration_of_sub_module.clicked.connect(self.select_final_configuration_of_sub_module)

  # CONFIGURATION // ADD OBJECT 2
    def configuration_add_object_2_tab_events_func(self):
        """
            Configuration Add Object 2 Tab Events Func
        """
     # ADD MODULE TAB CONFIGURATION
        self.pushButton_c_ao_am_add_module.clicked.connect(self.add_new_module)
        self.pushButton_c_ao_am_delete_module.clicked.connect(self.delete_a_module)
        self.lineEdit_c_ao_am_module_fr.textChanged.connect(self.calculate_set_fr_module_total_fr_func)
        self.spinBox_c_ao_am_quantity.valueChanged.connect(self.calculate_set_fr_module_total_fr_func)
        self.comboBox_c_ao_am_select_type.currentIndexChanged.connect(self.calculate_set_fr_module_total_fr_func)

     # ADD SUB MODULE TAB CONFIGURATION
        self.pushButton_c_ao_asm_add_sub_module.clicked.connect(self.add_new_sub_module)
        self.comboBox_c_ao_asm_select_module.currentIndexChanged.connect(self.add_sub_module_into_list_widget)
        self.lineEdit_c_ao_asm_sub_module_fr.textChanged.connect(self.calculate_set_fr_sub_module_total_fr_func)
        self.spinBox_c_ao_asm_sfr_quantity.valueChanged.connect(self.calculate_set_fr_sub_module_total_fr_func)
        self.comboBox_c_ao_asm_sfr_select_type.currentIndexChanged.connect(self.calculate_set_fr_sub_module_total_fr_func)
        self.groupBox_c_ao_asm_sub_module_fr.clicked.connect(self.sub_module_set_fr_groupbox_control)
        self.groupBox_c_ao_asm_default_fr.clicked.connect(self.sub_module_default_equipments_groupbox_control)
        self.lineEdit_c_ao_asm_component_fr.textChanged.connect(self.calculate_sub_module_total_fr_func)
        self.spinBox_c_ao_asm_quantity.valueChanged.connect(self.calculate_sub_module_total_fr_func)
        self.comboBox_c_ao_asm_select_type.currentIndexChanged.connect(self.calculate_sub_module_total_fr_func)
        self.pushButton_c_ao_asm_set_equipment.clicked.connect(self.sub_module_add_equipment_func)
        self.pushButton_c_ao_asm_delete_sub_module.clicked.connect(self.delete_a_sub_module)

     # ADD COMPONENT TAB CONFIGURATION
        self.comboBox_c_ao_ac_select_module.currentIndexChanged.connect(self.add_component_tab_func)
        self.pushButton_c_ao_ac_add_component.clicked.connect(self.add_new_component)
        self.lineEdit_c_ao_ac_component_fr.textChanged.connect(self.calculate_set_fr_component_total_fr_func)
        self.spinBox_c_ao_ac_sfr_quantity.valueChanged.connect(self.calculate_set_fr_component_total_fr_func)
        self.groupBox_c_ao_ac_component_fr.clicked.connect(self.component_set_fr_groupbox_control)
        self.groupBox_c_ao_ac_default_fr.clicked.connect(self.component_default_equipments_groupbox_control)
        self.comboBox_c_ao_ac_select_module.currentIndexChanged.connect(self.add_component_into_list_widget)
        self.comboBox_c_ao_ac_select_sub_module.currentIndexChanged.connect(self.add_component_into_list_widget)
        self.lineEdit_c_ao_ac_component_fr_2.textChanged.connect(self.calculate_component_total_fr_func)
        self.spinBox_c_ao_ac_quantity.valueChanged.connect(self.calculate_component_total_fr_func)
        self.pushButton_c_ao_ac_set_equipment.clicked.connect(self.component_add_equipment_func)
        self.pushButton_c_ao_ac_delete_component.clicked.connect(self.delete_a_component)

     # ADD COMPONENT ELEMENT TAB CONFIGURATION
        self.comboBox_c_ao_ace_select_module.currentIndexChanged.connect(self.add_component_element_tab_func)
        self.comboBox_c_ao_ace_select_sub_module.currentIndexChanged.connect(self.add_component_element_tab_func_2)
        self.pushButton_c_ao_ace_add_component_element.clicked.connect(self.add_new_custom_component)
        self.comboBox_c_ao_ace_select_module.currentIndexChanged.connect(self.add_elements_into_list_widget)
        self.comboBox_c_ao_ace_select_sub_module.currentIndexChanged.connect(self.add_elements_into_list_widget)
        self.comboBox_c_ao_ace_select_component.currentIndexChanged.connect(self.add_elements_into_list_widget)
        self.pushButton_c_ao_ac_set_formula.clicked.connect(self.set_formula_of_component)
        self.pushButton_c_ao_ac_set_formula.clicked.connect(self.add_component_into_list_widget)
        self.comboBox_c_ao_ace_select_component_2.currentIndexChanged.connect(self.control_component_formula)
        self.pushButton_c_ao_ace_delete_element.clicked.connect(self.delete_an_element)
        self.pushButton_c_ao_ace_delete_element.clicked.connect(self.add_component_into_list_widget)

  # START PHM - SAVE and LOAD
    def start_phm_save_and_load_buttons_control_func(self):
        """
            PHM Save And Load Button Function
        """
        self.pushButton_c_ao_save.clicked.connect(self.save_system_into_file)
        self.pushButton_c_ao_load.clicked.connect(self.load_system_from_file)

# "CONTROL FUNCTIONS OF INDEX CHANGES" - END -

# CONTROL OF CONFIGURATION LIST - START -

    def control_of_configuration_list(self, *args):
        """
            Control of Configuration List Function
        """
        if len(args) == 3:
            configuration_list = list(self.main_dict[str(args[0])][str(args[1])][str(args[2])]["Configurations"].keys())

        elif len(args) == 2:
            configuration_list = list(self.main_dict[str(args[0])][str(args[1])]["Configurations"].keys())

        elif len(args) == 1:
            configuration_list = list(self.main_dict[str(args[0])]["Configurations"].keys())

        else:
            print("\nThe function must have at least 1 input.\n")

        self.remove_key(configuration_list)

        return configuration_list

# CONTROL OF CONFIGURATION LIST - END -

# REMOVE KEY FUNCTIONS - START -

    def remove_key(self, temp_list):
        """
            Remove Key Function
        """
        self.remove_fr_key(temp_list)
        self.remove_reliability_key(temp_list)
        self.remove_configurations_key(temp_list)
        self.remove_elements_key(temp_list)
        self.remove_formula_key(temp_list)
        self.remove_key_final_config(temp_list)
        self.remove_depth_level_key(temp_list)
        self.remove_configuration_count_key(temp_list)
        temp_list.sort()

    @classmethod
    def remove_configuration_count_key(cls, temp_list):
        """
            Remove "Configuration Count" Key Function
        """
        if "Configuration Count" in temp_list:
            temp_list.remove("Configuration Count")

        else:
            pass

    @classmethod
    def remove_fr_key(cls, temp_list):
        """
            Remove "Failure Rate Key" Function
        """
        if "Failure Rate" in temp_list:
            temp_list.remove("Failure Rate")

        else:
            pass

    @classmethod
    def remove_reliability_key(cls, temp_list):
        """
            Remove "Reliability" Function
        """
        if "Reliability" in temp_list:
            temp_list.remove("Reliability")

        else:
            pass

    @classmethod
    def remove_depth_level_key(cls, temp_list):
        """
            Remove "Depth Level" Function
        """
        if "Depth Level" in temp_list:
            temp_list.remove("Depth Level")

        else:
            pass

    @classmethod
    def remove_configurations_key(cls, temp_list):
        """
            Remove "Configurations" Function
        """
        if "Configurations" in temp_list:
            temp_list.remove("Configurations")

        else:
            pass

    @classmethod
    def remove_elements_key(cls, temp_list):
        """
            Remove "Elements" Function
        """
        if "Elements" in temp_list:
            temp_list.remove("Elements")

        else:
            pass

    @classmethod
    def remove_formula_key(cls, temp_list):
        """
            Remove "Formula" Function
        """
        if "Formula" in temp_list:
            temp_list.remove("Formula")

        else:
            pass

    @classmethod
    def remove_key_final_config(cls, temp_list):
        """
            Remove "Final Configuration" Function
        """
        if "Final Configuration" in temp_list:
            temp_list.remove("Final Configuration")

        else:
            pass

# REMOVE KEY FUNCTIONS - END -

# PHM START MAIN DICT - START -

    def start_phm_gui_main_dict_func(self):
        """
            Start PHM Gui Main Dict Function
        """
        try:
            current_workspace = self.get_current_workspace()
            file_dir = "phm_start/params/"
            temp_file_name = "last_configurations"
            file_name = str(str(current_workspace) + str(file_dir) + str(temp_file_name) + '.yaml')

            last_saved_dict = dict(self.read_data_from_file(file_name))
            last_saved_dict_keys = list(last_saved_dict["System"].keys())
            self.remove_key(last_saved_dict_keys)

            if last_saved_dict_keys:
                self.main_dict["System"].update(last_saved_dict["System"])
                self.add_module_into_am_list_widget()
                self.add_module_into_combobox()

            else:
                self.main_dict["System"] = {"Reliability": {"Nominal": 0.0, "Sensor Based": 0.0}, "Failure Rate": {"Nominal": 0.0, "Sensor Based": 0.0}, "Configurations": {'Configuration Count': {'Serial': 0, 'Parallel': 0}}, "Final Configuration": str()}

        except Exception:
            self.main_dict["System"] = {"Reliability": {"Nominal": 0.0, "Sensor Based": 0.0}, "Failure Rate": {"Nominal": 0.0, "Sensor Based": 0.0}, "Configurations": {'Configuration Count': {'Serial': 0, 'Parallel': 0}}, "Final Configuration": str()}

# PHM START MAIN DICT - END -

# "CONFIGURATION // SET TYPE GENERAL FUNCTIONS" - START -

    def system_list_tab_func(self):
        """
            System List Tab Function
        """
        # Set Module tab
        self.comboBox_configuration_st_sm_select_system.clear()
        self.comboBox_configuration_st_sm_select_system.addItem("None")

        # Set Sub-Module tab
        self.comboBox_configuration_st_ssm_select_system.clear()
        self.comboBox_configuration_st_ssm_select_system.addItem("None")

        # Set Component tab
        self.comboBox_configuration_st_sc_select_system.clear()
        self.comboBox_configuration_st_sc_select_system.addItem("None")

        system_list = list(self.main_dict.keys())
        self.remove_key(system_list)

        self.comboBox_configuration_st_sm_select_system.addItems(sorted(system_list))
        self.comboBox_configuration_st_ssm_select_system.addItems(sorted(system_list))
        self.comboBox_configuration_st_sc_select_system.addItems(sorted(system_list))


    def module_list_tab_func(self):
        """
            Module List Tab Function
        """
        # Set Sub-Module tab
        self.comboBox_configuration_st_ssm_select_module.clear()
        self.comboBox_configuration_st_ssm_select_module.addItem("None")

        # Set Component tab
        self.comboBox_configuration_st_sc_select_module.clear()
        self.comboBox_configuration_st_sc_select_module.addItem("None")


    def sub_module_list_tab_func(self):
        """
            Sub Module List Tab Function
        """
        # Set Component tab
        self.comboBox_configuration_st_sc_select_sub_module.clear()
        self.comboBox_configuration_st_sc_select_sub_module.addItem("None")


    def combobox_configuration_type(self):
        """
            Configuration Type Function
        """
        # Set Module tab
        self.comboBox_configuration_type_of_module.clear()
        self.comboBox_configuration_type_of_module.addItems(self.type_list)

        # Set Sub-Module tab
        self.comboBox_configuration_type_of_sub_module.clear()
        self.comboBox_configuration_type_of_sub_module.addItems(self.type_list)

        # Set Component tab
        self.comboBox_configuration_type_of_component.clear()
        self.comboBox_configuration_type_of_component.addItems(self.type_list)

    @classmethod
    def check_type(cls, configuration_all):
        """
            Check Type Function
        """
        if 'series' in configuration_all:
            type_value = 'series'

        elif 'parallel' in configuration_all:
            type_value = 'parallel'

        return type_value

# "CONFIGURATION // SET TYPE GENERAL FUNCTIONS" - END -

# "SAVE - LOAD BUTTONS CONTROL" - START -

    def save_system_into_file(self):
        """
            Save System Into File Function
        """
        main_dict_keys = list(self.main_dict["System"].keys())
        self.remove_key(main_dict_keys)

        if main_dict_keys:
            file_dir = "phm_start/params/"
            now = datetime.now()
            dt_string = now.strftime("%Y_%m_%d_-_%H_%M_%S")
            temp_file_name = str("phm_configurations_" + str(dt_string))
            temp_data = self.main_dict

            self.fill_the_file_with_data(file_dir, temp_file_name, temp_data)
            self.pushButton_c_ao_load.setEnabled(True)

        else:
            print("\nPlease enter the data.\n")


    def load_system_from_file(self):
        """
            Load System From File Function
        """
        try:
            select_yaml_files = SelectFileGui.get_file_path(is_app=True, caption="Select yaml file", filefilter="*yaml")

            if not select_yaml_files:
                print("\nError: Please select a yaml file.\n")
                sys.exit()

            selected_file_list = str(select_yaml_files[0]).split("'")
            selected_file = selected_file_list[1]
            self.configuration_loaded_object_dict = dict(self.read_data_from_file(selected_file))
            temp_loaded_object_keys = list(self.configuration_loaded_object_dict["System"].keys())

            if temp_loaded_object_keys:
                self.main_dict["System"].update(self.configuration_loaded_object_dict["System"])

                self.add_module_into_am_list_widget()
                self.add_module_into_combobox()
                self.pushButton_c_ao_load.setEnabled(False)
                self.monitoring_general_tab_func()
                self.general_set_result_func()

            else:
                print("\nPlease enter the data.\n")

        except Exception as err:
            print("\nPlease enter the data.\n")
            print(err)

# "SAVE - LOAD BUTTONS CONTROL" - END -

# "CONFIGURATION // SET TYPE // SET COMPONENT TYPES FUNCTIONS" - START -

    def first_component_list_tab_func(self):
        """
            First Component List Tab Function
        """
        self.comboBox_configuration_select_first_comp.clear()
        self.comboBox_configuration_select_first_comp.addItem("None")


    def second_component_list_tab_func(self):
        """
            Second Component List Tab Function
        """
        self.comboBox_configuration_select_second_comp.clear()
        self.comboBox_configuration_select_second_comp.addItem("None")


    def component_list_widget_func(self):
        """
            Component List Widget Function
        """
        self.listWidget_components.clear()
        self.listWidget_configurations_of_components.clear()


    def module_list_control_of_set_component_types_tab(self):
        """
            Module List Control of Set Component Types Tab Function
        """
        selected_system = self.comboBox_configuration_st_sc_select_system.currentText()
        self.comboBox_configuration_st_sc_select_module.clear()

        if selected_system != "None" and selected_system != "":
            module_list = list(self.main_dict[str(selected_system)].keys())
            self.remove_key(module_list)
            self.comboBox_configuration_st_sc_select_module.addItems(module_list)


    def sub_module_list_control_of_set_component_types_tab(self):
        """
            Sub Module List Control of Set Component Types Tab Function
        """
        selected_system = self.comboBox_configuration_st_sc_select_system.currentText()
        selected_module = self.comboBox_configuration_st_sc_select_module.currentText()

        self.comboBox_configuration_st_sc_select_sub_module.clear()

        if selected_system != "None" and selected_module != "None" and selected_system != "" and selected_module != "":
            sub_module_list = list(self.main_dict[str(selected_system)][str(selected_module)].keys())
            self.remove_key(sub_module_list)
            self.comboBox_configuration_st_sc_select_sub_module.addItems(sub_module_list)


    def component_list_control_of_set_component_types_tab(self):
        """
            Component List Control of Set Component Types Tab Function
        """
        selected_system = self.comboBox_configuration_st_sc_select_system.currentText()
        selected_module = self.comboBox_configuration_st_sc_select_module.currentText()
        selected_sub_module = self.comboBox_configuration_st_sc_select_sub_module.currentText()

        self.comboBox_configuration_select_first_comp.clear()
        self.comboBox_configuration_select_second_comp.clear()
        self.lineEdit_configuration_failure_rate_first_component.clear()
        self.lineEdit_configuration_failure_rate_second_component.clear()
        self.lineEdit_configuration_first_component.clear()
        self.lineEdit_configuration_second_component.clear()
        self.listWidget_components.clear()

        if ((selected_system and selected_module and selected_sub_module) != "None") and ((selected_system and selected_module and selected_sub_module) != ""):
            component_list = list(self.main_dict[str(selected_system)][str(selected_module)][str(selected_sub_module)].keys())
            self.remove_key(component_list)
            configuration_list = list(self.main_dict[str(selected_system)][str(selected_module)][str(selected_sub_module)]['Configurations'].keys())
            self.remove_key(configuration_list)
            component_list.extend(configuration_list)

            self.comboBox_configuration_select_first_comp.addItems(component_list)
            self.comboBox_configuration_select_second_comp.addItems(component_list)
            self.listWidget_components.addItems(sorted(component_list))

        else:
            self.comboBox_configuration_select_first_comp.clear()
            self.comboBox_configuration_select_second_comp.clear()
            self.listWidget_components.clear()


    def sub_module_fr_control(self):
        """
            Sub Module Failure Rate Control Function
        """
        self.lineEdit_configuration_failure_rate_sub_module.clear()
        selected_system = self.comboBox_configuration_st_sc_select_system.currentText()
        selected_module = self.comboBox_configuration_st_sc_select_module.currentText()
        selected_sub_module = self.comboBox_configuration_st_sc_select_sub_module.currentText()

        if ((selected_system and selected_module and selected_sub_module) != "") and ((selected_system and selected_module and selected_sub_module) != "None"):
            selected_sub_module_fr = self.main_dict[str(selected_system)][str(selected_module)][str(selected_sub_module)]['Failure Rate']
            self.lineEdit_configuration_failure_rate_sub_module.setText(str(selected_sub_module_fr))


    def convert_configurations_of_components_into_syntax(self):
        """
            Convert Configurations of Components Into Syntax Function
        """
        selected_system = self.comboBox_configuration_st_sc_select_system.currentText()
        selected_module = self.comboBox_configuration_st_sc_select_module.currentText()
        selected_sub_module = self.comboBox_configuration_st_sc_select_sub_module.currentText()

        self.listWidget_configurations_of_components.clear()

        if ((selected_system and selected_module and selected_sub_module) != "") and ((selected_system and selected_module and selected_sub_module) != "None"):
            configuraiton_list = list(self.main_dict[str(selected_system)][str(selected_module)][str(selected_sub_module)]["Configurations"].keys())
            self.remove_key(configuraiton_list)
            syntax_list = list()

            for configuration in configuraiton_list:
                type_value = self.main_dict[str(selected_system)][str(selected_module)][str(selected_sub_module)]["Configurations"][str(configuration)]["Type"]
                component_list = list(self.main_dict[str(selected_system)][str(selected_module)][str(selected_sub_module)]["Configurations"][str(configuration)]["Components"])
                syntax_name = str(configuration)

                if type_value == "Serial":
                    temp_syntax = str()

                    for component in component_list:
                        temp_syntax += str(component) + ', '

                    length = len(temp_syntax)
                    temp_syntax = temp_syntax[:length-2]
                    syntax = str(syntax_name) + ' = ' + 'series(' + str(temp_syntax) + ')'

                elif type_value == 'Parallel':
                    temp_syntax = str()

                    for component in component_list:
                        temp_syntax += str(component) + ', '

                    length = len(temp_syntax)
                    temp_syntax = temp_syntax[:length-2]
                    syntax = str(syntax_name) + ' = ' + 'parallel(' + str(temp_syntax) + ')'

                syntax_list.append(syntax)

            self.listWidget_configurations_of_components.addItems(sorted(syntax_list))


    def first_component_list_control_of_set_component_types_tab(self):
        """
            First Component List Control of Set Component Types Tab Function
        """
        selected_system = self.comboBox_configuration_st_sc_select_system.currentText()
        selected_module = self.comboBox_configuration_st_sc_select_module.currentText()
        selected_sub_module = self.comboBox_configuration_st_sc_select_sub_module.currentText()

        if ((selected_system and selected_sub_module and selected_module) != "None") and ((selected_system and selected_sub_module and selected_module) != ""):
            component1 = self.comboBox_configuration_select_first_comp.currentText()
            configuration_list = self.control_of_configuration_list(selected_system, selected_module, selected_sub_module)

            if component1 != "None" and component1 != "":
                self.lineEdit_configuration_first_component.setText(str(component1))

                if component1 in configuration_list:
                    component1_fr = self.main_dict[str(selected_system)][str(selected_module)][selected_sub_module]["Configurations"][str(component1)]["Failure Rate"]

                else:
                    component1_fr = self.main_dict[str(selected_system)][str(selected_module)][selected_sub_module][str(component1)]['Failure Rate']

                self.lineEdit_configuration_failure_rate_first_component.setText(str(component1_fr))


    def second_component_list_control_of_set_component_types_tab(self):
        """
            Second Component List Control of Set Component Types Tab Function
        """
        selected_system = self.comboBox_configuration_st_sc_select_system.currentText()
        selected_module = self.comboBox_configuration_st_sc_select_module.currentText()
        selected_sub_module = self.comboBox_configuration_st_sc_select_sub_module.currentText()

        if ((selected_system and selected_sub_module and selected_module) != "None") and ((selected_system and selected_sub_module and selected_module) != ""):
            component2 = self.comboBox_configuration_select_second_comp.currentText()
            configuration_list = self.control_of_configuration_list(selected_system, selected_module, selected_sub_module)

            if component2 != "None" and component2 != "":
                self.lineEdit_configuration_second_component.setText(str(component2))

                if component2 in configuration_list:
                    component2_fr = self.main_dict[str(selected_system)][str(selected_module)][selected_sub_module]["Configurations"][str(component2)]["Failure Rate"]

                else:
                    component2_fr = self.main_dict[str(selected_system)][str(selected_module)][selected_sub_module][str(component2)]['Failure Rate']

                self.lineEdit_configuration_failure_rate_second_component.setText(str(component2_fr))


    def create_syntax_of_components(self):
        """
            Create Syntax of Components Function
        """
        type_value = self.comboBox_configuration_type_of_component.currentText()
        selected_system = self.comboBox_configuration_st_sc_select_system.currentText()
        selected_module = self.comboBox_configuration_st_sc_select_module.currentText()
        selected_sub_module = self.comboBox_configuration_st_sc_select_sub_module.currentText()
        component1 = self.lineEdit_configuration_first_component.text()
        component2 = self.lineEdit_configuration_second_component.text()
        level = self.calculate_depth_level_of_component_configs()

        if type_value != "":
            new_config_name = str()
            self.main_dict[str(selected_system)][str(selected_module)][str(selected_sub_module)]["Configurations"]["Configuration Count"][str(type_value)] += 1
            selected_configuration_count = self.main_dict[str(selected_system)][str(selected_module)][str(selected_sub_module)]["Configurations"]["Configuration Count"][str(type_value)]

            if type_value == 'Serial':
                s_name = 's' + str(selected_configuration_count)
                new_config_name = s_name
                s_content = 'series(' + str(component1) + ', ' + str(component2) + ')'
                serial_fr = self.calculate_serial_parallel_fr_value_of_components()

                self.main_dict[str(selected_system)][str(selected_module)][str(selected_sub_module)]["Configurations"][str(s_name)] = None
                self.main_dict[str(selected_system)][str(selected_module)][str(selected_sub_module)]["Configurations"][str(s_name)] = {'Depth Level': level, 'Failure Rate': serial_fr, 'Type': str(type_value), 'Components': [str(component1), str(component2)]}

                self.listWidget_configurations_of_components.addItem(str(s_name + ' = ' + s_content))

            elif type_value == 'Parallel':
                p_name = 'p' + str(selected_configuration_count)
                new_config_name = p_name
                p_content = 'parallel(' + str(component1) + ', ' + str(component2) + ')'
                parallel_fr = self.calculate_serial_parallel_fr_value_of_components()

                self.main_dict[str(selected_system)][str(selected_module)][str(selected_sub_module)]["Configurations"][str(p_name)] = None
                self.main_dict[str(selected_system)][str(selected_module)][str(selected_sub_module)]["Configurations"][str(p_name)] = {'Depth Level': level, 'Failure Rate': parallel_fr, 'Type': str(type_value), 'Components': [str(component1), str(component2)]}

                self.listWidget_configurations_of_components.addItem(str(p_name + ' = ' + p_content))

            max_level = self.find_max_depth_level_of_component_configs()
            self.main_dict[str(selected_system)][str(selected_module)][str(selected_sub_module)]["Configurations"]["Depth Level"] = max_level

            self.comboBox_configuration_select_first_comp.addItem(str(new_config_name))
            self.comboBox_configuration_select_second_comp.addItem(str(new_config_name))
            self.listWidget_components.addItem(str(new_config_name))


    def calculate_serial_parallel_fr_value_of_components(self):
        """
            Calculate Serial Parallel Failure Rate Value of Components Function
        """
        type_value = self.comboBox_configuration_type_of_component.currentText()
        selected_system = self.comboBox_configuration_st_sc_select_system.currentText()
        selected_module = self.comboBox_configuration_st_sc_select_module.currentText()
        selected_sub_module = self.comboBox_configuration_st_sc_select_sub_module.currentText()

        component1 = self.lineEdit_configuration_first_component.text()
        component2 = self.lineEdit_configuration_second_component.text()

        configuration_list = self.control_of_configuration_list(selected_system, selected_module, selected_sub_module)

        if ((component1 and component2) != "None") and ((component1 and component2) != ""):
            if component1 in configuration_list:
                component1_fr = self.main_dict[str(selected_system)][str(selected_module)][str(selected_sub_module)]['Configurations'][str(component1)]['Failure Rate']

            else:
                component1_fr = self.main_dict[str(selected_system)][str(selected_module)][str(selected_sub_module)][str(component1)]['Failure Rate']

            if component2 in configuration_list:
                component2_fr = self.main_dict[str(selected_system)][str(selected_module)][str(selected_sub_module)]['Configurations'][str(component2)]['Failure Rate']

            else:
                component2_fr = self.main_dict[str(selected_system)][str(selected_module)][str(selected_sub_module)][str(component2)]['Failure Rate']

        fr_list = list()
        fr_list = [component1_fr, component2_fr]

        if type_value == "Serial":
            serial_fr = self.fr_calculation_class.component_serial_failure_rate_calculation(fr_list)
            temp_fr = serial_fr

        elif type_value == "Parallel":
            parallel_fr = self.fr_calculation_class.component_parallel_failure_rate_calculation(fr_list)
            temp_fr = parallel_fr

        return temp_fr


    def calculate_depth_level_of_component_configs(self):
        """
            Calculate Depth Level of Component Configs Function
        """
        selected_system = self.comboBox_configuration_st_sc_select_system.currentText()
        selected_module = self.comboBox_configuration_st_sc_select_module.currentText()
        selected_sub_module = self.comboBox_configuration_st_sc_select_sub_module.currentText()

        component1 = self.lineEdit_configuration_first_component.text()
        component2 = self.lineEdit_configuration_second_component.text()

        sub_module_dict = self.main_dict[str(selected_system)][str(selected_module)][str(selected_sub_module)]
        configuration_list = self.control_of_configuration_list(selected_system, selected_module, selected_sub_module)

        if (component1 in configuration_list) and (component2 not in configuration_list):
            old_level = sub_module_dict["Configurations"][str(component1)]["Depth Level"]
            level = old_level + 1

        elif (component2 in configuration_list) and (component1 not in configuration_list):
            old_level = sub_module_dict["Configurations"][str(component2)]["Depth Level"]
            level = old_level + 1

        elif (component1 and component2) in configuration_list:
            old_level1 = sub_module_dict["Configurations"][str(component1)]["Depth Level"]
            old_level2 = sub_module_dict["Configurations"][str(component2)]["Depth Level"]
            old_level = max(old_level1, old_level2)
            level = old_level + 1

        else:
            level = 0

        return level


    def delete_component_configuration(self):
        """
            Delete Component Configuration Function
        """
        try:
            selected_config_item = self.listWidget_configurations_of_components.selectedItems()
            configuration = self.listWidget_configurations_of_components.currentItem().text()
            ind = configuration.find(' =')
            configuration_name = configuration[:ind]

            # Remove selected configuration item from listWidget_components list.
            for cnt in range(self.listWidget_components.count()):
                if self.listWidget_components.item(cnt) != None:
                    if str(self.listWidget_components.item(cnt).text()) == str(configuration_name):
                        self.listWidget_components.takeItem(cnt)

                    else:
                        continue

            # Remove selected configuration item from listWidget_configuration list.
            for item in selected_config_item:
                self.listWidget_configurations_of_components.takeItem(self.listWidget_configurations_of_components.row(item))

            # Remove selected configuration from general_dict, also.
            selected_system = self.comboBox_configuration_st_sc_select_system.currentText()
            selected_module = self.comboBox_configuration_st_sc_select_module.currentText()
            selected_sub_module = self.comboBox_configuration_st_sc_select_sub_module.currentText()

            del self.main_dict[str(selected_system)][str(selected_module)][str(selected_sub_module)]['Configurations'][str(configuration_name)]
            self.delete_component_from_configuration_func(selected_module, selected_sub_module, configuration_name)

        except AttributeError:
            print("\nNot found an object to delete.\n")


    def add_component_into_config(self):
        """
            Add Component Into Config Function
        """
        selected_component = self.listWidget_components.currentItem().text()
        selected_configuration = self.listWidget_configurations_of_components.currentItem().text()
        ind = selected_configuration.find(' =')
        selected_configuration_name = selected_configuration[:ind]

        if selected_component != "None" and selected_configuration != "None":
            new_config = selected_configuration[:-1] + ", " + selected_component + ")"

            for item in self.listWidget_configurations_of_components.selectedItems():
                item.setText(new_config)

        selected_system = self.comboBox_configuration_st_sc_select_system.currentText()
        selected_module = self.comboBox_configuration_st_sc_select_module.currentText()
        selected_sub_module = self.comboBox_configuration_st_sc_select_sub_module.currentText()

        component_list = list(self.main_dict[str(selected_system)][str(selected_module)][str(selected_sub_module)]['Configurations'][str(selected_configuration_name)]['Components'])
        component_list.append(str(selected_component))

        self.main_dict[str(selected_system)][str(selected_module)][str(selected_sub_module)]['Configurations'][str(selected_configuration_name)]['Components'] = component_list

        fr_list = list()
        configuration_list_of_components = self.control_of_configuration_list(selected_system, selected_module, selected_sub_module)

        for component in component_list:
            if component in configuration_list_of_components:
                fr_list.append(self.main_dict[str(selected_system)][str(selected_module)][str(selected_sub_module)]["Configurations"][str(component)]['Failure Rate'])
            else:
                fr_list.append(self.main_dict[str(selected_system)][str(selected_module)][str(selected_sub_module)][str(component)]['Failure Rate'])

        type_value = self.check_type(selected_configuration)

        if type_value == 'series':
            calculated_fr = self.fr_calculation_class.component_serial_failure_rate_calculation(fr_list)

        elif type_value == 'parallel':
            calculated_fr = self.fr_calculation_class.component_parallel_failure_rate_calculation(fr_list)

        self.main_dict[str(selected_system)][str(selected_module)][str(selected_sub_module)]['Configurations'][str(selected_configuration_name)]['Failure Rate'] = calculated_fr

        level = self.add_component_calculate_depth_level_of_configurations(selected_configuration_name)
        self.main_dict[str(selected_system)][str(selected_module)][str(selected_sub_module)]['Configurations'][str(selected_configuration_name)]['Depth Level'] = level

        max_level = self.find_max_depth_level_of_component_configs()
        self.main_dict[str(selected_system)][str(selected_module)][str(selected_sub_module)]['Configurations']['Depth Level'] = max_level


    def add_component_calculate_depth_level_of_configurations(self, selected_configuration_name):
        """
            Add Component Calculate Depth Level of Configurations
        """
        selected_system = self.comboBox_configuration_st_sc_select_system.currentText()
        selected_module = self.comboBox_configuration_st_sc_select_module.currentText()
        selected_sub_module = self.comboBox_configuration_st_sc_select_sub_module.currentText()

        sub_module_dict = self.main_dict[str(selected_system)][str(selected_module)][str(selected_sub_module)]
        component_list = list(sub_module_dict["Configurations"][str(selected_configuration_name)]["Components"])

        level_list = list()
        configuration_list_of_components = self.control_of_configuration_list(selected_system, selected_module, selected_sub_module)

        for item in component_list:
            if item not in configuration_list_of_components:
                level = 0

            else:
                level = sub_module_dict["Configurations"][str(item)]["Depth Level"]
            level_list.append(level)

        max_level = max(level_list) + 1

        return max_level


    def find_max_depth_level_of_component_configs(self):
        """
            Find Max Depth Level of Component Configs Function
        """
        selected_system = self.comboBox_configuration_st_sc_select_system.currentText()
        selected_module = self.comboBox_configuration_st_sc_select_module.currentText()
        selected_sub_module = self.comboBox_configuration_st_sc_select_sub_module.currentText()

        if ((selected_system and selected_module and selected_sub_module) != "") and ((selected_system and selected_module and selected_sub_module) != "None"):
            sub_module_dict = self.main_dict[str(selected_system)][str(selected_module)][str(selected_sub_module)]
            configuration_list = list(sub_module_dict["Configurations"].keys())
            self.remove_key(configuration_list)
            level_list = list()

            for item in configuration_list:
                level = sub_module_dict["Configurations"][str(item)]["Depth Level"]
                level_list.append(level)

            return max(level_list)


    def select_final_configuration_of_sub_module(self):
        """
            Select Final Configuration of Sub Module Function
        """
        try:
            selected_system = self.comboBox_configuration_st_sc_select_system.currentText()
            selected_module = self.comboBox_configuration_st_sc_select_module.currentText()
            selected_sub_module = self.comboBox_configuration_st_sc_select_sub_module.currentText()
            configuration_count = self.listWidget_configurations_of_components.count()

            if configuration_count > 0:
                selected_final_configuration = self.listWidget_configurations_of_components.currentItem().text()
                ind = selected_final_configuration.find(' =')
                selected_final_configuration_name = selected_final_configuration[:ind]
                selected_final_configuration_fr = self.main_dict[str(selected_system)][str(selected_module)][str(selected_sub_module)]['Configurations'][str(selected_final_configuration_name)]['Failure Rate']

            else:
                selected_final_configuration_name = self.listWidget_components.currentItem().text()
                selected_final_configuration_fr = self.main_dict[str(selected_system)][str(selected_module)][str(selected_sub_module)][str(selected_final_configuration_name)]['Failure Rate']

            self.main_dict[str(selected_system)][str(selected_module)][str(selected_sub_module)]["Final Configuration"] = str(selected_final_configuration_name)
            self.main_dict[str(selected_system)][str(selected_module)][str(selected_sub_module)]['Failure Rate'] = selected_final_configuration_fr

            self.lineEdit_configuration_failure_rate_sub_module.setText(str(selected_final_configuration_fr))
            self.set_sub_module_listwidget_func()

        except Exception as err:
            print("\nError: select_final_configuration_of_sub_module\n")
            print(err)

# "CONFIGURATION // SET TYPE // SET COMPONENT TYPES FUNCTIONS" - END -

# "CONFIGURATION // SET TYPE // SET SUB-MODULE TYPES FUNCTIONS" - START -

    def first_sub_module_list_tab_func(self):
        """
            First Sub Module List Tab Function
        """
        self.comboBox_configuration_select_first_sub_module.clear()
        self.comboBox_configuration_select_first_sub_module.addItem("None")


    def second_sub_module_list_tab_func(self):
        """
            Second Sub Module List Tab Function
        """
        self.comboBox_configuration_select_second_sub_module.clear()
        self.comboBox_configuration_select_second_sub_module.addItem("None")


    def sub_module_list_widget_func(self):
        """
            Sub Module List Widget Function
        """
        self.listWidget_sub_modules.clear()
        self.listWidget_configurations_of_sub_modules.clear()


    def module_list_control_of_set_sub_module_types_tab(self):
        """
            Module List Control of Set Sub Module Types Tab Function
        """
        selected_system = self.comboBox_configuration_st_ssm_select_system.currentText()

        self.comboBox_configuration_st_ssm_select_module.clear()

        if selected_system != "None" and selected_system != "":
            module_list = list(self.main_dict[str(selected_system)].keys())
            self.remove_key(module_list)

            self.comboBox_configuration_st_ssm_select_module.addItems(module_list)


    def sub_module_list_control_of_set_sub_module_types_tab(self):
        """
            Sub Module List Control of Set Sub Module Types Tab Function
        """
        selected_system = self.comboBox_configuration_st_ssm_select_system.currentText()
        selected_module = self.comboBox_configuration_st_ssm_select_module.currentText()

        self.comboBox_configuration_select_first_sub_module.clear()
        self.comboBox_configuration_select_second_sub_module.clear()
        self.listWidget_sub_modules.clear()
        self.lineEdit_configuration_failure_rate_first_sub_module.clear()
        self.lineEdit_configuration_failure_rate_second_sub_module.clear()
        self.lineEdit_configuration_first_sub_module.clear()
        self.lineEdit_configuration_second_sub_module.clear()

        if ((selected_system and selected_module) != "None") and ((selected_system and selected_module) != ""):
            sub_module_list = list(self.main_dict[str(selected_system)][str(selected_module)].keys())
            self.remove_key(sub_module_list)

            if "Configurations" in self.main_dict[str(selected_system)][str(selected_module)]:
                configuration_list = list(self.main_dict[str(selected_system)][str(selected_module)]["Configurations"].keys())

            else:
                configuration_list = list()

            self.remove_key(configuration_list)
            sub_module_list.extend(configuration_list)

            self.comboBox_configuration_select_first_sub_module.addItems(sub_module_list)
            self.comboBox_configuration_select_second_sub_module.addItems(sub_module_list)
            self.listWidget_sub_modules.addItems(sorted(sub_module_list))

        else:
            self.comboBox_configuration_select_first_sub_module.clear()
            self.comboBox_configuration_select_second_sub_module.clear()
            self.listWidget_sub_modules.clear()


    def module_fr_control(self):
        """
            Module Failure Rate Control Function
        """
        self.lineEdit_configuration_failure_rate_module.clear()

        selected_system = self.comboBox_configuration_st_ssm_select_system.currentText()
        selected_module = self.comboBox_configuration_st_ssm_select_module.currentText()

        if ((selected_system and selected_module) != "") and ((selected_system and selected_module) != "None"):
            selected_module_fr = self.main_dict[str(selected_system)][str(selected_module)]['Failure Rate']["Nominal"]
            self.lineEdit_configuration_failure_rate_module.setText(str(selected_module_fr))


    def convert_configurations_of_sub_modules_into_syntax(self):
        """
            Convert Configurations of Sub Modules Into Syntax Function
        """
        selected_system = self.comboBox_configuration_st_ssm_select_system.currentText()
        selected_module = self.comboBox_configuration_st_ssm_select_module.currentText()

        self.listWidget_configurations_of_sub_modules.clear()

        if ((selected_system and selected_module) != "") and ((selected_system and selected_module) != "None"):
            if "Configurations" in self.main_dict[str(selected_system)][str(selected_module)]:
                configuraiton_list = list(self.main_dict[str(selected_system)][str(selected_module)]["Configurations"].keys())
                self.remove_key(configuraiton_list)
                syntax_list = list()

                for configuration in configuraiton_list:
                    type_value = self.main_dict[str(selected_system)][str(selected_module)]["Configurations"][str(configuration)]["Type"]
                    component_list = list(self.main_dict[str(selected_system)][str(selected_module)]["Configurations"][str(configuration)]["Components"])
                    syntax_name = str(configuration)

                    if type_value == "Serial":
                        temp_syntax = str()

                        for component in component_list:
                            temp_syntax += str(component) + ', '

                        length = len(temp_syntax)
                        temp_syntax = temp_syntax[:length-2]
                        syntax = str(syntax_name) + ' = ' + 'series(' + str(temp_syntax) + ')'

                    elif type_value == 'Parallel':
                        temp_syntax = str()

                        for component in component_list:
                            temp_syntax += str(component) + ', '

                        length = len(temp_syntax)
                        temp_syntax = temp_syntax[:length-2]
                        syntax = str(syntax_name) + ' = ' + 'parallel(' + str(temp_syntax) + ')'

                    syntax_list.append(syntax)

                self.listWidget_configurations_of_sub_modules.addItems(sorted(syntax_list))


    def first_sub_module_list_control_of_set_sub_module_types_tab(self):
        """
            First Sub Module List Control of Set Sub Module Types Tab Function
        """
        selected_system = self.comboBox_configuration_st_ssm_select_system.currentText()
        selected_module = self.comboBox_configuration_st_ssm_select_module.currentText()

        if ((selected_system and selected_module) != "None") and ((selected_system and selected_module) != ""):
            sub_module1 = self.comboBox_configuration_select_first_sub_module.currentText()
            configuration_list = self.control_of_configuration_list(selected_system, selected_module)

            if sub_module1 != "None" and sub_module1 != "":
                self.lineEdit_configuration_first_sub_module.setText(str(sub_module1))

                if sub_module1 in configuration_list:
                    sub_module1_fr = self.main_dict[str(selected_system)][str(selected_module)]["Configurations"][str(sub_module1)]["Failure Rate"]

                else:
                    sub_module1_fr = self.main_dict[str(selected_system)][str(selected_module)][str(sub_module1)]['Failure Rate']

                self.lineEdit_configuration_failure_rate_first_sub_module.setText(str(sub_module1_fr))


    def second_sub_module_list_control_of_set_sub_module_types_tab(self):
        """
            Second Sub Module List Control of Set Sub Module Types Tab Function
        """
        selected_system = self.comboBox_configuration_st_ssm_select_system.currentText()
        selected_module = self.comboBox_configuration_st_ssm_select_module.currentText()

        if ((selected_system and selected_module) != "None") and ((selected_system and selected_module) != ""):
            sub_module2 = self.comboBox_configuration_select_second_sub_module.currentText()
            configuration_list = self.control_of_configuration_list(selected_system, selected_module)

            if sub_module2 != "None" and sub_module2 != "":
                self.lineEdit_configuration_second_sub_module.setText(str(sub_module2))

                if sub_module2 in configuration_list:
                    sub_module2_fr = self.main_dict[str(selected_system)][str(selected_module)]["Configurations"][str(sub_module2)]["Failure Rate"]

                else:
                    sub_module2_fr = self.main_dict[str(selected_system)][str(selected_module)][str(sub_module2)]['Failure Rate']

                self.lineEdit_configuration_failure_rate_second_sub_module.setText(str(sub_module2_fr))


    def create_syntax_of_sub_modules(self):
        """
            Create Syntax of Sub Modules Function
        """
        type_value = self.comboBox_configuration_type_of_sub_module.currentText()
        selected_system = self.comboBox_configuration_st_ssm_select_system.currentText()
        selected_module = self.comboBox_configuration_st_ssm_select_module.currentText()
        sub_module1 = self.lineEdit_configuration_first_sub_module.text()
        sub_module2 = self.lineEdit_configuration_second_sub_module.text()

        level = self.calculate_depth_level_of_sub_module_configs()

        if type_value != "":
            new_config_name = str()
            self.main_dict[str(selected_system)][str(selected_module)]["Configurations"]["Configuration Count"][str(type_value)] += 1
            selected_configuration_count = self.main_dict[str(selected_system)][str(selected_module)]["Configurations"]["Configuration Count"][str(type_value)]

            if type_value == 'Serial':
                s_name = 's' + str(selected_configuration_count)
                new_config_name = s_name
                s_content = 'series(' + str(sub_module1) + ', ' + str(sub_module2) + ')'
                serial_fr = self.calculate_serial_parallel_fr_value_of_sub_modules()

                self.main_dict[str(selected_system)][str(selected_module)]["Configurations"][str(s_name)] = None
                self.main_dict[str(selected_system)][str(selected_module)]["Configurations"][str(s_name)] = {'Depth Level': level, 'Failure Rate': serial_fr, 'Type': str(type_value), 'Components': [str(sub_module1), str(sub_module2)]}
                self.listWidget_configurations_of_sub_modules.addItem(str(s_name + ' = ' + s_content))

            elif type_value == 'Parallel':
                p_name = 'p' + str(selected_configuration_count)
                new_config_name = p_name
                p_content = 'parallel(' + str(sub_module1) + ', ' + str(sub_module2) + ')'
                parallel_fr = self.calculate_serial_parallel_fr_value_of_sub_modules()

                self.main_dict[str(selected_system)][str(selected_module)]["Configurations"][str(p_name)] = None
                self.main_dict[str(selected_system)][str(selected_module)]["Configurations"][str(p_name)] = {'Depth Level': level, 'Failure Rate': parallel_fr, 'Type': str(type_value), 'Components': [str(sub_module1), str(sub_module2)]}
                self.listWidget_configurations_of_sub_modules.addItem(str(p_name + ' = ' + p_content))

            max_level = self.find_max_depth_level_of_sub_module_configs()
            self.main_dict[str(selected_system)][str(selected_module)]["Configurations"]["Depth Level"] = max_level

            self.comboBox_configuration_select_first_sub_module.addItem(str(new_config_name))
            self.comboBox_configuration_select_second_sub_module.addItem(str(new_config_name))
            self.listWidget_sub_modules.addItem(str(new_config_name))


    def calculate_serial_parallel_fr_value_of_sub_modules(self):
        """
            Calculate Serial Parallel Failure Rate Value of Sub Modules Function
        """
        type_value = self.comboBox_configuration_type_of_sub_module.currentText()
        selected_system = self.comboBox_configuration_st_ssm_select_system.currentText()
        selected_module = self.comboBox_configuration_st_ssm_select_module.currentText()
        sub_module1 = self.lineEdit_configuration_first_sub_module.text()
        sub_module2 = self.lineEdit_configuration_second_sub_module.text()

        configuration_list = self.control_of_configuration_list(selected_system, selected_module)

        if ((sub_module1 and sub_module2) != "None") and ((sub_module1 and sub_module2) != ""):
            if sub_module1 in configuration_list:
                sub_module1_fr = self.main_dict[str(selected_system)][str(selected_module)]['Configurations'][str(sub_module1)]['Failure Rate']

            else:
                sub_module1_fr = self.main_dict[str(selected_system)][str(selected_module)][str(sub_module1)]['Failure Rate']

            if sub_module2 in configuration_list:
                sub_module2_fr = self.main_dict[str(selected_system)][str(selected_module)]['Configurations'][str(sub_module2)]['Failure Rate']

            else:
                sub_module2_fr = self.main_dict[str(selected_system)][str(selected_module)][str(sub_module2)]['Failure Rate']

        fr_list = list()
        fr_list = [sub_module1_fr, sub_module2_fr]

        if type_value == "Serial":
            serial_fr = self.fr_calculation_class.component_serial_failure_rate_calculation(fr_list)
            temp_fr = serial_fr

        elif type_value == "Parallel":
            parallel_fr = self.fr_calculation_class.component_parallel_failure_rate_calculation(fr_list)
            temp_fr = parallel_fr

        return temp_fr


    def calculate_depth_level_of_sub_module_configs(self):
        """
            Calculate Depth Level of Sub Module Configs Function
        """
        selected_system = self.comboBox_configuration_st_ssm_select_system.currentText()
        selected_module = self.comboBox_configuration_st_ssm_select_module.currentText()

        sub_module1 = self.lineEdit_configuration_first_sub_module.text()
        sub_module2 = self.lineEdit_configuration_second_sub_module.text()

        module_dict = self.main_dict[str(selected_system)][str(selected_module)]
        configuration_list_of_sub_modules = self.control_of_configuration_list(selected_system, selected_module)

        if (sub_module1 in configuration_list_of_sub_modules) and (sub_module2 not in configuration_list_of_sub_modules):
            old_level = module_dict["Configurations"][str(sub_module1)]["Depth Level"]
            level = old_level + 1

        elif (sub_module2 in configuration_list_of_sub_modules) and (sub_module1 not in configuration_list_of_sub_modules):
            old_level = module_dict["Configurations"][str(sub_module2)]["Depth Level"]
            level = old_level + 1

        elif (sub_module1 and sub_module2) in configuration_list_of_sub_modules:
            old_level1 = module_dict["Configurations"][str(sub_module1)]["Depth Level"]
            old_level2 = module_dict["Configurations"][str(sub_module2)]["Depth Level"]
            old_level = max(old_level1, old_level2)
            level = old_level + 1

        else:
            level = 0

        return level


    def find_max_depth_level_of_sub_module_configs(self):
        """
            Find Max Depth Level of Sub Module Configs Function
        """
        selected_system = self.comboBox_configuration_st_ssm_select_system.currentText()
        selected_module = self.comboBox_configuration_st_ssm_select_module.currentText()

        if ((selected_system and selected_module) != "") and ((selected_system and selected_module) != "None"):
            module_dict = self.main_dict[str(selected_system)][str(selected_module)]
            configuration_list = list(module_dict["Configurations"].keys())
            self.remove_key(configuration_list)
            level_list = list()

            for item in configuration_list:
                level = module_dict["Configurations"][str(item)]["Depth Level"]
                level_list.append(level)

            return max(level_list)


    def delete_sub_module_configuration(self):
        """
            Delete Sub Module Configuration Function
        """
        try:
            selected_config_item = self.listWidget_configurations_of_sub_modules.selectedItems()
            configuration = self.listWidget_configurations_of_sub_modules.currentItem().text()
            ind = configuration.find(' =')
            configuration_name = configuration[:ind]

            # Remove selected configuration item from listWidget_components list.
            for cnt in range(self.listWidget_sub_modules.count()):
                if self.listWidget_sub_modules.item(cnt) != None:
                    if str(self.listWidget_sub_modules.item(cnt).text()) == str(configuration_name):
                        self.listWidget_sub_modules.takeItem(cnt)

                    else:
                        continue

            # Remove selected configuration item from listWidget_configuration list.
            for item in selected_config_item:
                self.listWidget_configurations_of_sub_modules.takeItem(self.listWidget_configurations_of_sub_modules.row(item))

            # Remove selected configuration item from comboBox_configuration_select_first_comp
            # and comboBox_configuration_select_second_comp.
            index_1 = self.comboBox_configuration_select_first_sub_module.findText(configuration_name)
            self.comboBox_configuration_select_first_sub_module.removeItem(index_1)

            index_2 = self.comboBox_configuration_select_second_sub_module.findText(configuration_name)
            self.comboBox_configuration_select_second_sub_module.removeItem(index_2)

            # Remove selected configuration from general_dict, also.
            selected_system = self.comboBox_configuration_st_ssm_select_system.currentText()
            selected_module = self.comboBox_configuration_st_ssm_select_module.currentText()
            del self.main_dict[str(selected_system)][str(selected_module)]['Configurations'][str(configuration_name)]

            self.delete_sub_module_from_configuration_func(selected_module, configuration_name)

        except AttributeError:
            print("\nNot found an object to delete.\n")


    def add_sub_module_into_config(self):
        """
            Add Sub Module Into Config Function
        """
        selected_sub_module = self.listWidget_sub_modules.currentItem().text()
        selected_configuration = self.listWidget_configurations_of_sub_modules.currentItem().text()
        ind = selected_configuration.find(' =')
        selected_configuration_name = selected_configuration[:ind]

        if selected_sub_module != "None" and selected_configuration != "None":
            new_config = selected_configuration[:-1] + ", " + selected_sub_module + ")"

            for item in self.listWidget_configurations_of_sub_modules.selectedItems():
                item.setText(new_config)

        selected_system = self.comboBox_configuration_st_ssm_select_system.currentText()
        selected_module = self.comboBox_configuration_st_ssm_select_module.currentText()

        component_list = list(self.main_dict[str(selected_system)][str(selected_module)]['Configurations'][str(selected_configuration_name)]['Components'])
        component_list.append(str(selected_sub_module))
        self.main_dict[str(selected_system)][str(selected_module)]['Configurations'][str(selected_configuration_name)]['Components'] = component_list

        fr_list = list()
        configuration_list_of_sub_modules = self.control_of_configuration_list(selected_system, selected_module)

        for component in component_list:
            if component in configuration_list_of_sub_modules:
                fr_list.append(self.main_dict[str(selected_system)][str(selected_module)]["Configurations"][str(component)]['Failure Rate'])

            else:
                fr_list.append(self.main_dict[str(selected_system)][str(selected_module)][str(component)]['Failure Rate'])

        type_value = self.check_type(selected_configuration)

        if type_value == 'series':
            calculated_fr = self.fr_calculation_class.component_serial_failure_rate_calculation(fr_list)

        elif type_value == 'parallel':
            calculated_fr = self.fr_calculation_class.component_parallel_failure_rate_calculation(fr_list)

        self.main_dict[str(selected_system)][str(selected_module)]['Configurations'][str(selected_configuration_name)]['Failure Rate'] = calculated_fr

        level = self.add_sub_module_calculate_depth_level_of_configurations(selected_configuration_name)
        self.main_dict[str(selected_system)][str(selected_module)]['Configurations'][str(selected_configuration_name)]['Depth Level'] = level

        max_level = self.find_max_depth_level_of_sub_module_configs()
        self.main_dict[str(selected_system)][str(selected_module)]['Configurations']['Depth Level'] = max_level


    def add_sub_module_calculate_depth_level_of_configurations(self, selected_configuration_name):
        """
            Add Sub Module Calculate Depth Level of Configurations Function
        """
        selected_system = self.comboBox_configuration_st_ssm_select_system.currentText()
        selected_module = self.comboBox_configuration_st_ssm_select_module.currentText()

        module_dict = self.main_dict[str(selected_system)][str(selected_module)]
        component_list = list(module_dict["Configurations"][str(selected_configuration_name)]["Components"])

        configuration_list_of_sub_modules = self.control_of_configuration_list(selected_system, selected_module)
        level_list = list()

        for item in component_list:
            if item not in configuration_list_of_sub_modules:
                level = 0

            else:
                level = module_dict["Configurations"][str(item)]["Depth Level"]

            level_list.append(level)

        max_level = max(level_list) + 1

        return max_level


    def select_final_configuration_of_module(self):
        """
            Select Final Configuration of Module Function
        """
        try:
            selected_system = self.comboBox_configuration_st_ssm_select_system.currentText()
            selected_module = self.comboBox_configuration_st_ssm_select_module.currentText()
            configuration_count = self.listWidget_configurations_of_sub_modules.count()

            if configuration_count > 0:
                selected_final_configuration = self.listWidget_configurations_of_sub_modules.currentItem().text()
                ind = selected_final_configuration.find(' =')
                selected_final_configuration_name = selected_final_configuration[:ind]
                selected_final_configuration_fr = self.main_dict[str(selected_system)][str(selected_module)]['Configurations'][str(selected_final_configuration_name)]['Failure Rate']

            else:
                selected_final_configuration_name = self.listWidget_sub_modules.currentItem().text()
                selected_final_configuration_fr = self.main_dict[str(selected_system)][str(selected_module)][str(selected_final_configuration_name)]['Failure Rate']

            self.main_dict[str(selected_system)][str(selected_module)]["Final Configuration"] = str(selected_final_configuration_name)
            self.main_dict[str(selected_system)][str(selected_module)]['Failure Rate']["Nominal"] = selected_final_configuration_fr

            self.lineEdit_configuration_failure_rate_module.setText(str(selected_final_configuration_fr))

            self.set_module_listwidget_func()

        except Exception as err:
            print("\nError: select_final_configuration_of_module\n")
            print(err)

# "CONFIGURATION // SET TYPE // SET SUB-MODULE TYPES FUNCTIONS" - END -

# "CONFIGURATION // SET TYPE // SET MODULE TYPES FUNCTIONS" - START -

    def first_module_list_tab_func(self):
        """
            First Module List Tab Function
        """
        self.comboBox_configuration_select_first_module.clear()
        self.comboBox_configuration_select_first_module.addItem("None")


    def second_module_list_tab_func(self):
        """
            Second Module List Tab Function
        """
        self.comboBox_configuration_select_second_module.clear()
        self.comboBox_configuration_select_second_module.addItem("None")


    def module_list_widget_func(self):
        """
            Module List Widget Function
        """
        self.listWidget_modules.clear()
        self.listWidget_configurations_of_modules.clear()


    def system_list_control_of_set_module_types_tab(self):
        """
            System List Control of Set Module Types Tab Function
        """
        current_system = self.comboBox_configuration_st_sm_select_system.currentText()
        self.comboBox_configuration_select_first_module.clear()
        self.comboBox_configuration_select_second_module.clear()
        self.listWidget_modules.clear()

        if current_system != "None" and current_system != "":
            module_list = list(self.main_dict[str(current_system)].keys())
            self.remove_key(module_list)

            configuration_list = list(self.main_dict[str(current_system)]['Configurations'].keys())
            self.remove_key(configuration_list)
            module_list.extend(configuration_list)

            self.comboBox_configuration_select_first_module.addItems(sorted(module_list))
            self.comboBox_configuration_select_second_module.addItems(sorted(module_list))
            self.listWidget_modules.addItems(sorted(module_list))

        else:
            self.comboBox_configuration_select_first_module.clear()
            self.comboBox_configuration_select_second_module.clear()
            self.listWidget_modules.clear()


    def system_fr_control(self):
        """
            System Failure Rate Control Function
        """
        self.lineEdit_configuration_failure_rate_system.clear()
        current_system = self.comboBox_configuration_st_sm_select_system.currentText()

        if current_system != "None" and current_system != "":
            current_system_fr = self.main_dict[str(current_system)]['Failure Rate']["Nominal"]
            self.lineEdit_configuration_failure_rate_system.setText(str(current_system_fr))


    def convert_configurations_of_modules_into_syntax(self):
        """
            Convert Configurations of Modules Into Syntax Function
        """
        current_system = self.comboBox_configuration_st_sm_select_system.currentText()
        self.listWidget_configurations_of_modules.clear()

        if current_system != "None" and current_system != "":
            configuration_list = list(self.main_dict[str(current_system)]['Configurations'].keys())
            self.remove_key(configuration_list)
            syntax_list = list()

            for configuration in configuration_list:
                type_value = self.main_dict[str(current_system)]['Configurations'][str(configuration)]['Type']
                component_list = list(self.main_dict[str(current_system)]['Configurations'][str(configuration)]['Components'])
                syntax_name = str(configuration)

                if type_value == 'Serial':
                    temp_syntax = str()
                    for component in component_list:
                        temp_syntax += str(component) + ', '

                    length = len(temp_syntax)
                    temp_syntax = temp_syntax[:length-2]
                    syntax = str(syntax_name) + ' = ' + 'series(' + str(temp_syntax) + ')'

                elif type_value == 'Parallel':
                    temp_syntax = str()

                    for component in component_list:
                        temp_syntax += str(component) + ', '

                    length = len(temp_syntax)
                    temp_syntax = temp_syntax[:length-2]
                    syntax = str(syntax_name) + ' = ' + 'parallel(' + str(temp_syntax) + ')'

                syntax_list.append(syntax)

            self.listWidget_configurations_of_modules.addItems(sorted(syntax_list))


    def module_list_control_of_set_module_types_tab(self):
        """
            Module List Control of Set Module Types Tab Function
        """
        current_system = self.comboBox_configuration_st_sm_select_system.currentText()

        if current_system != "None" and current_system != "":
            module1 = str(self.comboBox_configuration_select_first_module.currentText())
            module2 = str(self.comboBox_configuration_select_second_module.currentText())
            configuration_list = self.control_of_configuration_list(current_system)

            if module1 != "None" and module1 != "":
                self.lineEdit_configuration_first_module.setText(str(module1))

                if module1 in configuration_list:
                    module1_fr = self.main_dict[str(current_system)]['Configurations'][str(module1)]['Failure Rate']["Nominal"]

                else:
                    module1_fr = self.main_dict[str(current_system)][str(module1)]['Failure Rate']["Nominal"]

                self.lineEdit_configuration_failure_rate_first_module.setText(str(module1_fr))

            if module2 != "None" and module2 != "":
                self.lineEdit_configuration_second_module.setText(str(module2))

                if module2 in configuration_list:
                    module2_fr = self.main_dict[str(current_system)]['Configurations'][str(module2)]['Failure Rate']["Nominal"]

                else:
                    module2_fr = self.main_dict[str(current_system)][str(module2)]['Failure Rate']["Nominal"]

                self.lineEdit_configuration_failure_rate_second_module.setText(str(module2_fr))


    def create_syntax_of_modules(self):
        """
            Create Syntax of Modules Function
        """
        type_value = self.comboBox_configuration_type_of_module.currentText()
        selected_system = self.comboBox_configuration_st_sm_select_system.currentText()
        module1 = self.lineEdit_configuration_first_module.text()
        module2 = self.lineEdit_configuration_second_module.text()
        level = self.calculate_depth_level_of_module_configs()

        if type_value != "":
            new_config_name = str()
            self.main_dict[str(selected_system)]["Configurations"]["Configuration Count"][str(type_value)] += 1
            selected_configuration_count = self.main_dict[str(selected_system)]["Configurations"]["Configuration Count"][str(type_value)]

            if type_value == 'Serial':
                s_name = 's' + str(selected_configuration_count)
                new_config_name = s_name
                s_content = 'series(' + str(module1) + ', ' + str(module2) + ')'
                serial_fr = self.calculate_serial_parallel_fr_value_of_modules()

                self.main_dict[str(selected_system)]['Configurations'][str(s_name)] = None
                self.main_dict[str(selected_system)]['Configurations'][str(s_name)] = {'Depth Level': level, 'Failure Rate': {"Nominal": serial_fr, "Sensor Based": serial_fr}, 'Type': str(type_value), 'Components': [str(module1), str(module2)]}
                self.listWidget_configurations_of_modules.addItem(str(s_name + ' = ' + s_content))

            elif type_value == 'Parallel':
                p_name = 'p' + str(selected_configuration_count)
                new_config_name = p_name
                p_content = 'parallel(' + str(module1) + ', ' + str(module2) + ')'
                parallel_fr = self.calculate_serial_parallel_fr_value_of_modules()

                self.main_dict[str(selected_system)]['Configurations'][str(p_name)] = None
                self.main_dict[str(selected_system)]['Configurations'][str(p_name)] = {'Depth Level': level, 'Failure Rate': {"Nominal": parallel_fr, "Sensor Based": parallel_fr}, 'Type': str(type_value), 'Components': [str(module1), str(module2)]}

                self.listWidget_configurations_of_modules.addItem(str(p_name + ' = ' + p_content))

            max_level = self.find_max_depth_level_of_module_configs()
            self.main_dict[str(selected_system)]["Configurations"]["Depth Level"] = max_level

            self.comboBox_configuration_select_first_module.addItem(str(new_config_name))
            self.comboBox_configuration_select_second_module.addItem(str(new_config_name))
            self.listWidget_modules.addItem(str(new_config_name))


    def calculate_serial_parallel_fr_value_of_modules(self):
        """
            Calculate Serial Parallel Failure Rate Value of Modules
        """
        type_value = self.comboBox_configuration_type_of_module.currentText()
        current_system = self.comboBox_configuration_st_sm_select_system.currentText()
        module1 = self.lineEdit_configuration_first_module.text()
        module2 = self.lineEdit_configuration_second_module.text()

        configuration_list = self.control_of_configuration_list(current_system)

        if ((module1 and module2) != "None") and ((module1 and module2) != ""):
            if module1 in configuration_list:
                module1_fr = self.main_dict[str(current_system)]['Configurations'][str(module1)]['Failure Rate']["Nominal"]

            else:
                module1_fr = self.main_dict[str(current_system)][str(module1)]['Failure Rate']["Nominal"]

            if module2 in configuration_list:
                module2_fr = self.main_dict[str(current_system)]['Configurations'][str(module2)]['Failure Rate']["Nominal"]

            else:
                module2_fr = self.main_dict[str(current_system)][str(module2)]['Failure Rate']["Nominal"]

            fr_list = list()
            fr_list = [module1_fr, module2_fr]

            if type_value == "Serial":
                serial_fr = self.fr_calculation_class.component_serial_failure_rate_calculation(fr_list)
                temp_fr = serial_fr

            elif type_value == "Parallel":
                parallel_fr = self.fr_calculation_class.component_parallel_failure_rate_calculation(fr_list)
                temp_fr = parallel_fr

            return temp_fr


    def calculate_depth_level_of_module_configs(self):
        """
            Calculate Depth Level of Module Configs Function
        """
        selected_system = self.comboBox_configuration_st_sm_select_system.currentText()
        module1 = self.lineEdit_configuration_first_module.text()
        module2 = self.lineEdit_configuration_second_module.text()

        system_dict = self.main_dict[str(selected_system)]
        configuration_list_of_modules = self.control_of_configuration_list(selected_system)

        if (module1 in configuration_list_of_modules) and (module2 not in configuration_list_of_modules):
            old_level = system_dict["Configurations"][str(module1)]["Depth Level"]
            level = old_level + 1

        elif (module2 in configuration_list_of_modules) and (module1 not in configuration_list_of_modules):
            old_level = system_dict["Configurations"][str(module2)]["Depth Level"]
            level = old_level + 1

        elif (module1 and module2) in configuration_list_of_modules:
            old_level1 = system_dict["Configurations"][str(module1)]["Depth Level"]
            old_level2 = system_dict["Configurations"][str(module2)]["Depth Level"]
            old_level = max(old_level1, old_level2)
            level = old_level + 1

        else:
            level = 0

        return level


    def find_max_depth_level_of_module_configs(self):
        """
            Find Max Depth Level of Module Configs Function
        """
        selected_system = self.comboBox_configuration_st_sm_select_system.currentText()

        if (selected_system != "") and (selected_system != "None"):
            system_dict = self.main_dict[str(selected_system)]
            configuration_list = list(system_dict["Configurations"].keys())
            self.remove_key(configuration_list)
            level_list = list()

            for item in configuration_list:
                level = system_dict["Configurations"][str(item)]["Depth Level"]
                level_list.append(level)

            return max(level_list)


    def delete_module_configuration(self):
        """
            Delete Module Configuration Function
        """
        try:
            selected_config_item = self.listWidget_configurations_of_modules.selectedItems()
            configuration = self.listWidget_configurations_of_modules.currentItem().text()
            ind = configuration.find(' =')
            configuration_name = configuration[:ind]

            # Remove selected configuration item from listWidget_components list.
            for cnt in range(self.listWidget_modules.count()):
                if self.listWidget_modules.item(cnt) != None:
                    if str(self.listWidget_modules.item(cnt).text()) == str(configuration_name):
                        self.listWidget_modules.takeItem(cnt)

                    else:
                        continue

            # Remove selected configuration item from listWidget_configuration list.
            for item in selected_config_item:
                self.listWidget_configurations_of_modules.takeItem(self.listWidget_configurations_of_modules.row(item))

            # Remove selected configuration item from comboBox_configuration_select_first_comp
            # and comboBox_configuration_select_second_comp.
            index_1 = self.comboBox_configuration_select_first_module.findText(configuration_name)
            self.comboBox_configuration_select_first_module.removeItem(index_1)

            index_2 = self.comboBox_configuration_select_second_module.findText(configuration_name)
            self.comboBox_configuration_select_second_module.removeItem(index_2)

            # Remove selected configuration from general_dict, also.
            current_system = self.comboBox_configuration_st_sm_select_system.currentText()
            del self.main_dict[str(current_system)]['Configurations'][str(configuration_name)]

            self.delete_module_from_configuration_func(configuration_name)

        except AttributeError:
            print("\nNot found an object to delete.\n")


    def add_module_into_config(self):
        """
            Add Module Into Config Function
        """
        selected_module = self.listWidget_modules.currentItem().text()
        selected_configuration = self.listWidget_configurations_of_modules.currentItem().text()
        ind = selected_configuration.find(' =')
        selected_configuration_name = selected_configuration[:ind]

        if selected_module != "None" and selected_configuration != "None":
            new_config = selected_configuration[:-1] + ", " + selected_module + ")"

            for item in self.listWidget_configurations_of_modules.selectedItems():
                item.setText(new_config)

        current_system = self.comboBox_configuration_st_sm_select_system.currentText()
        component_list = list(self.main_dict[str(current_system)]['Configurations'][str(selected_configuration_name)]['Components'])
        component_list.append(str(selected_module))
        self.main_dict[str(current_system)]['Configurations'][str(selected_configuration_name)]['Components'] = component_list

        fr_list = list()
        configuration_list_of_modules = self.control_of_configuration_list(current_system)

        for component in component_list:
            if component in configuration_list_of_modules:
                fr_list.append(self.main_dict[str(current_system)]["Configurations"][str(component)]["Failure Rate"]["Nominal"])

            else:
                fr_list.append(self.main_dict[current_system][str(component)]["Failure Rate"]["Nominal"])

        type_value = self.check_type(selected_configuration)

        if type_value == 'series':
            calculated_fr = self.fr_calculation_class.component_serial_failure_rate_calculation(fr_list)

        elif type_value == 'parallel':
            calculated_fr = self.fr_calculation_class.component_parallel_failure_rate_calculation(fr_list)

        self.main_dict[str(current_system)]['Configurations'][str(selected_configuration_name)]['Failure Rate']["Nominal"] = calculated_fr
        self.main_dict[str(current_system)]['Configurations'][str(selected_configuration_name)]['Failure Rate']["Sensor Based"] = calculated_fr

        level = self.add_module_calculate_depth_level_of_configurations(selected_configuration_name)
        self.main_dict[str(current_system)]['Configurations'][str(selected_configuration_name)]['Depth Level'] = level

        max_level = self.find_max_depth_level_of_module_configs()
        self.main_dict[str(current_system)]['Configurations']['Depth Level'] = max_level


    def add_module_calculate_depth_level_of_configurations(self, selected_configuration_name):
        """
            Add Module Calculate Depth Level of Configurations Function
        """
        selected_system = self.comboBox_configuration_st_sm_select_system.currentText()
        system_dict = self.main_dict[str(selected_system)]
        component_list = list(system_dict["Configurations"][str(selected_configuration_name)]["Components"])

        configuration_list_of_modules = self.control_of_configuration_list(selected_system)
        level_list = list()

        for item in component_list:
            if item not in configuration_list_of_modules:
                level = 0

            else:
                level = system_dict["Configurations"][str(item)]["Depth Level"]
            level_list.append(level)

        max_level = max(level_list) + 1

        return max_level


    def select_final_configuration_of_system(self):
        """
            Select Final Configuration of System Function
        """
        try:
            current_system = self.comboBox_configuration_st_sm_select_system.currentText()
            configuration_count = self.listWidget_configurations_of_modules.count()

            if configuration_count > 0:
                selected_final_configuration = self.listWidget_configurations_of_modules.currentItem().text()
                ind = selected_final_configuration.find(' =')
                selected_final_configuration_name = selected_final_configuration[:ind]
                selected_final_configuration_fr = self.main_dict[str(current_system)]['Configurations'][str(selected_final_configuration_name)]['Failure Rate']["Nominal"]

            else:
                selected_final_configuration_name = self.listWidget_modules.currentItem().text()
                selected_final_configuration_fr = self.main_dict[str(current_system)][str(selected_final_configuration_name)]['Failure Rate']["Nominal"]

            self.main_dict[str(current_system)]["Final Configuration"] = str(selected_final_configuration_name)
            self.main_dict[str(current_system)]['Failure Rate']["Nominal"] = selected_final_configuration_fr

            self.lineEdit_configuration_failure_rate_system.setText(str(selected_final_configuration_fr))

        except Exception as err:
            print("\nError: select_final_configuration_of_system\n")
            print(err)


    def select_reliability_model_func(self):
        """
            Select Reliability Model Function
        """
        try:
            selected_model = self.comboBox_configuration_reliability_model.currentText()
            shape_parameter_list = ["Weibull Distribution", "Curve Distribution"]
            self.selected_reliability_model = selected_model

            if selected_model in shape_parameter_list:
                self.label_configuration_shape_parameter.setEnabled(True)
                self.lineEdit_configuration_reliability_shape_parameter.setEnabled(True)

            else:
                self.label_configuration_shape_parameter.setEnabled(False)
                self.lineEdit_configuration_reliability_shape_parameter.setEnabled(False)

        except Exception as err:
            print("\nError: select_reliability_model_func\n")
            print(err)


    def select_phm_unit_func(self):
        """
            Select PHM Unit Function
        """
        try:
            selected_unit = self.comboBox_configuration_unit.currentText()

            if selected_unit == self.configuration_reliability_unit[0]:
                self.selected_reliability_unit = True

            elif selected_unit == self.configuration_reliability_unit[1]:
                self.selected_reliability_unit = False

            else:
                pass

        except Exception as err:
            print("\nError: select_phm_unit_func\n")
            print(err)


    def set_shape_parameter_func(self):
        """
            Set Shape Parameter Function
        """
        try:
            shape_parameter = float(self.lineEdit_configuration_reliability_shape_parameter.text())
            self.shape_parameter = shape_parameter

        except Exception as err:
            print("\nError: set_shape_parameter_func\n")
            print(err)

# "CONFIGURATION // SET TYPE // SET MODULE TYPES FUNCTIONS" - END -

# "CONFIGURATION // ADD OBJECT 2 // ADD COMPONENT ELEMENT" - START -

    def add_component_element_tab_func(self):
        """
            Add Component Element Tab Function
        """
        self.comboBox_c_ao_ace_select_sub_module.clear()
        selected_module = self.comboBox_c_ao_ace_select_module.currentText()

        if selected_module != "":
            sub_module_list = list(self.main_dict["System"][str(selected_module)].keys())
            self.remove_key(sub_module_list)
            self.comboBox_c_ao_ace_select_sub_module.addItems(sub_module_list)


    def add_component_element_tab_func_2(self):
        """
            Add Component Element Tab Function 2
        """
        self.comboBox_c_ao_ace_select_component.clear()
        self.comboBox_c_ao_ace_select_component_2.clear()
        selected_module = self.comboBox_c_ao_ace_select_module.currentText()
        selected_sub_module = self.comboBox_c_ao_ace_select_sub_module.currentText()

        if selected_sub_module != "":
            component_list = list(self.main_dict["System"][str(selected_module)][str(selected_sub_module)].keys())
            self.remove_key(component_list)

            self.comboBox_c_ao_ace_select_component.addItems(component_list)
            self.comboBox_c_ao_ace_select_component_2.addItems(component_list)


    def add_new_custom_component(self):
        """
            Add New Custom Component Function
        """
        selected_module = self.comboBox_c_ao_ace_select_module.currentText()
        selected_sub_module = self.comboBox_c_ao_ace_select_sub_module.currentText()
        component_name = self.lineEdit_c_ao_ace_set_component_name.text()
        element_name = self.lineEdit_c_ao_ace_set_element_name.text()
        element_variable_name = self.lineEdit_c_ao_ace_set_element_variable_name.text()
        element_value = self.lineEdit_c_ao_ace_set_element_value.text()

        component_list = list(self.main_dict["System"][str(selected_module)][str(selected_sub_module)].keys())
        self.remove_key(component_list)

        if component_name != "":
            if component_name not in component_list:
                self.main_dict["System"][str(selected_module)][str(selected_sub_module)][str(component_name)] = {"Elements": dict(), "Formula": str(), "Failure Rate": float(0.0), "Final Configuration": str()}
                self.add_elements(selected_module, selected_sub_module, component_name, element_name, element_variable_name, element_value)

            else:
                self.add_elements(selected_module, selected_sub_module, component_name, element_name, element_variable_name, element_value)
        else:
            print("\nEnter a component name.\n")


    def add_elements(self, selected_module, selected_sub_module, component_name, element_name, element_variable_name, element_value):
        """
            Add Elements Function
        """
        if element_name != "" and element_variable_name != "" and element_value != "":
            self.main_dict["System"][str(selected_module)][str(selected_sub_module)][str(component_name)]["Elements"][str(element_variable_name)] = {"Name": str(element_name), "Value": float(element_value)}

            self.add_elements_into_list_widget()
            self.lineEdit_c_ao_ace_set_element_name.clear()
            self.lineEdit_c_ao_ace_set_element_variable_name.clear()
            self.lineEdit_c_ao_ace_set_element_value.clear()

            component_list = list(self.main_dict["System"][str(selected_module)][str(selected_sub_module)].keys())
            self.remove_key(component_list)

            self.comboBox_c_ao_ace_select_component.clear()
            self.comboBox_c_ao_ace_select_component_2.clear()
            self.comboBox_c_ao_ace_select_component.addItems(component_list)
            self.comboBox_c_ao_ace_select_component_2.addItems(component_list)

        else:
            if element_name == "":
                print("\nEnter an element name.\n")

            elif element_variable_name == "":
                print("\n Enter a variable name for element " + str(element_name) + ".\n")

            elif element_value == "":
                print("\n Enter a value for element " + str(element_name) + ".\n")


    def add_elements_into_list_widget(self):
        """
            Add Elements Into List Widget Function
        """
        self.listWidget_c_ao_ace_list_of_elements.clear()
        selected_module = self.comboBox_c_ao_ace_select_module.currentText()
        selected_sub_module = self.comboBox_c_ao_ace_select_sub_module.currentText()
        selected_component = self.comboBox_c_ao_ace_select_component.currentText()

        if selected_module != "" and selected_sub_module != "" and selected_component != "":
            element_variable_name_list = list(self.main_dict["System"][str(selected_module)][str(selected_sub_module)][str(selected_component)]["Elements"].keys())
            self.remove_key(element_variable_name_list)

            for item in element_variable_name_list:
                element_name = self.main_dict["System"][str(selected_module)][str(selected_sub_module)][str(selected_component)]["Elements"][str(item)]["Name"]
                element_value = self.main_dict["System"][str(selected_module)][str(selected_sub_module)][str(selected_component)]["Elements"][str(item)]["Value"]
                temp_str = str(element_name) + " : " + str(item) + " ---> " + str(element_value)

                self.listWidget_c_ao_ace_list_of_elements.addItem(temp_str)


    def set_formula_of_component(self):
        """
            Set Formula of Component Function
        """
        try:
            selected_module = str(self.comboBox_c_ao_ace_select_module.currentText())
            selected_sub_module = str(self.comboBox_c_ao_ace_select_sub_module.currentText())
            selected_component = str(self.comboBox_c_ao_ace_select_component_2.currentText())
            formula = self.plainTextEdit_c_ao_ac_set_formula.toPlainText()

            self.main_dict["System"][str(selected_module)][str(selected_sub_module)][str(selected_component)]["Formula"] = str(formula)

            if formula != "" and formula != " " and formula != "None":
                component_dict = self.main_dict["System"][str(selected_module)][str(selected_sub_module)][str(selected_component)]
                formula_result = self.calculate_formula_result(component_dict)

            else:
                formula_result = 0.0

            self.main_dict["System"][str(selected_module)][str(selected_sub_module)][str(selected_component)]["Failure Rate"] = formula_result

            self.lineEdit_c_ao_ace_component_fr.clear()
            self.lineEdit_c_ao_ace_component_fr.setText(str(formula_result))

        except Exception as err:
            print(err)
            temp_string = "Incorrect Operation"
            self.lineEdit_c_ao_ace_component_fr.clear()
            self.lineEdit_c_ao_ace_component_fr.setText(str(temp_string))

    @classmethod
    def calculate_formula_result(cls, component_dict):
        """
            Calculate Formula Result Function
        """
        formula = component_dict["Formula"]
        element_variable_list = list(component_dict["Elements"].keys())

        for item in element_variable_list:
            formula = formula.replace(str(item), str(component_dict["Elements"][str(item)]["Value"]))

        result = eval(formula)

        return result


    def control_component_formula(self):
        """
            Control Component Formula Function
        """
        selected_module = str(self.comboBox_c_ao_ace_select_module.currentText())
        selected_sub_module = str(self.comboBox_c_ao_ace_select_sub_module.currentText())
        selected_component = str(self.comboBox_c_ao_ace_select_component_2.currentText())

        if selected_component != "" and selected_sub_module != "" and selected_component != "":
            formula = str(self.main_dict["System"][str(selected_module)][str(selected_sub_module)][str(selected_component)]["Formula"])
            component_failure_rate = str(self.main_dict["System"][str(selected_module)][str(selected_sub_module)][str(selected_component)]["Failure Rate"])

            self.plainTextEdit_c_ao_ac_set_formula.clear()
            self.lineEdit_c_ao_ace_component_fr.clear()

            if formula != "" and component_failure_rate != "":
                self.plainTextEdit_c_ao_ac_set_formula.setPlainText(str(formula))
                self.lineEdit_c_ao_ace_component_fr.setText(str(component_failure_rate))


    def delete_an_element(self):
        """
            Delete an Element Function
        """
        selected_element_item = self.listWidget_c_ao_ace_list_of_elements.selectedItems()

        element = self.listWidget_c_ao_ace_list_of_elements.currentItem().text()
        ind1 = element.find(':')
        ind2 = element.find('-')
        element_name = element[ind1+2:ind2-1]

        # Remove selected module item from list of sub_modules.
        for item in selected_element_item:
            self.listWidget_c_ao_ace_list_of_elements.takeItem(self.listWidget_c_ao_ace_list_of_elements.row(item))

        selected_module = self.comboBox_c_ao_ace_select_module.currentText()
        selected_sub_module = self.comboBox_c_ao_ace_select_sub_module.currentText()
        selected_component = self.comboBox_c_ao_ace_select_component.currentText()

        if ((selected_module and selected_module and selected_component) != "None") and ((selected_sub_module and selected_sub_module and selected_component) != ""):
            # Remove selected module from main_dict, also.
            del self.main_dict["System"][str(selected_module)][str(selected_sub_module)][str(selected_component)]["Elements"][str(element_name)]
            self.main_dict["System"][str(selected_module)][str(selected_sub_module)][str(selected_component)]["Formula"] = ""
            self.main_dict["System"][str(selected_module)][str(selected_sub_module)][str(selected_component)]["Failure Rate"] = float(0.0)

            self.plainTextEdit_c_ao_ac_set_formula.clear()
            self.lineEdit_c_ao_ace_component_fr.clear()

# "CONFIGURATION // ADD OBJECT 2 // ADD COMPONENT ELEMENT" - END -

# "CONFIGURATION // ADD OBJECT 2 // ADD COMPONENT" - START -

    def add_component_tab_func(self):
        """
            Add Component Tab Function
        """
        self.comboBox_c_ao_ac_select_sub_module.clear()
        selected_module = self.comboBox_c_ao_ac_select_module.currentText()

        if selected_module != "":
            sub_module_list = list(self.main_dict["System"][str(selected_module)].keys())
            self.remove_key(sub_module_list)

            self.comboBox_c_ao_ac_select_sub_module.addItems(sub_module_list)

        self.comboBox_c_ao_ac_select_equipment.clear()
        self.comboBox_c_ao_ac_select_equipment.addItems(self.equipment_list)


    def add_new_component(self):
        """
            Add New Component Function
        """
        selected_module = self.comboBox_c_ao_ac_select_module.currentText()
        selected_sub_module = self.comboBox_c_ao_ac_select_sub_module.currentText()
        component_name = self.lineEdit_c_ao_ac_set_component_name.text()

        # self.control_the_component_fr_is_changed_or_not()

        if self.groupBox_c_ao_ac_component_fr.isChecked():
            component_fr = self.set_only_component_fr()
        elif self.groupBox_c_ao_ac_default_fr.isChecked():
            component_fr = self.set_component_fr_from_default_equipments()
        else:
            component_fr = float(0.0)

        if component_name != "":
            self.main_dict["System"][str(selected_module)][str(selected_sub_module)][str(component_name)] = {"Elements": dict(), "Formula": str(), "Failure Rate": float(0.0), 'Final Configuration': str()}

            if component_fr != "":
                self.main_dict["System"][str(selected_module)][str(selected_sub_module)][str(component_name)]["Failure Rate"] = float(component_fr)
                self.add_component_into_list_widget()
                self.lineEdit_c_ao_ac_set_component_name.clear()
                self.component_list_control_of_set_component_types_tab()
            else:
                print("\nEnter a failure rate value for the component " + str(component_name) + ".\n")
        else:
            print("\nEnter a component name.\n")

        self.groupBox_c_ao_ac_component_fr.setChecked(False)
        self.groupBox_c_ao_ac_default_fr.setChecked(False)


    def set_only_component_fr(self):
        """
            Set Only Component Failure Rate Function
        """
        self.groupBox_c_ao_ac_default_fr.setChecked(False)
        component_fr = self.lineEdit_c_ao_ac_sfr_total_component_fr.text()

        self.lineEdit_c_ao_ac_component_fr.clear()
        self.spinBox_c_ao_ac_sfr_quantity.setValue(1)
        self.lineEdit_c_ao_ac_sfr_total_component_fr.clear()

        return component_fr


    def set_component_fr_from_default_equipments(self):
        """
            Set Component Failure Rate From Default Equipments Function
        """
        self.groupBox_c_ao_ac_component_fr.setChecked(False)
        component_fr = self.lineEdit_c_ao_ac_total_component_fr.text()

        self.lineEdit_c_ao_ac_component_fr_2.clear()
        self.spinBox_c_ao_ac_quantity.setValue(1)
        self.lineEdit_c_ao_ac_total_component_fr.clear()

        return component_fr


    def calculate_component_total_fr_func(self):
        """
            Calculate Component Total Failure Rate Function
        """
        try:
            equipment_fr = float(self.lineEdit_c_ao_ac_component_fr_2.text())
            quantity = float(self.spinBox_c_ao_ac_quantity.value())

            if equipment_fr != 0 and str(equipment_fr) != "":
                calculate_fr = float(equipment_fr * quantity)
                self.lineEdit_c_ao_ac_total_component_fr.setText(str(calculate_fr))

        except ValueError as err:
            print("\nError: calculate_component_total_fr_func\n")
            print(err)


    def calculate_set_fr_component_total_fr_func(self):
        """
            Calculate Set Failure Rate Component Total Failure Rate Func
        """
        try:
            equipment_fr = float(self.lineEdit_c_ao_ac_component_fr.text())
            quantity = float(self.spinBox_c_ao_ac_sfr_quantity.value())

            if equipment_fr != 0 and str(equipment_fr) != "":
                calculate_fr = float(equipment_fr * quantity)
                self.lineEdit_c_ao_ac_sfr_total_component_fr.setText(str(calculate_fr))

        except ValueError as err:
            print("\nError: calculate_set_fr_component_total_fr_func\n")
            print(err)


    def add_component_into_list_widget(self):
        """
            Add Component Into List Widget Function
        """
        self.listWidget_c_ao_ac_list_of_components.clear()
        selected_module = self.comboBox_c_ao_ac_select_module.currentText()
        selected_sub_module = self.comboBox_c_ao_ac_select_sub_module.currentText()

        if selected_module != "" and selected_sub_module != "":
            component_list = list(self.main_dict["System"][str(selected_module)][str(selected_sub_module)].keys())
            self.remove_key(component_list)

            for item in component_list:
                component_fr = self.main_dict["System"][str(selected_module)][str(selected_sub_module)][str(item)]["Failure Rate"]
                temp_str = str(str(item) + str(" \t---> ") + str(component_fr))

                self.listWidget_c_ao_ac_list_of_components.addItem(temp_str)


    def component_set_fr_groupbox_control(self):
        """
            Component Set Failure Rate Groupbox Control Function
        """
        self.groupBox_c_ao_ac_default_fr.setChecked(False)


    def component_default_equipments_groupbox_control(self):
        """
            Component Default Equipments Groupbox Control Function
        """
        self.groupBox_c_ao_ac_component_fr.setChecked(False)
        self.comboBox_c_ao_ac_select_equipment.clear()
        self.comboBox_c_ao_ac_select_equipment.addItems(self.equipment_list)


    # This function added as a comment line
    def control_the_component_fr_is_changed_or_not(self):
        """
            Control the Component Failure Rate is Changed or Not Function
        """
        selected_module = self.comboBox_c_ao_ac_select_module.currentText()
        selected_sub_module = self.comboBox_c_ao_ac_select_sub_module.currentText()

        if ((selected_module and selected_sub_module) != "") and ((selected_module and selected_sub_module) != "None"):
            component_list = list(self.main_dict["System"][str(selected_module)][str(selected_sub_module)].keys())
            self.remove_key(component_list)

            configuration_list = list(self.main_dict["System"][str(selected_module)][str(selected_sub_module)]["Configurations"].keys())
            new_component_name = self.lineEdit_c_ao_ac_set_component_name.text()

            if new_component_name in component_list:
                if self.lineEdit_c_ao_ac_sfr_total_component_fr.text() != "" and self.groupBox_c_ao_ac_component_fr.isChecked():
                    new_component_fr = self.lineEdit_c_ao_ac_sfr_total_component_fr.text()

                elif self.lineEdit_c_ao_ac_component_fr_2.text() != "" and self.groupBox_c_ao_ac_default_fr.isChecked():
                    new_component_fr = self.lineEdit_c_ao_ac_component_fr_2.text()

                for configuration in configuration_list:
                    component_list_in_configuration = list(self.main_dict["System"][str(selected_module)][str(selected_sub_module)]["Configurations"][str(configuration)]["Components"])

                    if new_component_name in component_list_in_configuration:
                        self.main_dict["System"][str(selected_module)][str(selected_sub_module)][str(new_component_name)]["Failure Rate"] = new_component_fr
                        component_fr_list_in_configuration = list()

                        for component in component_list_in_configuration:
                            component_fr_list_in_configuration.append(float(self.main_dict["System"][str(selected_module)][str(selected_sub_module)][str(component)]["Failure Rate"]))

                        type_value = self.main_dict["System"][str(selected_module)][str(selected_sub_module)]["Configurations"][str(configuration)]["Type"]

                        if type_value == "Serial":
                            serial_fr = self.fr_calculation_class.component_serial_failure_rate_calculation(component_fr_list_in_configuration)
                            fr_value = serial_fr

                        elif type_value == "Parallel":
                            parallel_fr = self.fr_calculation_class.component_parallel_failure_rate_calculation(component_fr_list_in_configuration)
                            fr_value = parallel_fr

                        self.main_dict["System"][str(selected_module)][str(selected_sub_module)]["Configurations"][str(configuration)]["Failure Rate"] = float(fr_value)


    def component_add_equipment_func(self):
        """
            Component Add Equipment Function
        """
        selected_equipment = str(self.comboBox_c_ao_ac_select_equipment.currentText())
        self.add_equipment_component_window = QtWidgets.QMainWindow()

        data_path = "lineEdit_c_ao_ac_component_fr_2"

        if selected_equipment == "Mechanical Equipment":
            self.add_equipment_component = MechanicalEquipmentsWindow(self, data_path, False)

        elif selected_equipment == "Electrical Equipment":
            self.add_equipment_component = ElectricalEquipmentsWindow(self, data_path, False)

        self.add_equipment_component.setupUi(self.add_equipment_component_window)
        self.add_equipment_component_window.show()


    def delete_a_component(self):
        """
            Delete a Component Function
        """
        selected_component_item = self.listWidget_c_ao_ac_list_of_components.selectedItems()
        component = self.listWidget_c_ao_ac_list_of_components.currentItem().text()
        ind = component.find(' \t')
        component_name = component[:ind]

        # Remove selected module item from list of sub_modules.
        for item in selected_component_item:
            self.listWidget_c_ao_ac_list_of_components.takeItem(self.listWidget_c_ao_ac_list_of_components.row(item))

        selected_module = self.comboBox_c_ao_ac_select_module.currentText()
        selected_sub_module = self.comboBox_c_ao_ac_select_sub_module.currentText()

        if selected_module != "" and selected_module != "None" and selected_sub_module != "" and selected_sub_module != "None":
            # Remove selected module from main_dict, also.
            del self.main_dict["System"][str(selected_module)][str(selected_sub_module)][str(component_name)]
            self.delete_component_from_configuration_func(selected_module, selected_sub_module, component_name)

            # Remove selected module from comboboxes, also.
            ind1 = self.comboBox_c_ao_ace_select_component.findText(str(component_name))
            self.comboBox_c_ao_ace_select_component.removeItem(ind1)

            ind2 = self.comboBox_c_ao_ace_select_component_2.findText(str(component_name))
            self.comboBox_c_ao_ace_select_component_2.removeItem(ind2)


    def delete_component_from_configuration_func(self, selected_module, selected_sub_module, deleted_component):
        """
            Delete Component From Configuration Function
        """
        configuration_list = list(self.main_dict["System"][str(selected_module)][str(selected_sub_module)]["Configurations"].keys())
        self.remove_key(configuration_list)

        for item in configuration_list:
            if str(deleted_component) in list(self.main_dict["System"][str(selected_module)][str(selected_sub_module)]["Configurations"][str(item)]["Components"]):
                self.main_dict["System"][str(selected_module)][str(selected_sub_module)]["Configurations"][str(item)]["Components"].remove(str(deleted_component))
                temp_count = len(list(self.main_dict["System"][str(selected_module)][str(selected_sub_module)]["Configurations"][str(item)]["Components"]))

                if temp_count == 0:
                    self.main_dict["System"][str(selected_module)][str(selected_sub_module)]["Configurations"][str(item)]["Failure Rate"] = 0.0

        self.convert_configurations_of_components_into_syntax()
        self.component_list_control_of_set_component_types_tab()

# "CONFIGURATION // ADD OBJECT 2 // ADD COMPONENT" - END -

# "CONFIGURATION // ADD OBJECT 2 // ADD SUB MODULE" - START -

    def add_new_sub_module(self):
        """
            Add New Sub Module Function
        """
        selected_module = self.comboBox_c_ao_asm_select_module.currentText()
        sub_module_name = self.lineEdit_c_ao_asm_set_sub_module_name.text()
        # self.control_the_sub_module_fr_is_changed_or_not()

        if self.groupBox_c_ao_asm_sub_module_fr.isChecked():
            sub_module_fr = self.set_only_sub_module_fr()

        elif self.groupBox_c_ao_asm_default_fr.isChecked():
            sub_module_fr = self.set_sub_module_fr_from_default_equipments()

        else:
            sub_module_fr = float(0.0)

        if sub_module_name != "":
            self.main_dict["System"][str(selected_module)][str(sub_module_name)] = {"Failure Rate": float(0.0), 'Configurations': {'Configuration Count': {'Serial': 0, 'Parallel': 0}}, 'Reliability': float(0.0), 'Final Configuration': str()}

            if str(sub_module_fr) != "":
                self.main_dict["System"][str(selected_module)][str(sub_module_name)]["Failure Rate"] = float(sub_module_fr)
                self.add_sub_module_into_list_widget()

            else:
                print("\nEnter a failure rate value for the sub-module " + str(sub_module_name) + ".\n")
        else:
            print("\nEnter a sub-module name.\n")

        self.groupBox_c_ao_asm_sub_module_fr.setChecked(False)
        self.groupBox_c_ao_asm_default_fr.setChecked(False)


    def set_sub_module_listwidget_func(self):
        """
            Set Sub Module Listwidget Function
        """
        self.listWidget_c_ao_asm_list_of_sub_modules.clear()
        selected_module = self.comboBox_c_ao_asm_select_module.currentText()

        if selected_module != "":
            sub_module_list = list(self.main_dict["System"][str(selected_module)].keys())
            self.remove_key(sub_module_list)

            for item in sub_module_list:
                sub_module_fr = self.main_dict["System"][str(selected_module)][str(item)]["Failure Rate"]
                temp_str = str(str(item) + str(" \t---> ") + str(sub_module_fr))

                self.listWidget_c_ao_asm_list_of_sub_modules.addItem(temp_str)


    def set_only_sub_module_fr(self):
        """
            Set Only Sub Module Failure Rate Function
        """
        self.groupBox_c_ao_asm_default_fr.setChecked(False)
        sub_module_fr = self.lineEdit_c_ao_asm_sfr_total_component_fr.text()

        self.lineEdit_c_ao_asm_sub_module_fr.clear()
        self.spinBox_c_ao_asm_sfr_quantity.setValue(1)
        self.lineEdit_c_ao_asm_sfr_total_component_fr.clear()

        return sub_module_fr


    def set_sub_module_fr_from_default_equipments(self):
        """
            Set Sub Module Failure Rate From Default Equipments Function
        """
        self.groupBox_c_ao_asm_sub_module_fr.setChecked(False)
        sub_module_fr = self.lineEdit_c_ao_asm_total_component_fr.text()

        self.lineEdit_c_ao_asm_component_fr.clear()
        self.spinBox_c_ao_asm_quantity.setValue(1)
        self.lineEdit_c_ao_asm_total_component_fr.clear()

        return sub_module_fr


    def calculate_sub_module_total_fr_func(self):
        """
            Calculate Sub Module Total Failure Rate Function
        """
        try:
            equipment_fr = float(self.lineEdit_c_ao_asm_component_fr.text())
            quantity = float(self.spinBox_c_ao_asm_quantity.value())
            equipment_type = str(self.comboBox_c_ao_asm_select_type.currentText())

            if equipment_fr != 0 and str(equipment_fr) != "":
                if equipment_type == "Serial":
                    calculate_fr = float(equipment_fr * quantity)

                else:
                    parallel_count = float(self.fr_calculation_class.parallel_count_calculate_func(quantity))
                    calculate_fr = float(equipment_fr * parallel_count)

                self.lineEdit_c_ao_asm_total_component_fr.setText(str(calculate_fr))

        except ValueError as err:
            print("\nError: calculate_sub_module_total_fr_func\n")
            print(err)


    def calculate_set_fr_sub_module_total_fr_func(self):
        """
            Calculate Set Failure Rate Sub Module Total Failure Rate Function
        """
        try:
            equipment_fr = float(self.lineEdit_c_ao_asm_sub_module_fr.text())
            quantity = float(self.spinBox_c_ao_asm_sfr_quantity.value())
            equipment_type = str(self.comboBox_c_ao_asm_sfr_select_type.currentText())

            if equipment_fr != 0 and str(equipment_fr) != "":
                if equipment_type == "Serial":
                    calculate_fr = float(equipment_fr * quantity)

                else:
                    parallel_count = float(self.fr_calculation_class.parallel_count_calculate_func(quantity))
                    calculate_fr = float(equipment_fr * parallel_count)

                self.lineEdit_c_ao_asm_sfr_total_component_fr.setText(str(calculate_fr))

        except ValueError as err:
            print("\nError: calculate_set_fr_sub_module_total_fr_func\n")
            print(err)


    def add_sub_module_into_list_widget(self):
        """
            Add Sub Module Into Listwidget Function
        """
        self.set_sub_module_listwidget_func()
        self.lineEdit_c_ao_asm_set_sub_module_name.clear()


    def sub_module_set_fr_groupbox_control(self):
        """
            Sub Module Set Failure Rate Groupbox Control Function
        """
        self.groupBox_c_ao_asm_default_fr.setChecked(False)


    def sub_module_default_equipments_groupbox_control(self):
        """
            Sub Module Default Equipments Groupbox Control Function
        """
        self.groupBox_c_ao_asm_sub_module_fr.setChecked(False)
        self.comboBox_c_ao_asm_select_equipment.clear()
        self.comboBox_c_ao_asm_select_equipment.addItems(self.equipment_list)


    # This function added as a comment line
    def control_the_sub_module_fr_is_changed_or_not(self):
        """
            Control the Sub Module Failure Rate is Changed or Not Function
        """
        selected_module = self.comboBox_configuration_st_ssm_select_module.currentText()

        if (selected_module != "") and (selected_module != "None"):
            sub_module_list = list(self.main_dict["System"][str(selected_module)].keys())
            self.remove_key(sub_module_list)

            configuration_list = list(self.main_dict["System"][str(selected_module)]["Configurations"].keys())
            new_sub_module_name = self.lineEdit_c_ao_asm_set_sub_module_name.text()

            if new_sub_module_name in sub_module_list:
                if self.lineEdit_c_ao_asm_sfr_total_component_fr.text() != "" and self.groupBox_c_ao_asm_sub_module_fr.isChecked():
                    new_sub_module_fr = self.lineEdit_c_ao_asm_sfr_total_component_fr.text()

                elif self.lineEdit_c_ao_asm_component_fr.text() != "" and self.groupBox_c_ao_asm_default_fr.isChecked():
                    new_sub_module_fr = self.lineEdit_c_ao_asm_component_fr.text()

                for configuration in configuration_list:
                    sub_module_list_in_configuration = list(self.main_dict["System"][str(selected_module)]["Configurations"][str(configuration)]["Components"])

                    if new_sub_module_name in sub_module_list_in_configuration:
                        self.main_dict["System"][str(selected_module)][str(new_sub_module_name)]["Failure Rate"] = new_sub_module_fr
                        sub_module_fr_list_in_configuration = list()

                        for sub_module in sub_module_list_in_configuration:
                            sub_module_fr_list_in_configuration.append(float(self.main_dict["System"][str(selected_module)][str(sub_module)]["Failure Rate"]))

                        type_value = self.main_dict["System"][str(selected_module)]["Configurations"][str(configuration)]["Type"]

                        if type_value == "Serial":
                            serial_fr = self.fr_calculation_class.component_serial_failure_rate_calculation(sub_module_fr_list_in_configuration)
                            fr_value = serial_fr

                        elif type_value == "Parallel":
                            parallel_fr = self.fr_calculation_class.component_parallel_failure_rate_calculation(sub_module_fr_list_in_configuration)
                            fr_value = parallel_fr

                        self.main_dict["System"][str(selected_module)]["Configurations"][str(configuration)]["Failure Rate"] = float(fr_value)


    def sub_module_add_equipment_func(self, MainWindow):
        """
            Sub Module Add Equipment Function
        """
        selected_equipment = str(self.comboBox_c_ao_asm_select_equipment.currentText())
        self.add_equipment_sub_module_window = QtWidgets.QMainWindow()

        data_path = "lineEdit_c_ao_asm_component_fr"

        if selected_equipment == "Mechanical Equipment":
            self.add_equipment_sub_module = MechanicalEquipmentsWindow(self, data_path, True)

        elif selected_equipment == "Electrical Equipment":
            self.add_equipment_sub_module = ElectricalEquipmentsWindow(self, data_path, True)

        self.add_equipment_sub_module.setupUi(self.add_equipment_sub_module_window)
        self.add_equipment_sub_module_window.show()


    def delete_a_sub_module(self):
        """
            Delete a Sub Module Function
        """
        selected_sub_module_item = self.listWidget_c_ao_asm_list_of_sub_modules.selectedItems()
        sub_module = self.listWidget_c_ao_asm_list_of_sub_modules.currentItem().text()
        ind = sub_module.find(' \t')
        sub_module_name = sub_module[:ind]

        # Remove selected module item from list of sub_modules.
        for item in selected_sub_module_item:
            self.listWidget_c_ao_asm_list_of_sub_modules.takeItem(self.listWidget_c_ao_asm_list_of_sub_modules.row(item))

        selected_module = self.comboBox_c_ao_asm_select_module.currentText()

        if selected_module != "" and selected_module != "None":
            # Remove selected module from main_dict, also.
            del self.main_dict["System"][str(selected_module)][str(sub_module_name)]
            self.delete_sub_module_from_configuration_func(selected_module, sub_module_name)

            # Remove selected module from comboboxes, also.
            ind1 = self.comboBox_c_ao_ac_select_sub_module.findText(str(sub_module_name))
            self.comboBox_c_ao_ac_select_sub_module.removeItem(ind1)

            ind2 = self.comboBox_c_ao_ace_select_sub_module.findText(str(sub_module_name))
            self.comboBox_c_ao_ace_select_sub_module.removeItem(ind2)


    def delete_sub_module_from_configuration_func(self, selected_module, deleted_sub_module):
        """
            Delete Sub Module From Configuration Function
        """
        configuration_list = list(self.main_dict["System"][str(selected_module)]["Configurations"].keys())
        self.remove_key(configuration_list)

        for item in configuration_list:
            if str(deleted_sub_module) in list(self.main_dict["System"][str(selected_module)]["Configurations"][str(item)]["Components"]):
                self.main_dict["System"][str(selected_module)]["Configurations"][str(item)]["Components"].remove(str(deleted_sub_module))
                temp_count = len(list(self.main_dict["System"][str(selected_module)]["Configurations"][str(item)]["Components"]))

                if temp_count == 0:
                    self.main_dict["System"][str(selected_module)]["Configurations"][str(item)]["Failure Rate"] = 0.0

        self.convert_configurations_of_sub_modules_into_syntax()
        self.sub_module_list_control_of_set_sub_module_types_tab()

# "CONFIGURATION // ADD OBJECT 2 // ADD SUB MODULE" - END -

# "CONFIGURATION // ADD OBJECT 2 // ADD MODULE" - START -

    def add_new_module(self):
        """
            Add New Module Function
        """
        module_name = self.lineEdit_c_ao_am_module_name.text()

        #self.control_the_module_fr_is_changed_or_not()

        if self.groupBox_c_ao_am_module_fr.isChecked():
            module_fr = self.set_only_module_fr()

        else:
            module_fr = float(0.0)

        if module_name != "":
            self.main_dict["System"][str(module_name)] = {"Failure Rate": {"Nominal": float(0.0)}, 'Configurations': {'Configuration Count': {'Serial': 0, 'Parallel': 0}}, 'Reliability': {"Nominal": float(0.0)}, 'Final Configuration': str()}

            if module_fr != "":
                self.main_dict["System"][str(module_name)]["Failure Rate"]["Nominal"] = float(module_fr)

                self.set_module_listwidget_func()
                self.add_module_into_combobox()
                self.monitoring_regeneration_module_func()

            else:
                print("\nEnter a failure rate value for the module " + str(module_name) + ".\n")
        else:
            print("\nEnter a module name.\n")

        self.groupBox_c_ao_am_module_fr.setChecked(False)


    def set_module_listwidget_func(self):
        """
            Set Module Listwidget Function
        """
        module_list = list(self.main_dict["System"].keys())
        self.remove_key(module_list)

        self.listWidget_c_ao_am_list_of_modules.clear()

        for item in module_list:
            module_fr = self.main_dict["System"][str(item)]["Failure Rate"]["Nominal"]
            self.listWidget_c_ao_am_list_of_modules.addItem(str(item) + str(" \t---> ") + str(module_fr))


    def add_module_into_am_list_widget(self):
        """
            Add Module Into Add Module Listwidget Function
        """
        module_list = list(self.main_dict["System"].keys())
        self.remove_key(module_list)
        self.listWidget_c_ao_am_list_of_modules.clear()

        for item in module_list:
            module_fr = self.main_dict["System"][str(item)]["Failure Rate"]["Nominal"]
            self.listWidget_c_ao_am_list_of_modules.addItem(str(item) + str(" \t---> ") + str(module_fr))


    def add_module_into_combobox(self):
        """
            Add Module Into Combobox Function
        """
        module_list = list(self.main_dict["System"].keys())
        self.remove_key(module_list)

        self.comboBox_c_ao_asm_select_module.clear()
        self.comboBox_c_ao_asm_select_module.addItems(module_list)

        self.comboBox_c_ao_ac_select_module.clear()
        self.comboBox_c_ao_ac_select_module.addItems(module_list)

        self.comboBox_c_ao_ace_select_module.clear()
        self.comboBox_c_ao_ace_select_module.addItems(module_list)

        self.system_list_control_of_set_module_types_tab()


    def set_only_module_fr(self):
        """
            Set Only Module Failure Rate Function
        """
        module_fr = self.lineEdit_c_ao_am_total_module_fr.text()

        self.lineEdit_c_ao_am_module_fr.clear()
        self.spinBox_c_ao_am_quantity.setValue(1)
        self.lineEdit_c_ao_am_total_module_fr.clear()

        return module_fr


    def calculate_set_fr_module_total_fr_func(self):
        """
            Calculate Set Failure Rate Module Total Failure Rate Function
        """
        try:
            equipment_fr = float(self.lineEdit_c_ao_am_module_fr.text())
            quantity = float(self.spinBox_c_ao_am_quantity.value())
            equipment_type = str(self.comboBox_c_ao_am_select_type.currentText())

            if equipment_fr != 0 and str(equipment_fr) != "":
                if equipment_type == "Serial":
                    calculate_fr = float(equipment_fr * quantity)

                else:
                    parallel_count = float(self.fr_calculation_class.parallel_count_calculate_func(quantity))
                    calculate_fr = float(equipment_fr * parallel_count)

                self.lineEdit_c_ao_am_total_module_fr.setText(str(calculate_fr))

        except ValueError as err:
            print("\nError: calculate_set_fr_module_total_fr_func\n")
            print(err)

    # This function added as a comment line
    def control_the_module_fr_is_changed_or_not(self):
        """
            Control the Module Failure Rate is Changed or Not Function
        """
        module_list = list(self.main_dict["System"].keys())
        self.remove_key(module_list)

        configuration_list = list(self.main_dict["System"]["Configurations"].keys())
        new_module_name = self.lineEdit_c_ao_am_module_name.text()

        if new_module_name in module_list:
            new_module_fr = self.lineEdit_c_ao_am_module_fr.text()

            for configuration in configuration_list:
                module_list_in_configuration = list(self.main_dict["System"]["Configurations"][str(configuration)]["Components"])

                if new_module_name in module_list_in_configuration:
                    self.main_dict["System"][str(new_module_name)]["Failure Rate"]["Nominal"] = new_module_fr
                    module_fr_list_in_configuration = list()

                    for module in module_list_in_configuration:
                        module_fr_list_in_configuration.append(float(self.main_dict["System"][str(module)]["Failure Rate"]["Nominal"]))

                    type_value = self.main_dict["System"]["Configurations"][str(configuration)]["Type"]

                    if type_value == "Serial":
                        serial_fr = self.fr_calculation_class.component_serial_failure_rate_calculation(module_fr_list_in_configuration)
                        fr_value = serial_fr

                    elif type_value == "Parallel":
                        parallel_fr = self.fr_calculation_class.component_parallel_failure_rate_calculation(module_fr_list_in_configuration)
                        fr_value = parallel_fr

                    self.main_dict["System"]["Configurations"][str(configuration)]["Failure Rate"] = float(fr_value)


    def delete_a_module(self):
        """
            Delete a Module Function
        """
        selected_module_item = self.listWidget_c_ao_am_list_of_modules.selectedItems()

        module = self.listWidget_c_ao_am_list_of_modules.currentItem().text()
        ind = module.find(' \t')
        module_name = module[:ind]

        # Remove selected module item from list of modules.
        for item in selected_module_item:
            self.listWidget_c_ao_am_list_of_modules.takeItem(self.listWidget_c_ao_am_list_of_modules.row(item))

        # Remove selected module from main_dict, also.
        del self.main_dict["System"][str(module_name)]
        self.delete_module_from_configuration_func(module_name)

        # Remove selected module from comboboxes, also.
        self.monitoring_regeneration_module_func()

        ind1 = self.comboBox_c_ao_asm_select_module.findText(str(module_name))
        self.comboBox_c_ao_asm_select_module.removeItem(ind1)

        ind2 = self.comboBox_c_ao_ac_select_module.findText(str(module_name))
        self.comboBox_c_ao_ac_select_module.removeItem(ind2)

        ind3 = self.comboBox_c_ao_ace_select_module.findText(str(module_name))
        self.comboBox_c_ao_ace_select_module.removeItem(ind3)


    def delete_module_from_configuration_func(self, deleted_module):
        """
            Delete Module From Configuration Function
        """
        configuration_list = list(self.main_dict["System"]["Configurations"].keys())
        self.remove_key(configuration_list)

        for item in configuration_list:
            if str(deleted_module) in list(self.main_dict["System"]["Configurations"][str(item)]["Components"]):
                self.main_dict["System"]["Configurations"][str(item)]["Components"].remove(str(deleted_module))
                temp_count = len(list(self.main_dict["System"]["Configurations"][str(item)]["Components"]))

                if temp_count == 0:
                    self.main_dict["System"]["Configurations"][str(item)]["Failure Rate"] = 0.0

        self.convert_configurations_of_modules_into_syntax()
        self.system_list_control_of_set_module_types_tab()

# "CONFIGURATION // ADD OBJECT 2 // ADD MODULE" - END -

# ---------------------------------------------------------------------------------------------------------------

    def monitoring_events_func(self):
        """
            Monitoring Events Function
        """
        self.comboBox_m_reliability_module.currentIndexChanged.connect(self.monitoring_module_reliability_func)
        self.pushButton_m_hazard_rate_refresh_sensor.clicked.connect(self.monitoring_show_sensor_list_func)
        self.pushButton_m_hazard_rate_add_sensor.clicked.connect(self.monitoring_hazard_rate_add_sensor_func)
        self.comboBox_m_select_place_failure_rate.currentIndexChanged.connect(self.monitoring_hazard_rate_select_place_func)
        self.comboBox_m_hazard_rate_select_sensor.currentIndexChanged.connect(self.combobox_hazard_rate_select_sensor_func)

# ---------------------------------------------------------------------------------------------------------------

# "MONITORING // HAZARD RATE FUNCTIONS" - START -

    def monitoring_show_sensor_list_func(self):
        """
            Monitoring Show Sensor List Function
        """
        selected_item = str(self.comboBox_m_select_place_failure_rate.currentText())
        sensor_list = list(self.get_all_topics_func())

        if (selected_item in list(self.temperature_sensor_dict.keys())) and selected_item != "" and selected_item != "None":
            temperature_sensor_list = list(self.temperature_sensor_dict[str(selected_item)]["Sensors"].keys())
            distance_list = list()

            if temperature_sensor_list and sensor_list:
                for sensor in sensor_list:
                    if sensor not in temperature_sensor_list:
                        distance_list.append(sensor)

            else:
                distance_list = sensor_list

            self.combobox_hazard_rate_selection_sensor_func(distance_list)

        elif selected_item not in list(self.temperature_sensor_dict.keys()):
            self.combobox_hazard_rate_selection_sensor_func(sensor_list)


    def combobox_hazard_rate_selection_sensor_func(self, sensor_list):
        """
            Combobox Hazard Rate Selection Sensor Function
        """
        self.comboBox_m_hazard_rate_select_sensor.clear()
        self.comboBox_m_hazard_rate_select_sensor.addItem("None")
        self.comboBox_m_hazard_rate_select_sensor.addItems(sensor_list)
        self.lineEdit_m_hazard_rate_select_sensor_value.clear()


    def combobox_hazard_rate_select_sensor_func(self):
        """
            Combobox Hazard Rate Select Sensor Function
        """
        select_topic = self.comboBox_m_hazard_rate_select_sensor.currentText()

        if select_topic != "None" and select_topic != "":
            if self.thread_hazard_rate_sensor_based_control:
                self.thread_hazard_rate_sensor_based.stop_thread()
                self.thread_hazard_rate_sensor_based_control = False

            self.monitoring_hazard_rate_sensor = ROSSensor(select_topic)
            self.start_hazard_rate_sensor_based_thread_func(self.monitoring_hazard_rate_sensor, 0.5)

        elif select_topic == "None":
            if self.thread_hazard_rate_sensor_based_control:
                self.thread_hazard_rate_sensor_based.stop_thread()
                self.thread_hazard_rate_sensor_based_control = False

            self.lineEdit_m_hazard_rate_select_sensor_value.clear()


    def monitoring_hazard_rate_select_place_func(self):
        """
            Monitoring Hazard Rate Select Place Function
        """
        selected_item = str(self.comboBox_m_select_place_failure_rate.currentText())

        if (selected_item in list(self.temperature_sensor_dict.keys())) and selected_item != "" and selected_item != "None":
            self.monitoring_hazard_rate_set_list_widget_sensor_list_func(selected_item)

        elif selected_item not in list(self.temperature_sensor_dict.keys()):
            self.listWidget_monitoring_hazard_rate_sensor_list.clear()
            self.monitoring_show_sensor_list_func()


    def monitoring_hazard_rate_add_sensor_func(self):
        """
            Monitoring Hazard Rate Add Sensor Function
        """
        selected_sensor = str(self.comboBox_m_hazard_rate_select_sensor.currentText())
        selected_item = str(self.comboBox_m_select_place_failure_rate.currentText())

        if selected_item != "" and selected_item != "None" and selected_sensor != "None" and selected_sensor != "":
            if selected_item not in list(self.temperature_sensor_dict.keys()):
                self.temperature_sensor_dict[str(selected_item)] = {"Avarage": self.temperature_t0, "Sensors": dict()}

            self.temperature_sensor_dict[str(selected_item)]["Sensors"][str(selected_sensor)] = dict({"Thread": ROSSensor(selected_sensor), "Value": 0.0})
            self.monitoring_hazard_rate_set_list_widget_sensor_list_func(selected_item)


    def monitoring_hazard_rate_set_list_widget_sensor_list_func(self, selected_item):
        """
            Monitoring Hazard Rate Set Listwidget Sensor List Function
        """
        sensor_list = list(self.temperature_sensor_dict[str(selected_item)]["Sensors"].keys())

        self.listWidget_monitoring_hazard_rate_sensor_list.clear()
        self.listWidget_monitoring_hazard_rate_sensor_list.addItems(sensor_list)
        self.monitoring_show_sensor_list_func()

    @classmethod
    def get_all_topics_func(cls):
        """
            Get All Topics Function
        """
        topic_list = list()
        current_topics = rospy.get_published_topics()

        for topic_name, topic_type in current_topics:
            if topic_name != "/rosout" and topic_name != "/rosout_agg":
                topic_list.append(topic_name)

        topic_list.sort()

        return topic_list

# "MONITORING // HAZARD RATE FUNCTIONS" - END -

# "MONITORING // GENERAL TAB FUNCTIONS" - START -

    def monitoring_general_tab_func(self):
        """
            Monitoring General Tab Function
        """
        self.monitoring_regeneration_module_func()
        self.monitoring_regeneration_system_failure_type_func()
        self.monitoring_regeneration_potc_task_func()


    def monitoring_regeneration_module_func(self):
        """
            Monitoring Regeneration Module Function
        """
        module_list = list(self.main_dict["System"].keys())
        self.remove_key(module_list)
        self.comboBox_m_reliability_module.clear()
        self.comboBox_m_reliability_module.addItem("None")
        self.comboBox_m_reliability_module.addItems(module_list)
        self.comboBox_m_select_place_failure_rate.clear()
        self.comboBox_m_select_place_failure_rate.addItems(module_list)


    def monitoring_regeneration_system_failure_type_func(self):
        """
            Monitoring Regeneration System Failure Type Function
        """
        failure_rate_type_list = self.get_failure_rate_type_func()
        self.comboBox_m_sys_select_failure_rate_type.clear()
        self.comboBox_m_sys_select_failure_rate_type.addItems(failure_rate_type_list)
        self.comboBox_m_sys_select_reliability_type.clear()
        self.comboBox_m_sys_select_reliability_type.addItems(failure_rate_type_list)


    def monitoring_regeneration_potc_task_func(self):
        """
            Monitoring Regeneration POTC Function
        """
        task_list = list(self.prognostic_potc_dict.keys())

        if task_list:
            task_list.sort()
            self.comboBox_m_prognostic_potc_select_task.clear()
            self.comboBox_m_prognostic_potc_select_task.addItems(task_list)

        else:
            self.comboBox_m_prognostic_potc_select_task.clear()
            self.comboBox_m_prognostic_potc_select_task.addItem("None")


    def monitoring_module_reliability_func(self):
        """
            Monitoring Module Reliability Function
        """
        select_module = self.comboBox_m_reliability_module.currentText()

        if select_module != "" and select_module != "None":
            module_fr = self.main_dict["System"][str(select_module)]['Failure Rate']["Nominal"]
            reliability = self.r_calculation_class.reliability_calculate_func(self.phm_gui_time, module_fr, self.selected_reliability_model, self.selected_reliability_unit, self.shape_parameter)

            self.comboBox_m_sys_select_module_reliability_type.clear()
            self.comboBox_m_sys_select_module_reliability_type.addItem("Nominal")

            type_list = list(self.main_dict["System"][str(select_module)]['Failure Rate'].keys())
            type_list.remove("Nominal")

            if type_list:
                self.comboBox_m_sys_select_module_reliability_type.addItems(type_list)

        else:
            reliability = 1.0
            self.comboBox_m_sys_select_module_reliability_type.clear()

        self.lineEdit_m_module_reliability.setText(str(reliability))

# "MONITORING // GENERAL TAB FUNCTIONS" - END -

# PHM POTC FUNCTIONS - START -
    def monitoring_system_robot_task_completion_func(self):
        """
            Monitoring System POTC Function
        """
        system_fr = self.main_dict["System"]["Failure Rate"]["Nominal"]

        # Predict POTC
        if system_fr != 0.0 and self.potc_ros.robot_task_list_control:
            system_reliability = self.main_dict["System"]["Reliability"]["Nominal"]
            system_sb_fr = self.main_dict["System"]["Failure Rate"]["Sensor Based"]
            system_sb_reliability = self.main_dict["System"]["Reliability"]["Sensor Based"]

            predict_potc_class = SimulationRobotTaskCompletion(system_fr, system_reliability, self.selected_reliability_model, self.selected_reliability_unit, self.shape_parameter)
            predict_sb_potc_class = SimulationRobotTaskCompletion(system_sb_fr, system_sb_reliability, self.selected_reliability_model, self.selected_reliability_unit, self.shape_parameter)

            potc_position_list, potc_time_list = predict_potc_class.split_robot_task_list_func(self.potc_ros.robot_task_list)

            potc_position_list = self.potc_task_position_unit_func(potc_position_list, self.selected_reliability_unit)
            potc_time_list = self.potc_task_time_unit_func(potc_time_list, self.selected_reliability_unit)

            self.monitoring_add_prognostic_potc_func(potc_position_list, potc_time_list)

            predict_potc = predict_potc_class.prognostic_calculate_last_potc_func(potc_time_list, potc_position_list)
            predict_sb_potc = predict_sb_potc_class.prognostic_calculate_last_potc_func(potc_time_list, potc_position_list)

            self.potc_main_dict["Predict"]["Nominal"]["POTC"].append(float(predict_potc))
            predict_list_count = len(self.potc_main_dict["Predict"]["Nominal"]["POTC"]) - 1
            self.potc_main_dict["Predict"]["Nominal"]["Time"].append(int(predict_list_count))

            self.potc_main_dict["Predict"]["Sensor Based"]["POTC"].append(float(predict_sb_potc))
            predict_sb_list_count = len(self.potc_main_dict["Predict"]["Sensor Based"]["POTC"]) - 1
            self.potc_main_dict["Predict"]["Sensor Based"]["Time"].append(int(predict_sb_list_count))

            publish_data = dict()
            publish_data["Nominal"] = {"POTC": predict_potc, "Time": predict_potc_class.simulation_time, "Distance": predict_potc_class.simulation_distance}
            publish_data["Sensor Based"] = {"POTC": predict_sb_potc, "Time": predict_sb_potc_class.simulation_time, "Distance": predict_sb_potc_class.simulation_distance}
            self.publisher_gui_predict_potc_func(publish_data)

            # Predict Nominal POTC
            self.lineEdit_m_sys_potc_predict_reliability.setText(str(predict_potc))
            # Predict Sensor Based POTC
            self.lineEdit_m_sys_potc_predict_sb_reliability.setText(str(predict_sb_potc))

            # Predict Time
            self.lineEdit_m_potc_predict_time.setText(str(predict_potc_class.simulation_time))
            # Predict Distance
            self.lineEdit_m_potc_predict_distance.setText(str(predict_potc_class.simulation_distance))
            self.potc_ros.robot_task_list_control = False

        # Actual POTC
        if system_fr != 0.0 and self.potc_ros.task_time_control and self.potc_ros.task_position_control:
            potc_class = RobotTaskCompletion(self.selected_reliability_model, self.selected_reliability_unit, self.shape_parameter)
            potc_sb_class = RobotTaskCompletion(self.selected_reliability_model, self.selected_reliability_unit, self.shape_parameter)

            potc_task_time = self.potc_ros.task_time
            potc_task_position = self.potc_ros.task_position
            potc_task_position = self.potc_task_position_unit_func(potc_task_position, self.selected_reliability_unit)
            potc_task_time = self.potc_task_time_unit_func(potc_task_time, self.selected_reliability_unit)

            system_reliability = self.main_dict["System"]["Reliability"]["Nominal"]

            system_sb_fr = self.main_dict["System"]["Failure Rate"]["Sensor Based"]
            system_sb_reliability = self.main_dict["System"]["Reliability"]["Sensor Based"]

            actual_potc = potc_class.probability_of_task_completion_calculate_func(potc_task_time, potc_task_position, system_fr, system_reliability)
            actual_sb_potc = potc_sb_class.probability_of_task_completion_calculate_func(potc_task_time, potc_task_position, system_sb_fr, system_sb_reliability)

            self.potc_main_dict["Actual"]["Nominal"]["POTC"].append(float(actual_potc))
            actual_list_count = len(self.potc_main_dict["Actual"]["Nominal"]["POTC"]) - 1
            self.potc_main_dict["Actual"]["Nominal"]["Time"].append(int(actual_list_count))

            self.potc_main_dict["Actual"]["Sensor Based"]["POTC"].append(float(actual_sb_potc))
            actual_sb_list_count = len(self.potc_main_dict["Actual"]["Sensor Based"]["POTC"]) - 1
            self.potc_main_dict["Actual"]["Sensor Based"]["Time"].append(int(actual_sb_list_count))

            publish_data = dict()
            publish_data["Nominal"] = {"POTC": potc_class.last_potc, "Time": potc_class.current_potc_time, "Distance": potc_class.current_potc_distance}
            publish_data["Sensor Based"] = {"POTC": potc_sb_class.last_potc, "Time": potc_sb_class.current_potc_time, "Distance": potc_sb_class.current_potc_distance}

            self.publisher_gui_actual_potc_func(publish_data)

            # Actual POTC
            self.lineEdit_m_sys_potc_reliability.setText(str(potc_class.last_potc))
            # Actual Sensor Based POTC
            self.lineEdit_m_sys_potc_sb_reliability.setText(str(potc_sb_class.last_potc))

            # Actual Time
            self.lineEdit_m_potc_actual_time.setText(str(potc_class.current_potc_time))
            # Actual Distance
            self.lineEdit_m_potc_actual_distance.setText(str(potc_class.current_potc_distance))

            self.potc_ros.task_time_control = False
            self.potc_ros.task_position_control = False

    @classmethod
    def potc_task_time_unit_func(cls, time_list, unit_type):
        """
            POTC Task Time Unit Function
        """
        new_time_list = list()
        unit = 1

        if unit_type:
            unit = 3600

        for time_value in time_list:
            new_time = float(time_value / unit)
            new_time_list.append(new_time)

        return new_time_list

    @classmethod
    def potc_task_position_unit_func(cls, position_list, unit_type):
        """
            POTC Task Position Unit Function
        """
        new_position_list = list()
        unit = 1

        if unit_type:
            unit = 1000

        for position in position_list:
            new_position = [float(position[0] / unit), float(position[1] / unit)]
            new_position_list.append(new_position)

        return new_position_list


    def potc_list_split_func(self, potc_list):
        """
            POTC List Split Function
        """
        potc_x_list = list()
        potc_y_list = list()

        potc_list_count = len(potc_list)

        for item in range(potc_list_count):
            potc_x_list.append(potc_list[item][0])
            potc_y_list.append(potc_list[item][1])

        return potc_x_list, potc_y_list


    def plot_prognostic_potc_func(self, MainWindow, x_list, y_list, title_name):
        """
            Plot Prognostic POTC Function
        """
        self.new_graph_window_prognostic_potc = QtWidgets.QMainWindow()
        x_value = x_list
        y_value = y_list
        title = str(title_name)

        self.graph_ui_prognostic_potc = Ui_StaticGraphWindow(x_value, y_value, title)
        self.graph_ui_prognostic_potc.setupUi(self.new_graph_window_prognostic_potc)
        self.new_graph_window_prognostic_potc.show()


    def monitoring_add_prognostic_potc_func(self, position_list, time_list):
        """
            Monitoring Add Prognostic POTC Function
        """
        prognostic_potc_keys_count = len(list(self.prognostic_potc_dict.keys()))
        node_count = len(position_list)
        task_name = str("task_" + str(prognostic_potc_keys_count + 1) + "_node_count_" + str(node_count))

        self.prognostic_potc_dict[str(task_name)] = {"Position": position_list, "Time": time_list}
        self.monitoring_regeneration_potc_task_func()


    def monitoring_prognostic_robot_task_completion_func(self, MainWindow):
        """
            Monitoring Prognostic POTC Function
        """
        try:
            system_fr = self.main_dict["System"]["Failure Rate"]["Nominal"]
            selected_task = str(self.comboBox_m_prognostic_potc_select_task.currentText())
            simulation_count = int(self.lineEdit_m_prognostic_potc_simulation_count.text())

            if system_fr != 0.0 and selected_task != "" and selected_task != "None" and simulation_count > 0 and str(simulation_count) != "":
                system_reliability = self.main_dict["System"]["Reliability"]["Nominal"]
                prognostic_potc_class = SimulationRobotTaskCompletion(system_fr, system_reliability, self.selected_reliability_model, self.selected_reliability_unit, self.shape_parameter)
                potc_position_list = self.prognostic_potc_dict[str(selected_task)]["Position"]
                potc_time_list = self.prognostic_potc_dict[str(selected_task)]["Time"]
                title = str(simulation_count) + " Count Simulation"

                potc_list = prognostic_potc_class.prognostic_calculate_potc_func(simulation_count, potc_time_list, potc_position_list)
                self.prognostic_potc_x_list, self.prognostic_potc_y_list = self.potc_list_split_func(potc_list)

                self.lineEdit_m_prognostic_potc.setText(str(prognostic_potc_class.simulation_potc))
                self.lineEdit_m_prognostic_potc_simulation_time.setText(str(prognostic_potc_class.simulation_time))
                self.plot_prognostic_potc_func(MainWindow, self.prognostic_potc_x_list, self.prognostic_potc_y_list, title)

            elif str(simulation_count) == "":
                self.lineEdit_m_prognostic_potc.setText(str("Simulation Count Giriniz"))
                self.lineEdit_m_prognostic_potc_simulation_time.setText(str("Simulation Count Giriniz"))

            elif selected_task == "" or selected_task == "None":
                self.lineEdit_m_prognostic_potc.setText(str("Lutfen Task Bekleyiniz"))
                self.lineEdit_m_prognostic_potc_simulation_time.setText(str("Lutfen Task Bekleyiniz"))

            else:
                self.lineEdit_m_prognostic_potc.setText(str("Failure Rate Hesaplayiniz"))
                self.lineEdit_m_prognostic_potc_simulation_time.setText(str("Failure Rate Hesaplayiniz"))

        except Exception as err:
            print("\nError: monitoring_prognostic_robot_task_completion_func\n")
            print(err)

# PHM POTC FUNCTIONS - END -

# ---------------------------------------------------------------------------------------------------------------

    def fill_the_file_with_data(self, file_dir, temp_file_name, temp_data):
        """
            Fill The File With Data Function
        """
        current_workspace = self.get_current_workspace()
        file_name = str(str(current_workspace) + str(file_dir) + str(temp_file_name) + '.yaml')

        with open(file_name, 'w') as outfile:
            yaml.dump(temp_data, outfile, default_flow_style=False)

    @classmethod
    def read_data_from_file(cls, selected_file):
        """
            Read Data From File Function
        """
        load_file = file(str(selected_file), 'r')
        temp_dict = yaml.load(load_file)

        return temp_dict

    @classmethod
    def get_current_workspace(cls):
        """
            Get Current Workspace Function
        """
        file_full_path = os.path.dirname(os.path.realpath(__file__))
        directory_name = sys.argv[0].split('/')[-2]
        workspace_name = file_full_path.split(str(directory_name))[0]

        return workspace_name


    def get_failure_rate_type_func(self):
        """
            Get Failure Rate Type Function
        """
        failure_rate_type_list = list(["Nominal"])
        type_list = list(self.main_dict["System"]["Failure Rate"].keys())
        type_list.remove("Nominal")
        failure_rate_type_list.extend(type_list)

        return failure_rate_type_list

# ---------------------------------------------------------------------------------------------------------------

    def start_hazard_rate_sensor_based_thread_func(self, ros, rate):
        """
            Start Hazard Rate Sensor Based Thread Function
        """
        self.thread_hazard_rate_sensor_based_control = True
        self.thread_hazard_rate_sensor_based = HazardRateSensorBasedThread(ros, rate)
        self.thread_hazard_rate_sensor_based.change_value.connect(self.lineEdit_m_hazard_rate_select_sensor_value.setText)
        self.thread_hazard_rate_sensor_based.start()

# ---------------------------------------------------------------------------------------------------------------

class HazardRateSensorBasedThread(QThread):
    """
        Hazard Rate Sensor Based Thread Class
    """
    change_value = pyqtSignal(str)
    error = pyqtSignal()
    def __init__(self, ros, rate):
        QThread.__init__(self)
        self.stop_event = Event()
        self.ros = ros
        self.rate = rate

    def run(self):
        """
            Main Thread Function
        """
        try:
            while not self.stop_event.isSet():
                time.sleep(self.rate)
                read_data = str(eval("self.ros.data"))
                if read_data != "None":
                    self.change_value.emit(read_data)

        except Exception as err:
            self.error.emit()
            raise err

    def stop_thread(self):
        """
            Stop Thread Function
        """
        self.stop_event.set()
        self.wait()


if __name__ == '__main__':
    try:
        rospy.init_node('start_gui')

        app = QtWidgets.QApplication(sys.argv)
        MAIN_WINDOW = QtWidgets.QMainWindow()
        PHM_UI = PHMGui()
        PHM_UI.setupUi(MAIN_WINDOW)
        MAIN_WINDOW.setWindowTitle('PHM GUI')
        MAIN_WINDOW.show()
        sys.exit(app.exec_())

    except Exception as err:
        print(err)
