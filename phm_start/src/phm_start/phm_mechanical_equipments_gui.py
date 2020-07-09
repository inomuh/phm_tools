#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
    PHM Mechanical Equipments Gui
"""

from PyQt5 import QtCore, QtGui, QtWidgets
from phm_hazard_rate_calculation.class_mechanical_equipment import MechanicalEquipment

class MechanicalEquipmentsWindow(object):
    """
        Mechanical Equipment Window Class
    """
    def __init__(self, ui_class, data_path, equipment_type):
        self.ui_class = ui_class
        self.data_path = data_path
        self.equipment_type = equipment_type

        self.m_eq = MechanicalEquipment()

        self.title_name = "Mechanical Equipments"

        if self.equipment_type:
            self.title_name = str("Sub Module " + self.title_name)

        else:
            self.title_name = str("Component " + self.title_name)


        self.mechanical_equipments_dict = self.ui_class.mechanical_equipments_dict
        self.shaft_sections_dict = dict()


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
        self.tabWidget_mechanical_equipments = QtWidgets.QTabWidget(self.centralwidget)
        self.tabWidget_mechanical_equipments.setGeometry(QtCore.QRect(0, 0, 1100, 900))
        self.tabWidget_mechanical_equipments.setObjectName("tabWidget_mechanical_equipments")
        self.tab_springs = QtWidgets.QWidget()
        self.tab_springs.setObjectName("tab_springs")
        self.comboBox_sp_material_type = QtWidgets.QComboBox(self.tab_springs)
        self.comboBox_sp_material_type.setGeometry(QtCore.QRect(140, 10, 131, 27))
        self.comboBox_sp_material_type.setObjectName("comboBox_sp_material_type")
        self.label_sp_material_type = QtWidgets.QLabel(self.tab_springs)
        self.label_sp_material_type.setGeometry(QtCore.QRect(30, 10, 101, 31))
        self.label_sp_material_type.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_sp_material_type.setObjectName("label_sp_material_type")
        self.comboBox_sp_material = QtWidgets.QComboBox(self.tab_springs)
        self.comboBox_sp_material.setGeometry(QtCore.QRect(140, 40, 131, 27))
        self.comboBox_sp_material.setObjectName("comboBox_sp_material")
        self.label_sp_material = QtWidgets.QLabel(self.tab_springs)
        self.label_sp_material.setGeometry(QtCore.QRect(30, 40, 101, 31))
        self.label_sp_material.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_sp_material.setObjectName("label_sp_material")
        self.lineEdit_sp_c_g = QtWidgets.QLineEdit(self.tab_springs)
        self.lineEdit_sp_c_g.setGeometry(QtCore.QRect(140, 80, 131, 27))
        self.lineEdit_sp_c_g.setReadOnly(True)
        self.lineEdit_sp_c_g.setObjectName("lineEdit_sp_c_g")
        self.label_sp_c_g = QtWidgets.QLabel(self.tab_springs)
        self.label_sp_c_g.setGeometry(QtCore.QRect(-10, 70, 141, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_sp_c_g.setFont(font)
        self.label_sp_c_g.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_sp_c_g.setWordWrap(True)
        self.label_sp_c_g.setObjectName("label_sp_c_g")
        self.line_58 = QtWidgets.QFrame(self.tab_springs)
        self.line_58.setGeometry(QtCore.QRect(10, 120, 300, 3))
        self.line_58.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_58.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_58.setObjectName("line_58")
        self.lineEdit_sp_d_w = QtWidgets.QLineEdit(self.tab_springs)
        self.lineEdit_sp_d_w.setGeometry(QtCore.QRect(141, 140, 131, 27))
        self.lineEdit_sp_d_w.setObjectName("lineEdit_sp_d_w")
        self.label_sp_d_w = QtWidgets.QLabel(self.tab_springs)
        self.label_sp_d_w.setGeometry(QtCore.QRect(30, 130, 101, 31))
        self.label_sp_d_w.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_sp_d_w.setWordWrap(True)
        self.label_sp_d_w.setObjectName("label_sp_d_w")
        self.label_sp_c_dw = QtWidgets.QLabel(self.tab_springs)
        self.label_sp_c_dw.setGeometry(QtCore.QRect(10, 220, 121, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_sp_c_dw.setFont(font)
        self.label_sp_c_dw.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_sp_c_dw.setWordWrap(True)
        self.label_sp_c_dw.setObjectName("label_sp_c_dw")
        self.lineEdit_sp_c_dw = QtWidgets.QLineEdit(self.tab_springs)
        self.lineEdit_sp_c_dw.setGeometry(QtCore.QRect(140, 230, 131, 27))
        self.lineEdit_sp_c_dw.setReadOnly(True)
        self.lineEdit_sp_c_dw.setObjectName("lineEdit_sp_c_dw")
        self.lineEdit_sp_c_dc = QtWidgets.QLineEdit(self.tab_springs)
        self.lineEdit_sp_c_dc.setGeometry(QtCore.QRect(140, 280, 131, 27))
        self.lineEdit_sp_c_dc.setReadOnly(True)
        self.lineEdit_sp_c_dc.setObjectName("lineEdit_sp_c_dc")
        self.line_60 = QtWidgets.QFrame(self.tab_springs)
        self.line_60.setGeometry(QtCore.QRect(10, 380, 300, 3))
        self.line_60.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_60.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_60.setObjectName("line_60")
        self.label_sp_c_dc = QtWidgets.QLabel(self.tab_springs)
        self.label_sp_c_dc.setGeometry(QtCore.QRect(10, 270, 121, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_sp_c_dc.setFont(font)
        self.label_sp_c_dc.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_sp_c_dc.setWordWrap(True)
        self.label_sp_c_dc.setObjectName("label_sp_c_dc")
        self.lineEdit_sp_d_c = QtWidgets.QLineEdit(self.tab_springs)
        self.lineEdit_sp_d_c.setGeometry(QtCore.QRect(140, 180, 131, 27))
        self.lineEdit_sp_d_c.setObjectName("lineEdit_sp_d_c")
        self.label_sp_d_c = QtWidgets.QLabel(self.tab_springs)
        self.label_sp_d_c.setGeometry(QtCore.QRect(29, 170, 101, 31))
        self.label_sp_d_c.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_sp_d_c.setWordWrap(True)
        self.label_sp_d_c.setObjectName("label_sp_d_c")
        self.lineEdit_sp_c_n = QtWidgets.QLineEdit(self.tab_springs)
        self.lineEdit_sp_c_n.setGeometry(QtCore.QRect(140, 440, 131, 27))
        self.lineEdit_sp_c_n.setReadOnly(True)
        self.lineEdit_sp_c_n.setObjectName("lineEdit_sp_c_n")
        self.line_61 = QtWidgets.QFrame(self.tab_springs)
        self.line_61.setGeometry(QtCore.QRect(10, 480, 300, 3))
        self.line_61.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_61.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_61.setObjectName("line_61")
        self.lineEdit_sp_n_a = QtWidgets.QLineEdit(self.tab_springs)
        self.lineEdit_sp_n_a.setGeometry(QtCore.QRect(141, 400, 131, 27))
        self.lineEdit_sp_n_a.setObjectName("lineEdit_sp_n_a")
        self.label_sp_c_n = QtWidgets.QLabel(self.tab_springs)
        self.label_sp_c_n.setGeometry(QtCore.QRect(20, 430, 111, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_sp_c_n.setFont(font)
        self.label_sp_c_n.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_sp_c_n.setWordWrap(True)
        self.label_sp_c_n.setObjectName("label_sp_c_n")
        self.label_sp_n_a = QtWidgets.QLabel(self.tab_springs)
        self.label_sp_n_a.setGeometry(QtCore.QRect(40, 390, 91, 31))
        self.label_sp_n_a.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_sp_n_a.setWordWrap(True)
        self.label_sp_n_a.setObjectName("label_sp_n_a")
        self.lineEdit_sp_l1_l2 = QtWidgets.QLineEdit(self.tab_springs)
        self.lineEdit_sp_l1_l2.setGeometry(QtCore.QRect(480, 100, 131, 27))
        self.lineEdit_sp_l1_l2.setReadOnly(True)
        self.lineEdit_sp_l1_l2.setObjectName("lineEdit_sp_l1_l2")
        self.label_sp_l_1 = QtWidgets.QLabel(self.tab_springs)
        self.label_sp_l_1.setGeometry(QtCore.QRect(340, 10, 131, 31))
        self.label_sp_l_1.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_sp_l_1.setWordWrap(True)
        self.label_sp_l_1.setObjectName("label_sp_l_1")
        self.lineEdit_sp_l_1 = QtWidgets.QLineEdit(self.tab_springs)
        self.lineEdit_sp_l_1.setGeometry(QtCore.QRect(481, 20, 131, 27))
        self.lineEdit_sp_l_1.setObjectName("lineEdit_sp_l_1")
        self.line_62 = QtWidgets.QFrame(self.tab_springs)
        self.line_62.setGeometry(QtCore.QRect(340, 140, 300, 3))
        self.line_62.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_62.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_62.setObjectName("line_62")
        self.label_sp_l1_l2 = QtWidgets.QLabel(self.tab_springs)
        self.label_sp_l1_l2.setGeometry(QtCore.QRect(350, 90, 121, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_sp_l1_l2.setFont(font)
        self.label_sp_l1_l2.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_sp_l1_l2.setWordWrap(True)
        self.label_sp_l1_l2.setObjectName("label_sp_l1_l2")
        self.lineEdit_sp_l_2 = QtWidgets.QLineEdit(self.tab_springs)
        self.lineEdit_sp_l_2.setGeometry(QtCore.QRect(481, 60, 131, 27))
        self.lineEdit_sp_l_2.setObjectName("lineEdit_sp_l_2")
        self.label_sp_l_2 = QtWidgets.QLabel(self.tab_springs)
        self.label_sp_l_2.setGeometry(QtCore.QRect(340, 50, 131, 31))
        self.label_sp_l_2.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_sp_l_2.setWordWrap(True)
        self.label_sp_l_2.setObjectName("label_sp_l_2")
        self.line_63 = QtWidgets.QFrame(self.tab_springs)
        self.line_63.setGeometry(QtCore.QRect(330, 10, 3, 450))
        self.line_63.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_63.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_63.setObjectName("line_63")
        self.label_sp_material_2 = QtWidgets.QLabel(self.tab_springs)
        self.label_sp_material_2.setGeometry(QtCore.QRect(370, 150, 101, 31))
        self.label_sp_material_2.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_sp_material_2.setObjectName("label_sp_material_2")
        self.comboBox_sp_material_2 = QtWidgets.QComboBox(self.tab_springs)
        self.comboBox_sp_material_2.setGeometry(QtCore.QRect(480, 150, 131, 27))
        self.comboBox_sp_material_2.setObjectName("comboBox_sp_material_2")
        self.label_sp_c_y = QtWidgets.QLabel(self.tab_springs)
        self.label_sp_c_y.setGeometry(QtCore.QRect(330, 180, 141, 41))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_sp_c_y.setFont(font)
        self.label_sp_c_y.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_sp_c_y.setWordWrap(True)
        self.label_sp_c_y.setObjectName("label_sp_c_y")
        self.lineEdit_sp_c_y = QtWidgets.QLineEdit(self.tab_springs)
        self.lineEdit_sp_c_y.setGeometry(QtCore.QRect(480, 190, 131, 27))
        self.lineEdit_sp_c_y.setReadOnly(True)
        self.lineEdit_sp_c_y.setObjectName("lineEdit_sp_c_y")
        self.line_64 = QtWidgets.QFrame(self.tab_springs)
        self.line_64.setGeometry(QtCore.QRect(340, 230, 300, 3))
        self.line_64.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_64.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_64.setObjectName("line_64")
        self.lineEdit_sp_c_k = QtWidgets.QLineEdit(self.tab_springs)
        self.lineEdit_sp_c_k.setGeometry(QtCore.QRect(140, 340, 131, 27))
        self.lineEdit_sp_c_k.setReadOnly(True)
        self.lineEdit_sp_c_k.setObjectName("lineEdit_sp_c_k")
        self.label_sp_c_k = QtWidgets.QLabel(self.tab_springs)
        self.label_sp_c_k.setGeometry(QtCore.QRect(0, 310, 131, 61))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_sp_c_k.setFont(font)
        self.label_sp_c_k.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_sp_c_k.setWordWrap(True)
        self.label_sp_c_k.setObjectName("label_sp_c_k")
        self.lineEdit_sp_c_r = QtWidgets.QLineEdit(self.tab_springs)
        self.lineEdit_sp_c_r.setGeometry(QtCore.QRect(481, 250, 131, 27))
        self.lineEdit_sp_c_r.setText("")
        self.lineEdit_sp_c_r.setObjectName("lineEdit_sp_c_r")
        self.label_sp_c_r = QtWidgets.QLabel(self.tab_springs)
        self.label_sp_c_r.setGeometry(QtCore.QRect(350, 240, 121, 31))
        self.label_sp_c_r.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_sp_c_r.setWordWrap(True)
        self.label_sp_c_r.setObjectName("label_sp_c_r")
        self.lineEdit_sp_c_cs = QtWidgets.QLineEdit(self.tab_springs)
        self.lineEdit_sp_c_cs.setGeometry(QtCore.QRect(480, 340, 131, 27))
        self.lineEdit_sp_c_cs.setReadOnly(True)
        self.lineEdit_sp_c_cs.setObjectName("lineEdit_sp_c_cs")
        self.label_sp_c_cs = QtWidgets.QLabel(self.tab_springs)
        self.label_sp_c_cs.setGeometry(QtCore.QRect(360, 310, 111, 61))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_sp_c_cs.setFont(font)
        self.label_sp_c_cs.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_sp_c_cs.setWordWrap(True)
        self.label_sp_c_cs.setObjectName("label_sp_c_cs")
        self.line_66 = QtWidgets.QFrame(self.tab_springs)
        self.line_66.setGeometry(QtCore.QRect(340, 380, 300, 3))
        self.line_66.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_66.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_66.setObjectName("line_66")
        self.lineEdit_sp_c_m = QtWidgets.QLineEdit(self.tab_springs)
        self.lineEdit_sp_c_m.setGeometry(QtCore.QRect(481, 290, 131, 27))
        self.lineEdit_sp_c_m.setText("")
        self.lineEdit_sp_c_m.setObjectName("lineEdit_sp_c_m")
        self.label_sp_c_m = QtWidgets.QLabel(self.tab_springs)
        self.label_sp_c_m.setGeometry(QtCore.QRect(350, 280, 121, 31))
        self.label_sp_c_m.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_sp_c_m.setWordWrap(True)
        self.label_sp_c_m.setObjectName("label_sp_c_m")
        self.groupBox_sp_lambda_sp = QtWidgets.QGroupBox(self.tab_springs)
        self.groupBox_sp_lambda_sp.setGeometry(QtCore.QRect(450, 640, 200, 80))
        self.groupBox_sp_lambda_sp.setObjectName("groupBox_sp_lambda_sp")
        self.lineEdit_sp_lamda_sp = QtWidgets.QLineEdit(self.groupBox_sp_lambda_sp)
        self.lineEdit_sp_lamda_sp.setGeometry(QtCore.QRect(0, 30, 200, 27))
        self.lineEdit_sp_lamda_sp.setReadOnly(True)
        self.lineEdit_sp_lamda_sp.setObjectName("lineEdit_sp_lamda_sp")
        self.lineEdit_sp_warning_box = QtWidgets.QLineEdit(self.tab_springs)
        self.lineEdit_sp_warning_box.setGeometry(QtCore.QRect(10, 640, 330, 70))
        self.lineEdit_sp_warning_box.setReadOnly(True)
        self.lineEdit_sp_warning_box.setObjectName("lineEdit_sp_warning_box")
        self.addButton_c_me_springs = QtWidgets.QPushButton(self.tab_springs)
        self.addButton_c_me_springs.setGeometry(QtCore.QRect(925, 650, 125, 50))
        self.addButton_c_me_springs.setObjectName("addButton_c_me_springs")
        self.tabWidget_mechanical_equipments.addTab(self.tab_springs, "")
        self.tab_gears = QtWidgets.QWidget()
        self.tab_gears.setObjectName("tab_gears")
        self.groupBox_gr_lambda_g = QtWidgets.QGroupBox(self.tab_gears)
        self.groupBox_gr_lambda_g.setGeometry(QtCore.QRect(450, 640, 200, 80))
        self.groupBox_gr_lambda_g.setObjectName("groupBox_gr_lambda_g")
        self.lineEdit_gr_lamda_g = QtWidgets.QLineEdit(self.groupBox_gr_lambda_g)
        self.lineEdit_gr_lamda_g.setGeometry(QtCore.QRect(0, 30, 200, 27))
        self.lineEdit_gr_lamda_g.setReadOnly(True)
        self.lineEdit_gr_lamda_g.setObjectName("lineEdit_gr_lamda_g")
        self.line = QtWidgets.QFrame(self.tab_gears)
        self.line.setGeometry(QtCore.QRect(10, 120, 320, 3))
        self.line.setFrameShape(QtWidgets.QFrame.HLine)
        self.line.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line.setObjectName("line")
        self.line_2 = QtWidgets.QFrame(self.tab_gears)
        self.line_2.setGeometry(QtCore.QRect(10, 240, 320, 3))
        self.line_2.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_2.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_2.setObjectName("line_2")
        self.line_3 = QtWidgets.QFrame(self.tab_gears)
        self.line_3.setGeometry(QtCore.QRect(10, 360, 320, 3))
        self.line_3.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_3.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_3.setObjectName("line_3")
        self.line_4 = QtWidgets.QFrame(self.tab_gears)
        self.line_4.setGeometry(QtCore.QRect(10, 450, 320, 3))
        self.line_4.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_4.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_4.setObjectName("line_4")
        self.line_5 = QtWidgets.QFrame(self.tab_gears)
        self.line_5.setGeometry(QtCore.QRect(340, 10, 20, 441))
        self.line_5.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_5.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_5.setObjectName("line_5")
        self.line_6 = QtWidgets.QFrame(self.tab_gears)
        self.line_6.setGeometry(QtCore.QRect(360, 220, 351, 16))
        self.line_6.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_6.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_6.setObjectName("line_6")
        self.line_7 = QtWidgets.QFrame(self.tab_gears)
        self.line_7.setGeometry(QtCore.QRect(360, 130, 351, 16))
        self.line_7.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_7.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_7.setObjectName("line_7")
        self.line_8 = QtWidgets.QFrame(self.tab_gears)
        self.line_8.setGeometry(QtCore.QRect(360, 360, 351, 16))
        self.line_8.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_8.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_8.setObjectName("line_8")
        self.lineEdit_gr_revolutions = QtWidgets.QLineEdit(self.tab_gears)
        self.lineEdit_gr_revolutions.setGeometry(QtCore.QRect(220, 40, 113, 27))
        self.lineEdit_gr_revolutions.setObjectName("lineEdit_gr_revolutions")
        self.label_gr_revolutions = QtWidgets.QLabel(self.tab_gears)
        self.label_gr_revolutions.setGeometry(QtCore.QRect(120, 40, 91, 31))
        self.label_gr_revolutions.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_gr_revolutions.setObjectName("label_gr_revolutions")
        self.lineEdit_gr_rpm = QtWidgets.QLineEdit(self.tab_gears)
        self.lineEdit_gr_rpm.setGeometry(QtCore.QRect(220, 10, 113, 27))
        self.lineEdit_gr_rpm.setObjectName("lineEdit_gr_rpm")
        self.label_gr_rpm = QtWidgets.QLabel(self.tab_gears)
        self.label_gr_rpm.setGeometry(QtCore.QRect(119, 10, 91, 31))
        self.label_gr_rpm.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_gr_rpm.setObjectName("label_gr_rpm")
        self.label_gr_lambda_g_b = QtWidgets.QLabel(self.tab_gears)
        self.label_gr_lambda_g_b.setGeometry(QtCore.QRect(20, 70, 191, 21))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_gr_lambda_g_b.setFont(font)
        self.label_gr_lambda_g_b.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_gr_lambda_g_b.setWordWrap(True)
        self.label_gr_lambda_g_b.setObjectName("label_gr_lambda_g_b")
        self.lineEdit_gr_lambda_g_b = QtWidgets.QLineEdit(self.tab_gears)
        self.lineEdit_gr_lambda_g_b.setGeometry(QtCore.QRect(220, 70, 113, 27))
        self.lineEdit_gr_lambda_g_b.setReadOnly(True)
        self.lineEdit_gr_lambda_g_b.setObjectName("lineEdit_gr_lambda_g_b")
        self.label_gr_v_d = QtWidgets.QLabel(self.tab_gears)
        self.label_gr_v_d.setGeometry(QtCore.QRect(60, 170, 151, 31))
        self.label_gr_v_d.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_gr_v_d.setObjectName("label_gr_v_d")
        self.lineEdit_gr_v0 = QtWidgets.QLineEdit(self.tab_gears)
        self.lineEdit_gr_v0.setGeometry(QtCore.QRect(220, 140, 113, 27))
        self.lineEdit_gr_v0.setObjectName("lineEdit_gr_v0")
        self.label_gr_v_0 = QtWidgets.QLabel(self.tab_gears)
        self.label_gr_v_0.setGeometry(QtCore.QRect(60, 140, 151, 31))
        self.label_gr_v_0.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_gr_v_0.setObjectName("label_gr_v_0")
        self.lineEdit_gr_v_d = QtWidgets.QLineEdit(self.tab_gears)
        self.lineEdit_gr_v_d.setGeometry(QtCore.QRect(220, 170, 113, 27))
        self.lineEdit_gr_v_d.setObjectName("lineEdit_gr_v_d")
        self.label_gr_c_gs = QtWidgets.QLabel(self.tab_gears)
        self.label_gr_c_gs.setGeometry(QtCore.QRect(20, 200, 191, 21))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_gr_c_gs.setFont(font)
        self.label_gr_c_gs.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_gr_c_gs.setWordWrap(True)
        self.label_gr_c_gs.setObjectName("label_gr_c_gs")
        self.lineEdit_gr_c_gs = QtWidgets.QLineEdit(self.tab_gears)
        self.lineEdit_gr_c_gs.setGeometry(QtCore.QRect(220, 200, 113, 27))
        self.lineEdit_gr_c_gs.setReadOnly(True)
        self.lineEdit_gr_c_gs.setObjectName("lineEdit_gr_c_gs")
        self.label_gr_l_d = QtWidgets.QLabel(self.tab_gears)
        self.label_gr_l_d.setGeometry(QtCore.QRect(100, 290, 111, 31))
        self.label_gr_l_d.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_gr_l_d.setObjectName("label_gr_l_d")
        self.label_gr_l_0 = QtWidgets.QLabel(self.tab_gears)
        self.label_gr_l_0.setGeometry(QtCore.QRect(70, 260, 141, 31))
        self.label_gr_l_0.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_gr_l_0.setObjectName("label_gr_l_0")
        self.lineEdit_gr_l_d = QtWidgets.QLineEdit(self.tab_gears)
        self.lineEdit_gr_l_d.setGeometry(QtCore.QRect(220, 290, 113, 27))
        self.lineEdit_gr_l_d.setObjectName("lineEdit_gr_l_d")
        self.lineEdit_gr_l_0 = QtWidgets.QLineEdit(self.tab_gears)
        self.lineEdit_gr_l_0.setGeometry(QtCore.QRect(220, 260, 113, 27))
        self.lineEdit_gr_l_0.setObjectName("lineEdit_gr_l_0")
        self.label_gr_c_gp = QtWidgets.QLabel(self.tab_gears)
        self.label_gr_c_gp.setGeometry(QtCore.QRect(10, 320, 201, 21))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_gr_c_gp.setFont(font)
        self.label_gr_c_gp.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_gr_c_gp.setWordWrap(True)
        self.label_gr_c_gp.setObjectName("label_gr_c_gp")
        self.lineEdit_gr_c_gp = QtWidgets.QLineEdit(self.tab_gears)
        self.lineEdit_gr_c_gp.setGeometry(QtCore.QRect(220, 320, 113, 27))
        self.lineEdit_gr_c_gp.setReadOnly(True)
        self.lineEdit_gr_c_gp.setObjectName("lineEdit_gr_c_gp")
        self.lineEdit_gr_a_e = QtWidgets.QLineEdit(self.tab_gears)
        self.lineEdit_gr_a_e.setGeometry(QtCore.QRect(220, 380, 113, 27))
        self.lineEdit_gr_a_e.setObjectName("lineEdit_gr_a_e")
        self.label_gr_a_e = QtWidgets.QLabel(self.tab_gears)
        self.label_gr_a_e.setGeometry(QtCore.QRect(0, 380, 211, 31))
        self.label_gr_a_e.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_gr_a_e.setObjectName("label_gr_a_e")
        self.lineEdit_gr_c_ga = QtWidgets.QLineEdit(self.tab_gears)
        self.lineEdit_gr_c_ga.setGeometry(QtCore.QRect(220, 410, 113, 27))
        self.lineEdit_gr_c_ga.setReadOnly(True)
        self.lineEdit_gr_c_ga.setObjectName("lineEdit_gr_c_ga")
        self.label_gr_c_ga = QtWidgets.QLabel(self.tab_gears)
        self.label_gr_c_ga.setGeometry(QtCore.QRect(10, 410, 201, 21))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_gr_c_ga.setFont(font)
        self.label_gr_c_ga.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_gr_c_ga.setWordWrap(True)
        self.label_gr_c_ga.setObjectName("label_gr_c_ga")
        self.label_gr_vl = QtWidgets.QLabel(self.tab_gears)
        self.label_gr_vl.setGeometry(QtCore.QRect(400, 40, 171, 41))
        self.label_gr_vl.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_gr_vl.setWordWrap(True)
        self.label_gr_vl.setObjectName("label_gr_vl")
        self.label_gr_v0 = QtWidgets.QLabel(self.tab_gears)
        self.label_gr_v0.setGeometry(QtCore.QRect(370, 0, 201, 41))
        self.label_gr_v0.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_gr_v0.setWordWrap(True)
        self.label_gr_v0.setObjectName("label_gr_v0")
        self.lineEdit_gr_vl = QtWidgets.QLineEdit(self.tab_gears)
        self.lineEdit_gr_vl.setGeometry(QtCore.QRect(590, 50, 113, 27))
        self.lineEdit_gr_vl.setObjectName("lineEdit_gr_vl")
        self.lineEdit_gr_v0_4 = QtWidgets.QLineEdit(self.tab_gears)
        self.lineEdit_gr_v0_4.setGeometry(QtCore.QRect(590, 10, 113, 27))
        self.lineEdit_gr_v0_4.setObjectName("lineEdit_gr_v0_4")
        self.label_gr_c_gl = QtWidgets.QLabel(self.tab_gears)
        self.label_gr_c_gl.setGeometry(QtCore.QRect(360, 90, 211, 21))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_gr_c_gl.setFont(font)
        self.label_gr_c_gl.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_gr_c_gl.setWordWrap(True)
        self.label_gr_c_gl.setObjectName("label_gr_c_gl")
        self.lineEdit_gr_c_gl = QtWidgets.QLineEdit(self.tab_gears)
        self.lineEdit_gr_c_gl.setGeometry(QtCore.QRect(590, 90, 113, 27))
        self.lineEdit_gr_c_gl.setReadOnly(True)
        self.lineEdit_gr_c_gl.setObjectName("lineEdit_gr_c_gl")
        self.lineEdit_gr_t_at = QtWidgets.QLineEdit(self.tab_gears)
        self.lineEdit_gr_t_at.setGeometry(QtCore.QRect(590, 150, 113, 27))
        self.lineEdit_gr_t_at.setObjectName("lineEdit_gr_t_at")
        self.label_gr_t_at = QtWidgets.QLabel(self.tab_gears)
        self.label_gr_t_at.setGeometry(QtCore.QRect(340, 150, 241, 31))
        self.label_gr_t_at.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_gr_t_at.setObjectName("label_gr_t_at")
        self.lineEdit_gr_c_gt = QtWidgets.QLineEdit(self.tab_gears)
        self.lineEdit_gr_c_gt.setGeometry(QtCore.QRect(590, 180, 113, 27))
        self.lineEdit_gr_c_gt.setReadOnly(True)
        self.lineEdit_gr_c_gt.setObjectName("lineEdit_gr_c_gt")
        self.label_gr_c_gt = QtWidgets.QLabel(self.tab_gears)
        self.label_gr_c_gt.setGeometry(QtCore.QRect(350, 180, 231, 21))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_gr_c_gt.setFont(font)
        self.label_gr_c_gt.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_gr_c_gt.setWordWrap(True)
        self.label_gr_c_gt.setObjectName("label_gr_c_gt")
        self.label_gr_load_character = QtWidgets.QLabel(self.tab_gears)
        self.label_gr_load_character.setGeometry(QtCore.QRect(420, 280, 151, 31))
        self.label_gr_load_character.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_gr_load_character.setObjectName("label_gr_load_character")
        self.comboBox_gr_prime_mover = QtWidgets.QComboBox(self.tab_gears)
        self.comboBox_gr_prime_mover.setGeometry(QtCore.QRect(580, 250, 131, 27))
        self.comboBox_gr_prime_mover.setObjectName("comboBox_gr_prime_mover")
        self.comboBox_gr_load_character = QtWidgets.QComboBox(self.tab_gears)
        self.comboBox_gr_load_character.setGeometry(QtCore.QRect(580, 280, 131, 27))
        self.comboBox_gr_load_character.setObjectName("comboBox_gr_load_character")
        self.label_gr_prime_mover = QtWidgets.QLabel(self.tab_gears)
        self.label_gr_prime_mover.setGeometry(QtCore.QRect(470, 250, 101, 31))
        self.label_gr_prime_mover.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_gr_prime_mover.setObjectName("label_gr_prime_mover")
        self.lineEdit_gr_c_gv = QtWidgets.QLineEdit(self.tab_gears)
        self.lineEdit_gr_c_gv.setGeometry(QtCore.QRect(580, 310, 113, 27))
        self.lineEdit_gr_c_gv.setReadOnly(True)
        self.lineEdit_gr_c_gv.setObjectName("lineEdit_gr_c_gv")
        self.label_gr_c_gv = QtWidgets.QLabel(self.tab_gears)
        self.label_gr_c_gv.setGeometry(QtCore.QRect(360, 310, 211, 21))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_gr_c_gv.setFont(font)
        self.label_gr_c_gv.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_gr_c_gv.setWordWrap(True)
        self.label_gr_c_gv.setObjectName("label_gr_c_gv")
        self.lineEdit_gr_warning_box = QtWidgets.QLineEdit(self.tab_gears)
        self.lineEdit_gr_warning_box.setGeometry(QtCore.QRect(10, 640, 330, 70))
        self.lineEdit_gr_warning_box.setReadOnly(True)
        self.lineEdit_gr_warning_box.setObjectName("lineEdit_gr_warning_box")
        self.addButton_c_me_gears = QtWidgets.QPushButton(self.tab_gears)
        self.addButton_c_me_gears.setGeometry(QtCore.QRect(925, 650, 125, 50))
        self.addButton_c_me_gears.setObjectName("addButton_c_me_gears")
        self.tabWidget_mechanical_equipments.addTab(self.tab_gears, "")
        self.tab_bearings = QtWidgets.QWidget()
        self.tab_bearings.setObjectName("tab_bearings")
        self.line_9 = QtWidgets.QFrame(self.tab_bearings)
        self.line_9.setGeometry(QtCore.QRect(10, 340, 321, 16))
        self.line_9.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_9.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_9.setObjectName("line_9")
        self.line_10 = QtWidgets.QFrame(self.tab_bearings)
        self.line_10.setGeometry(QtCore.QRect(10, 450, 321, 16))
        self.line_10.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_10.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_10.setObjectName("line_10")
        self.label_be_c_r = QtWidgets.QLabel(self.tab_bearings)
        self.label_be_c_r.setGeometry(QtCore.QRect(10, 400, 191, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_be_c_r.setFont(font)
        self.label_be_c_r.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_be_c_r.setWordWrap(True)
        self.label_be_c_r.setObjectName("label_be_c_r")
        self.label_be_r = QtWidgets.QLabel(self.tab_bearings)
        self.label_be_r.setGeometry(QtCore.QRect(20, 370, 181, 31))
        self.label_be_r.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_be_r.setObjectName("label_be_r")
        self.lineEdit_be_r = QtWidgets.QLineEdit(self.tab_bearings)
        self.lineEdit_be_r.setGeometry(QtCore.QRect(211, 370, 113, 27))
        self.lineEdit_be_r.setObjectName("lineEdit_be_r")
        self.lineEdit_be_c_r = QtWidgets.QLineEdit(self.tab_bearings)
        self.lineEdit_be_c_r.setGeometry(QtCore.QRect(210, 410, 113, 27))
        self.lineEdit_be_c_r.setReadOnly(True)
        self.lineEdit_be_c_r.setObjectName("lineEdit_be_c_r")
        self.label_be_c_r_2 = QtWidgets.QLabel(self.tab_bearings)
        self.label_be_c_r_2.setGeometry(QtCore.QRect(10, 550, 191, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_be_c_r_2.setFont(font)
        self.label_be_c_r_2.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_be_c_r_2.setWordWrap(True)
        self.label_be_c_r_2.setObjectName("label_be_c_r_2")
        self.lineEdit_be_c_v = QtWidgets.QLineEdit(self.tab_bearings)
        self.lineEdit_be_c_v.setGeometry(QtCore.QRect(210, 560, 113, 27))
        self.lineEdit_be_c_v.setReadOnly(True)
        self.lineEdit_be_c_v.setObjectName("lineEdit_be_c_v")
        self.label_be_v0 = QtWidgets.QLabel(self.tab_bearings)
        self.label_be_v0.setGeometry(QtCore.QRect(-10, 470, 201, 41))
        self.label_be_v0.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_be_v0.setWordWrap(True)
        self.label_be_v0.setObjectName("label_be_v0")
        self.label_be_vl = QtWidgets.QLabel(self.tab_bearings)
        self.label_be_vl.setGeometry(QtCore.QRect(20, 510, 171, 41))
        self.label_be_vl.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_be_vl.setWordWrap(True)
        self.label_be_vl.setObjectName("label_be_vl")
        self.lineEdit_be_vl = QtWidgets.QLineEdit(self.tab_bearings)
        self.lineEdit_be_vl.setGeometry(QtCore.QRect(210, 520, 113, 27))
        self.lineEdit_be_vl.setObjectName("lineEdit_be_vl")
        self.lineEdit_be_v0 = QtWidgets.QLineEdit(self.tab_bearings)
        self.lineEdit_be_v0.setGeometry(QtCore.QRect(210, 480, 113, 27))
        self.lineEdit_be_v0.setObjectName("lineEdit_be_v0")
        self.line_11 = QtWidgets.QFrame(self.tab_bearings)
        self.line_11.setGeometry(QtCore.QRect(10, 600, 321, 16))
        self.line_11.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_11.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_11.setObjectName("line_11")
        self.label_be_c_cw = QtWidgets.QLabel(self.tab_bearings)
        self.label_be_c_cw.setGeometry(QtCore.QRect(360, 60, 191, 51))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_be_c_cw.setFont(font)
        self.label_be_c_cw.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_be_c_cw.setWordWrap(True)
        self.label_be_c_cw.setObjectName("label_be_c_cw")
        self.lineEdit_be_c_cw = QtWidgets.QLineEdit(self.tab_bearings)
        self.lineEdit_be_c_cw.setGeometry(QtCore.QRect(560, 90, 113, 27))
        self.lineEdit_be_c_cw.setReadOnly(True)
        self.lineEdit_be_c_cw.setObjectName("lineEdit_be_c_cw")
        self.lineEdit_be_cw = QtWidgets.QLineEdit(self.tab_bearings)
        self.lineEdit_be_cw.setGeometry(QtCore.QRect(560, 40, 113, 27))
        self.lineEdit_be_cw.setObjectName("lineEdit_be_cw")
        self.label_be_cw = QtWidgets.QLabel(self.tab_bearings)
        self.label_be_cw.setGeometry(QtCore.QRect(340, 20, 201, 41))
        self.label_be_cw.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_be_cw.setWordWrap(True)
        self.label_be_cw.setObjectName("label_be_cw")
        self.line_23 = QtWidgets.QFrame(self.tab_bearings)
        self.line_23.setGeometry(QtCore.QRect(360, 130, 321, 16))
        self.line_23.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_23.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_23.setObjectName("line_23")
        self.line_24 = QtWidgets.QFrame(self.tab_bearings)
        self.line_24.setGeometry(QtCore.QRect(330, 10, 20, 581))
        self.line_24.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_24.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_24.setObjectName("line_24")
        self.label_be_c_t = QtWidgets.QLabel(self.tab_bearings)
        self.label_be_c_t.setGeometry(QtCore.QRect(370, 210, 191, 51))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_be_c_t.setFont(font)
        self.label_be_c_t.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_be_c_t.setWordWrap(True)
        self.label_be_c_t.setObjectName("label_be_c_t")
        self.lineEdit_be_c_t = QtWidgets.QLineEdit(self.tab_bearings)
        self.lineEdit_be_c_t.setGeometry(QtCore.QRect(570, 240, 113, 27))
        self.lineEdit_be_c_t.setReadOnly(True)
        self.lineEdit_be_c_t.setObjectName("lineEdit_be_c_t")
        self.lineEdit_be_t0 = QtWidgets.QLineEdit(self.tab_bearings)
        self.lineEdit_be_t0.setGeometry(QtCore.QRect(570, 170, 113, 27))
        self.lineEdit_be_t0.setText("")
        self.lineEdit_be_t0.setObjectName("lineEdit_be_t0")
        self.label_be_t0 = QtWidgets.QLabel(self.tab_bearings)
        self.label_be_t0.setGeometry(QtCore.QRect(350, 160, 201, 41))
        self.label_be_t0.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_be_t0.setWordWrap(True)
        self.label_be_t0.setObjectName("label_be_t0")
        self.line_25 = QtWidgets.QFrame(self.tab_bearings)
        self.line_25.setGeometry(QtCore.QRect(370, 280, 321, 16))
        self.line_25.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_25.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_25.setObjectName("line_25")
        self.label_be_type_of_application = QtWidgets.QLabel(self.tab_bearings)
        self.label_be_type_of_application.setGeometry(QtCore.QRect(380, 300, 131, 31))
        self.label_be_type_of_application.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_be_type_of_application.setObjectName("label_be_type_of_application")
        self.comboBox_be_type_of_application = QtWidgets.QComboBox(self.tab_bearings)
        self.comboBox_be_type_of_application.setGeometry(QtCore.QRect(520, 300, 161, 27))
        self.comboBox_be_type_of_application.setObjectName("comboBox_be_type_of_application")
        self.radioButton_be_ball_bearing = QtWidgets.QRadioButton(self.tab_bearings)
        self.radioButton_be_ball_bearing.setGeometry(QtCore.QRect(80, 40, 131, 22))
        self.radioButton_be_ball_bearing.setObjectName("radioButton_be_ball_bearing")
        self.radioButton_be_roller_bearing = QtWidgets.QRadioButton(self.tab_bearings)
        self.radioButton_be_roller_bearing.setGeometry(QtCore.QRect(80, 60, 131, 22))
        self.radioButton_be_roller_bearing.setObjectName("radioButton_be_roller_bearing")
        self.lineEdit_be_c_sf = QtWidgets.QLineEdit(self.tab_bearings)
        self.lineEdit_be_c_sf.setGeometry(QtCore.QRect(550, 350, 113, 27))
        self.lineEdit_be_c_sf.setReadOnly(True)
        self.lineEdit_be_c_sf.setObjectName("lineEdit_be_c_sf")
        self.label_be_c_sf = QtWidgets.QLabel(self.tab_bearings)
        self.label_be_c_sf.setGeometry(QtCore.QRect(380, 330, 161, 41))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_be_c_sf.setFont(font)
        self.label_be_c_sf.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_be_c_sf.setWordWrap(True)
        self.label_be_c_sf.setObjectName("label_be_c_sf")
        self.line_26 = QtWidgets.QFrame(self.tab_bearings)
        self.line_26.setGeometry(QtCore.QRect(370, 390, 321, 16))
        self.line_26.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_26.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_26.setObjectName("line_26")
        self.label_be_type_of_application_2 = QtWidgets.QLabel(self.tab_bearings)
        self.label_be_type_of_application_2.setGeometry(QtCore.QRect(380, 410, 131, 31))
        self.label_be_type_of_application_2.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_be_type_of_application_2.setObjectName("label_be_type_of_application_2")
        self.comboBox_be_type_of_application_2 = QtWidgets.QComboBox(self.tab_bearings)
        self.comboBox_be_type_of_application_2.setGeometry(QtCore.QRect(520, 410, 161, 27))
        self.comboBox_be_type_of_application_2.setObjectName("comboBox_be_type_of_application_2")
        self.lineEdit_be_bearing_diameter = QtWidgets.QLineEdit(self.tab_bearings)
        self.lineEdit_be_bearing_diameter.setGeometry(QtCore.QRect(560, 450, 113, 27))
        self.lineEdit_be_bearing_diameter.setText("")
        self.lineEdit_be_bearing_diameter.setObjectName("lineEdit_be_bearing_diameter")
        self.label_be_bearing_diameter = QtWidgets.QLabel(self.tab_bearings)
        self.label_be_bearing_diameter.setGeometry(QtCore.QRect(350, 440, 201, 41))
        self.label_be_bearing_diameter.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_be_bearing_diameter.setWordWrap(True)
        self.label_be_bearing_diameter.setObjectName("label_be_bearing_diameter")
        self.label_be_c_c = QtWidgets.QLabel(self.tab_bearings)
        self.label_be_c_c.setGeometry(QtCore.QRect(370, 480, 181, 41))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_be_c_c.setFont(font)
        self.label_be_c_c.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_be_c_c.setWordWrap(True)
        self.label_be_c_c.setObjectName("label_be_c_c")
        self.lineEdit_be_c_c = QtWidgets.QLineEdit(self.tab_bearings)
        self.lineEdit_be_c_c.setGeometry(QtCore.QRect(560, 500, 113, 27))
        self.lineEdit_be_c_c.setReadOnly(True)
        self.lineEdit_be_c_c.setObjectName("lineEdit_be_c_c")
        self.lineEdit_be_c_y = QtWidgets.QLineEdit(self.tab_bearings)
        self.lineEdit_be_c_y.setGeometry(QtCore.QRect(220, 300, 113, 27))
        self.lineEdit_be_c_y.setReadOnly(True)
        self.lineEdit_be_c_y.setObjectName("lineEdit_be_c_y")
        self.label_be_c_y = QtWidgets.QLabel(self.tab_bearings)
        self.label_be_c_y.setGeometry(QtCore.QRect(10, 290, 191, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_be_c_y.setFont(font)
        self.label_be_c_y.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_be_c_y.setWordWrap(True)
        self.label_be_c_y.setObjectName("label_be_c_y")
        self.label_be_l_s = QtWidgets.QLabel(self.tab_bearings)
        self.label_be_l_s.setGeometry(QtCore.QRect(29, 160, 181, 31))
        self.label_be_l_s.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_be_l_s.setObjectName("label_be_l_s")
        self.label_be_l_a = QtWidgets.QLabel(self.tab_bearings)
        self.label_be_l_a.setGeometry(QtCore.QRect(20, 190, 191, 31))
        self.label_be_l_a.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_be_l_a.setObjectName("label_be_l_a")
        self.label_be_n = QtWidgets.QLabel(self.tab_bearings)
        self.label_be_n.setGeometry(QtCore.QRect(20, 220, 191, 31))
        self.label_be_n.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_be_n.setObjectName("label_be_n")
        self.lineEdit_be_l_a = QtWidgets.QLineEdit(self.tab_bearings)
        self.lineEdit_be_l_a.setGeometry(QtCore.QRect(220, 190, 113, 27))
        self.lineEdit_be_l_a.setObjectName("lineEdit_be_l_a")
        self.lineEdit_be_n = QtWidgets.QLineEdit(self.tab_bearings)
        self.lineEdit_be_n.setGeometry(QtCore.QRect(220, 220, 113, 27))
        self.lineEdit_be_n.setObjectName("lineEdit_be_n")
        self.lineEdit_be_l_s = QtWidgets.QLineEdit(self.tab_bearings)
        self.lineEdit_be_l_s.setGeometry(QtCore.QRect(220, 160, 113, 27))
        self.lineEdit_be_l_s.setObjectName("lineEdit_be_l_s")
        self.label_be_lambda_be_b = QtWidgets.QLabel(self.tab_bearings)
        self.label_be_lambda_be_b.setGeometry(QtCore.QRect(10, 250, 191, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_be_lambda_be_b.setFont(font)
        self.label_be_lambda_be_b.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_be_lambda_be_b.setWordWrap(True)
        self.label_be_lambda_be_b.setObjectName("label_be_lambda_be_b")
        self.lineEdit_be_lambda_be_b = QtWidgets.QLineEdit(self.tab_bearings)
        self.lineEdit_be_lambda_be_b.setGeometry(QtCore.QRect(220, 260, 113, 27))
        self.lineEdit_be_lambda_be_b.setReadOnly(True)
        self.lineEdit_be_lambda_be_b.setObjectName("lineEdit_be_lambda_be_b")
        self.line_27 = QtWidgets.QFrame(self.tab_bearings)
        self.line_27.setGeometry(QtCore.QRect(360, 530, 321, 16))
        self.line_27.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_27.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_27.setObjectName("line_27")
        self.groupBox_be_lambda_be = QtWidgets.QGroupBox(self.tab_bearings)
        self.groupBox_be_lambda_be.setGeometry(QtCore.QRect(450, 640, 200, 63))
        font = QtGui.QFont()
        font.setUnderline(False)
        font.setStrikeOut(False)
        self.groupBox_be_lambda_be.setFont(font)
        self.groupBox_be_lambda_be.setObjectName("groupBox_be_lambda_be")
        self.lineEdit_be_lambda_be = QtWidgets.QLineEdit(self.groupBox_be_lambda_be)
        self.lineEdit_be_lambda_be.setGeometry(QtCore.QRect(0, 30, 200, 27))
        self.lineEdit_be_lambda_be.setReadOnly(True)
        self.lineEdit_be_lambda_be.setObjectName("lineEdit_be_lambda_be")
        self.label_2 = QtWidgets.QLabel(self.tab_bearings)
        self.label_2.setGeometry(QtCore.QRect(30, 20, 191, 21))
        font = QtGui.QFont()
        font.setItalic(True)
        self.label_2.setFont(font)
        self.label_2.setObjectName("label_2")
        self.line_31 = QtWidgets.QFrame(self.tab_bearings)
        self.line_31.setGeometry(QtCore.QRect(10, 130, 321, 16))
        self.line_31.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_31.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_31.setObjectName("line_31")
        self.lineEdit_be_warning_box = QtWidgets.QLineEdit(self.tab_bearings)
        self.lineEdit_be_warning_box.setGeometry(QtCore.QRect(10, 640, 330, 70))
        self.lineEdit_be_warning_box.setReadOnly(True)
        self.lineEdit_be_warning_box.setObjectName("lineEdit_be_warning_box")
        self.addButton_c_me_bearings = QtWidgets.QPushButton(self.tab_bearings)
        self.addButton_c_me_bearings.setGeometry(QtCore.QRect(925, 650, 125, 50))
        self.addButton_c_me_bearings.setObjectName("addButton_c_me_bearings")
        self.tabWidget_mechanical_equipments.addTab(self.tab_bearings, "")
        self.tab_actuators = QtWidgets.QWidget()
        self.tab_actuators.setObjectName("tab_actuators")
        self.line_19 = QtWidgets.QFrame(self.tab_actuators)
        self.line_19.setGeometry(QtCore.QRect(410, 10, 3, 525))
        self.line_19.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_19.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_19.setObjectName("line_19")
        self.line_20 = QtWidgets.QFrame(self.tab_actuators)
        self.line_20.setGeometry(QtCore.QRect(420, 220, 660, 3))
        self.line_20.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_20.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_20.setObjectName("line_20")
        self.groupBox_ac_lambda_ac = QtWidgets.QGroupBox(self.tab_actuators)
        self.groupBox_ac_lambda_ac.setGeometry(QtCore.QRect(450, 640, 200, 63))
        font = QtGui.QFont()
        font.setUnderline(False)
        font.setStrikeOut(False)
        self.groupBox_ac_lambda_ac.setFont(font)
        self.groupBox_ac_lambda_ac.setObjectName("groupBox_ac_lambda_ac")
        self.lineEdit_ac_lambda_ac = QtWidgets.QLineEdit(self.groupBox_ac_lambda_ac)
        self.lineEdit_ac_lambda_ac.setGeometry(QtCore.QRect(0, 30, 200, 27))
        self.lineEdit_ac_lambda_ac.setReadOnly(True)
        self.lineEdit_ac_lambda_ac.setObjectName("lineEdit_ac_lambda_ac")
        self.lineEdit_ac_warning_box = QtWidgets.QLineEdit(self.tab_actuators)
        self.lineEdit_ac_warning_box.setGeometry(QtCore.QRect(10, 640, 330, 70))
        self.lineEdit_ac_warning_box.setReadOnly(True)
        self.lineEdit_ac_warning_box.setObjectName("lineEdit_ac_warning_box")
        self.groupBox_ac_lambda_ac_b = QtWidgets.QGroupBox(self.tab_actuators)
        self.groupBox_ac_lambda_ac_b.setGeometry(QtCore.QRect(0, 20, 401, 521))
        self.groupBox_ac_lambda_ac_b.setTitle("")
        self.groupBox_ac_lambda_ac_b.setObjectName("groupBox_ac_lambda_ac_b")
        self.label_ac_w_a = QtWidgets.QLabel(self.groupBox_ac_lambda_ac_b)
        self.label_ac_w_a.setGeometry(QtCore.QRect(80, 130, 141, 41))
        self.label_ac_w_a.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_ac_w_a.setWordWrap(True)
        self.label_ac_w_a.setObjectName("label_ac_w_a")
        self.lineEdit_ac_n_o = QtWidgets.QLineEdit(self.groupBox_ac_lambda_ac_b)
        self.lineEdit_ac_n_o.setGeometry(QtCore.QRect(230, 430, 171, 27))
        self.lineEdit_ac_n_o.setReadOnly(True)
        self.lineEdit_ac_n_o.setObjectName("lineEdit_ac_n_o")
        self.lineEdit_ac_mu_2 = QtWidgets.QLineEdit(self.groupBox_ac_lambda_ac_b)
        self.lineEdit_ac_mu_2.setGeometry(QtCore.QRect(231, 300, 101, 27))
        self.lineEdit_ac_mu_2.setObjectName("lineEdit_ac_mu_2")
        self.lineEdit_ac_d_1 = QtWidgets.QLineEdit(self.groupBox_ac_lambda_ac_b)
        self.lineEdit_ac_d_1.setGeometry(QtCore.QRect(231, 180, 101, 27))
        self.lineEdit_ac_d_1.setObjectName("lineEdit_ac_d_1")
        self.label_ac_n_o = QtWidgets.QLabel(self.groupBox_ac_lambda_ac_b)
        self.label_ac_n_o.setGeometry(QtCore.QRect(30, 430, 191, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_ac_n_o.setFont(font)
        self.label_ac_n_o.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_ac_n_o.setWordWrap(True)
        self.label_ac_n_o.setObjectName("label_ac_n_o")
        self.lineEdit_ac_w_a = QtWidgets.QLineEdit(self.groupBox_ac_lambda_ac_b)
        self.lineEdit_ac_w_a.setGeometry(QtCore.QRect(231, 140, 101, 27))
        self.lineEdit_ac_w_a.setObjectName("lineEdit_ac_w_a")
        self.lineEdit_ac_e_1 = QtWidgets.QLineEdit(self.groupBox_ac_lambda_ac_b)
        self.lineEdit_ac_e_1.setGeometry(QtCore.QRect(230, 350, 101, 27))
        self.lineEdit_ac_e_1.setObjectName("lineEdit_ac_e_1")
        self.label_ac_e_1 = QtWidgets.QLabel(self.groupBox_ac_lambda_ac_b)
        self.label_ac_e_1.setGeometry(QtCore.QRect(9, 330, 211, 51))
        self.label_ac_e_1.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_ac_e_1.setWordWrap(True)
        self.label_ac_e_1.setObjectName("label_ac_e_1")
        self.label_ac_mu_2 = QtWidgets.QLabel(self.groupBox_ac_lambda_ac_b)
        self.label_ac_mu_2.setGeometry(QtCore.QRect(20, 290, 201, 41))
        self.label_ac_mu_2.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_ac_mu_2.setWordWrap(True)
        self.label_ac_mu_2.setObjectName("label_ac_mu_2")
        self.radioButton_ac_gama_0_54 = QtWidgets.QRadioButton(self.groupBox_ac_lambda_ac_b)
        self.radioButton_ac_gama_0_54.setGeometry(QtCore.QRect(160, 20, 175, 22))
        self.radioButton_ac_gama_0_54.setObjectName("radioButton_ac_gama_0_54")
        self.label_ac_e_2 = QtWidgets.QLabel(self.groupBox_ac_lambda_ac_b)
        self.label_ac_e_2.setGeometry(QtCore.QRect(19, 370, 201, 51))
        self.label_ac_e_2.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_ac_e_2.setWordWrap(True)
        self.label_ac_e_2.setObjectName("label_ac_e_2")
        self.lineEdit_ac_e_2 = QtWidgets.QLineEdit(self.groupBox_ac_lambda_ac_b)
        self.lineEdit_ac_e_2.setGeometry(QtCore.QRect(230, 390, 101, 27))
        self.lineEdit_ac_e_2.setObjectName("lineEdit_ac_e_2")
        self.lineEdit_ac_f_y = QtWidgets.QLineEdit(self.groupBox_ac_lambda_ac_b)
        self.lineEdit_ac_f_y.setGeometry(QtCore.QRect(231, 100, 171, 27))
        self.lineEdit_ac_f_y.setReadOnly(True)
        self.lineEdit_ac_f_y.setObjectName("lineEdit_ac_f_y")
        self.label_ac_various_metals = QtWidgets.QLabel(self.groupBox_ac_lambda_ac_b)
        self.label_ac_various_metals.setGeometry(QtCore.QRect(30, 60, 191, 31))
        self.label_ac_various_metals.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_ac_various_metals.setWordWrap(True)
        self.label_ac_various_metals.setObjectName("label_ac_various_metals")
        self.label_ac_lambda_ac_b = QtWidgets.QLabel(self.groupBox_ac_lambda_ac_b)
        self.label_ac_lambda_ac_b.setGeometry(QtCore.QRect(30, 460, 191, 41))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_ac_lambda_ac_b.setFont(font)
        self.label_ac_lambda_ac_b.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_ac_lambda_ac_b.setWordWrap(True)
        self.label_ac_lambda_ac_b.setObjectName("label_ac_lambda_ac_b")
        self.comboBox_ac_various_metals = QtWidgets.QComboBox(self.groupBox_ac_lambda_ac_b)
        self.comboBox_ac_various_metals.setGeometry(QtCore.QRect(230, 60, 171, 27))
        self.comboBox_ac_various_metals.setObjectName("comboBox_ac_various_metals")
        self.label_ac_d_2 = QtWidgets.QLabel(self.groupBox_ac_lambda_ac_b)
        self.label_ac_d_2.setGeometry(QtCore.QRect(20, 210, 201, 41))
        self.label_ac_d_2.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_ac_d_2.setWordWrap(True)
        self.label_ac_d_2.setObjectName("label_ac_d_2")
        self.lineEdit_ac_d_2 = QtWidgets.QLineEdit(self.groupBox_ac_lambda_ac_b)
        self.lineEdit_ac_d_2.setGeometry(QtCore.QRect(231, 220, 101, 27))
        self.lineEdit_ac_d_2.setObjectName("lineEdit_ac_d_2")
        self.radioButton_ac_gama_0_2 = QtWidgets.QRadioButton(self.groupBox_ac_lambda_ac_b)
        self.radioButton_ac_gama_0_2.setGeometry(QtCore.QRect(160, 0, 175, 21))
        self.radioButton_ac_gama_0_2.setObjectName("radioButton_ac_gama_0_2")
        self.label_ac_f_y = QtWidgets.QLabel(self.groupBox_ac_lambda_ac_b)
        self.label_ac_f_y.setGeometry(QtCore.QRect(80, 90, 141, 41))
        self.label_ac_f_y.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_ac_f_y.setWordWrap(True)
        self.label_ac_f_y.setObjectName("label_ac_f_y")
        self.label_ac_d_1 = QtWidgets.QLabel(self.groupBox_ac_lambda_ac_b)
        self.label_ac_d_1.setGeometry(QtCore.QRect(10, 170, 211, 41))
        self.label_ac_d_1.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_ac_d_1.setWordWrap(True)
        self.label_ac_d_1.setObjectName("label_ac_d_1")
        self.lineEdit_ac_mu_1 = QtWidgets.QLineEdit(self.groupBox_ac_lambda_ac_b)
        self.lineEdit_ac_mu_1.setGeometry(QtCore.QRect(231, 260, 101, 27))
        self.lineEdit_ac_mu_1.setObjectName("lineEdit_ac_mu_1")
        self.label_ac_mu_1 = QtWidgets.QLabel(self.groupBox_ac_lambda_ac_b)
        self.label_ac_mu_1.setGeometry(QtCore.QRect(20, 250, 201, 41))
        self.label_ac_mu_1.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_ac_mu_1.setWordWrap(True)
        self.label_ac_mu_1.setObjectName("label_ac_mu_1")
        self.lineEdit_ac_lambda_ac_b = QtWidgets.QLineEdit(self.groupBox_ac_lambda_ac_b)
        self.lineEdit_ac_lambda_ac_b.setGeometry(QtCore.QRect(230, 470, 171, 27))
        self.lineEdit_ac_lambda_ac_b.setReadOnly(True)
        self.lineEdit_ac_lambda_ac_b.setObjectName("lineEdit_ac_lambda_ac_b")
        self.groupBox_ac_c_cp = QtWidgets.QGroupBox(self.tab_actuators)
        self.groupBox_ac_c_cp.setGeometry(QtCore.QRect(420, 0, 651, 191))
        self.groupBox_ac_c_cp.setTitle("")
        self.groupBox_ac_c_cp.setObjectName("groupBox_ac_c_cp")
        self.label_ac_h_p = QtWidgets.QLabel(self.groupBox_ac_c_cp)
        self.label_ac_h_p.setGeometry(QtCore.QRect(340, 50, 181, 21))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_ac_h_p.setFont(font)
        self.label_ac_h_p.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_ac_h_p.setWordWrap(True)
        self.label_ac_h_p.setObjectName("label_ac_h_p")
        self.lineEdit_ac_filter_size = QtWidgets.QLineEdit(self.groupBox_ac_c_cp)
        self.lineEdit_ac_filter_size.setGeometry(QtCore.QRect(151, 100, 101, 27))
        self.lineEdit_ac_filter_size.setText("")
        self.lineEdit_ac_filter_size.setObjectName("lineEdit_ac_filter_size")
        self.lineEdit_ac_c_cp = QtWidgets.QLineEdit(self.groupBox_ac_c_cp)
        self.lineEdit_ac_c_cp.setGeometry(QtCore.QRect(310, 160, 171, 27))
        self.lineEdit_ac_c_cp.setReadOnly(True)
        self.lineEdit_ac_c_cp.setObjectName("lineEdit_ac_c_cp")
        self.label_ac_material = QtWidgets.QLabel(self.groupBox_ac_c_cp)
        self.label_ac_material.setGeometry(QtCore.QRect(110, 20, 61, 31))
        self.label_ac_material.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_ac_material.setWordWrap(True)
        self.label_ac_material.setObjectName("label_ac_material")
        self.lineEdit_ac_c_s = QtWidgets.QLineEdit(self.groupBox_ac_c_cp)
        self.lineEdit_ac_c_s.setGeometry(QtCore.QRect(510, 100, 101, 27))
        self.lineEdit_ac_c_s.setText("")
        self.lineEdit_ac_c_s.setReadOnly(True)
        self.lineEdit_ac_c_s.setObjectName("lineEdit_ac_c_s")
        self.lineEdit_ac_h_p = QtWidgets.QLineEdit(self.groupBox_ac_c_cp)
        self.lineEdit_ac_h_p.setGeometry(QtCore.QRect(530, 50, 113, 27))
        self.lineEdit_ac_h_p.setText("")
        self.lineEdit_ac_h_p.setReadOnly(True)
        self.lineEdit_ac_h_p.setObjectName("lineEdit_ac_h_p")
        self.radioButton_ac_piston = QtWidgets.QRadioButton(self.groupBox_ac_c_cp)
        self.radioButton_ac_piston.setGeometry(QtCore.QRect(0, 40, 91, 22))
        self.radioButton_ac_piston.setObjectName("radioButton_ac_piston")
        self.label_ac_h_c = QtWidgets.QLabel(self.groupBox_ac_c_cp)
        self.label_ac_h_c.setGeometry(QtCore.QRect(340, 20, 181, 21))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_ac_h_c.setFont(font)
        self.label_ac_h_c.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_ac_h_c.setWordWrap(True)
        self.label_ac_h_c.setObjectName("label_ac_h_c")
        self.comboBox_material_types = QtWidgets.QComboBox(self.groupBox_ac_c_cp)
        self.comboBox_material_types.setGeometry(QtCore.QRect(180, 50, 171, 27))
        self.comboBox_material_types.setObjectName("comboBox_material_types")
        self.lineEdit_ac_h_c = QtWidgets.QLineEdit(self.groupBox_ac_c_cp)
        self.lineEdit_ac_h_c.setGeometry(QtCore.QRect(530, 20, 113, 27))
        self.lineEdit_ac_h_c.setText("")
        self.lineEdit_ac_h_c.setReadOnly(True)
        self.lineEdit_ac_h_c.setObjectName("lineEdit_ac_h_c")
        self.label_ac_c_s = QtWidgets.QLabel(self.groupBox_ac_c_cp)
        self.label_ac_c_s.setGeometry(QtCore.QRect(260, 90, 241, 51))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_ac_c_s.setFont(font)
        self.label_ac_c_s.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_ac_c_s.setWordWrap(True)
        self.label_ac_c_s.setObjectName("label_ac_c_s")
        self.label_ac_c_cp = QtWidgets.QLabel(self.groupBox_ac_c_cp)
        self.label_ac_c_cp.setGeometry(QtCore.QRect(110, 150, 191, 41))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_ac_c_cp.setFont(font)
        self.label_ac_c_cp.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_ac_c_cp.setWordWrap(True)
        self.label_ac_c_cp.setObjectName("label_ac_c_cp")
        self.comboBox_ac_materials = QtWidgets.QComboBox(self.groupBox_ac_c_cp)
        self.comboBox_ac_materials.setGeometry(QtCore.QRect(180, 20, 171, 27))
        self.comboBox_ac_materials.setObjectName("comboBox_ac_materials")
        self.radioButton_ac_cylinder = QtWidgets.QRadioButton(self.groupBox_ac_c_cp)
        self.radioButton_ac_cylinder.setGeometry(QtCore.QRect(0, 20, 91, 22))
        self.radioButton_ac_cylinder.setObjectName("radioButton_ac_cylinder")
        self.label_ac_filter_size = QtWidgets.QLabel(self.groupBox_ac_c_cp)
        self.label_ac_filter_size.setGeometry(QtCore.QRect(0, 90, 141, 41))
        self.label_ac_filter_size.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_ac_filter_size.setWordWrap(True)
        self.label_ac_filter_size.setObjectName("label_ac_filter_size")
        self.groupBox_ac_c_t = QtWidgets.QGroupBox(self.tab_actuators)
        self.groupBox_ac_c_t.setGeometry(QtCore.QRect(420, 240, 651, 91))
        self.groupBox_ac_c_t.setTitle("")
        self.groupBox_ac_c_t.setObjectName("groupBox_ac_c_t")
        self.lineEdit_ac_teta = QtWidgets.QLineEdit(self.groupBox_ac_c_t)
        self.lineEdit_ac_teta.setGeometry(QtCore.QRect(181, 50, 101, 27))
        self.lineEdit_ac_teta.setObjectName("lineEdit_ac_teta")
        self.label_ac_c_t = QtWidgets.QLabel(self.groupBox_ac_c_t)
        self.label_ac_c_t.setGeometry(QtCore.QRect(300, 20, 191, 41))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_ac_c_t.setFont(font)
        self.label_ac_c_t.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_ac_c_t.setWordWrap(True)
        self.label_ac_c_t.setObjectName("label_ac_c_t")
        self.label_ac_t = QtWidgets.QLabel(self.groupBox_ac_c_t)
        self.label_ac_t.setGeometry(QtCore.QRect(-1, 0, 171, 41))
        self.label_ac_t.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_ac_t.setWordWrap(True)
        self.label_ac_t.setObjectName("label_ac_t")
        self.lineEdit_ac_t = QtWidgets.QLineEdit(self.groupBox_ac_c_t)
        self.lineEdit_ac_t.setGeometry(QtCore.QRect(180, 10, 101, 27))
        self.lineEdit_ac_t.setObjectName("lineEdit_ac_t")
        self.label_ac_teta = QtWidgets.QLabel(self.groupBox_ac_c_t)
        self.label_ac_teta.setGeometry(QtCore.QRect(0, 40, 171, 41))
        self.label_ac_teta.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_ac_teta.setWordWrap(True)
        self.label_ac_teta.setObjectName("label_ac_teta")
        self.lineEdit_ac_c_t = QtWidgets.QLineEdit(self.groupBox_ac_c_t)
        self.lineEdit_ac_c_t.setGeometry(QtCore.QRect(500, 30, 150, 27))
        self.lineEdit_ac_c_t.setReadOnly(True)
        self.lineEdit_ac_c_t.setObjectName("lineEdit_ac_c_t")
        self.addButton_c_me_actuators = QtWidgets.QPushButton(self.tab_actuators)
        self.addButton_c_me_actuators.setGeometry(QtCore.QRect(925, 650, 125, 50))
        self.addButton_c_me_actuators.setObjectName("addButton_c_me_actuators")
        self.tabWidget_mechanical_equipments.addTab(self.tab_actuators, "")
        self.tab_shafts = QtWidgets.QWidget()
        self.tab_shafts.setObjectName("tab_shafts")
        self.label_sh_lambda_sh_b = QtWidgets.QLabel(self.tab_shafts)
        self.label_sh_lambda_sh_b.setGeometry(QtCore.QRect(10, 50, 191, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_sh_lambda_sh_b.setFont(font)
        self.label_sh_lambda_sh_b.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_sh_lambda_sh_b.setWordWrap(True)
        self.label_sh_lambda_sh_b.setObjectName("label_sh_lambda_sh_b")
        self.label_sh_n = QtWidgets.QLabel(self.tab_shafts)
        self.label_sh_n.setGeometry(QtCore.QRect(20, 20, 191, 31))
        self.label_sh_n.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_sh_n.setObjectName("label_sh_n")
        self.lineEdit_sh_lambda_sh_b = QtWidgets.QLineEdit(self.tab_shafts)
        self.lineEdit_sh_lambda_sh_b.setGeometry(QtCore.QRect(220, 60, 113, 27))
        self.lineEdit_sh_lambda_sh_b.setReadOnly(True)
        self.lineEdit_sh_lambda_sh_b.setObjectName("lineEdit_sh_lambda_sh_b")
        self.lineEdit_sh_n = QtWidgets.QLineEdit(self.tab_shafts)
        self.lineEdit_sh_n.setGeometry(QtCore.QRect(220, 20, 113, 27))
        self.lineEdit_sh_n.setObjectName("lineEdit_sh_n")
        self.line_28 = QtWidgets.QFrame(self.tab_shafts)
        self.line_28.setGeometry(QtCore.QRect(10, 100, 411, 16))
        self.line_28.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_28.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_28.setObjectName("line_28")
        self.label_sh_surface_finish = QtWidgets.QLabel(self.tab_shafts)
        self.label_sh_surface_finish.setGeometry(QtCore.QRect(80, 120, 141, 31))
        self.label_sh_surface_finish.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_sh_surface_finish.setObjectName("label_sh_surface_finish")
        self.comboBox_sh_surface_finish = QtWidgets.QComboBox(self.tab_shafts)
        self.comboBox_sh_surface_finish.setGeometry(QtCore.QRect(230, 120, 111, 27))
        self.comboBox_sh_surface_finish.setObjectName("comboBox_sh_surface_finish")
        self.lineEdit_sh_c_f = QtWidgets.QLineEdit(self.tab_shafts)
        self.lineEdit_sh_c_f.setGeometry(QtCore.QRect(230, 190, 113, 27))
        self.lineEdit_sh_c_f.setReadOnly(True)
        self.lineEdit_sh_c_f.setObjectName("lineEdit_sh_c_f")
        self.label_sh_c_f = QtWidgets.QLabel(self.tab_shafts)
        self.label_sh_c_f.setGeometry(QtCore.QRect(40, 180, 181, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_sh_c_f.setFont(font)
        self.label_sh_c_f.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_sh_c_f.setWordWrap(True)
        self.label_sh_c_f.setObjectName("label_sh_c_f")
        self.label_sh_t_s = QtWidgets.QLabel(self.tab_shafts)
        self.label_sh_t_s.setGeometry(QtCore.QRect(0, 150, 221, 31))
        self.label_sh_t_s.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_sh_t_s.setObjectName("label_sh_t_s")
        self.lineEdit_sh_t_s = QtWidgets.QLineEdit(self.tab_shafts)
        self.lineEdit_sh_t_s.setGeometry(QtCore.QRect(230, 150, 113, 27))
        self.lineEdit_sh_t_s.setObjectName("lineEdit_sh_t_s")
        self.line_29 = QtWidgets.QFrame(self.tab_shafts)
        self.line_29.setGeometry(QtCore.QRect(10, 230, 421, 16))
        self.line_29.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_29.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_29.setObjectName("line_29")
        self.label_sh_t_at = QtWidgets.QLabel(self.tab_shafts)
        self.label_sh_t_at.setGeometry(QtCore.QRect(10, 250, 211, 31))
        self.label_sh_t_at.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_sh_t_at.setObjectName("label_sh_t_at")
        self.lineEdit_sh_t_at = QtWidgets.QLineEdit(self.tab_shafts)
        self.lineEdit_sh_t_at.setGeometry(QtCore.QRect(230, 250, 113, 27))
        self.lineEdit_sh_t_at.setObjectName("lineEdit_sh_t_at")
        self.label_sh_c_t = QtWidgets.QLabel(self.tab_shafts)
        self.label_sh_c_t.setGeometry(QtCore.QRect(40, 290, 181, 41))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_sh_c_t.setFont(font)
        self.label_sh_c_t.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_sh_c_t.setWordWrap(True)
        self.label_sh_c_t.setObjectName("label_sh_c_t")
        self.lineEdit_sh_c_t = QtWidgets.QLineEdit(self.tab_shafts)
        self.lineEdit_sh_c_t.setGeometry(QtCore.QRect(230, 300, 113, 27))
        self.lineEdit_sh_c_t.setReadOnly(True)
        self.lineEdit_sh_c_t.setObjectName("lineEdit_sh_c_t")
        self.line_30 = QtWidgets.QFrame(self.tab_shafts)
        self.line_30.setGeometry(QtCore.QRect(10, 340, 431, 16))
        self.line_30.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_30.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_30.setObjectName("line_30")
        self.label_sh_e = QtWidgets.QLabel(self.tab_shafts)
        self.label_sh_e.setGeometry(QtCore.QRect(470, 10, 261, 31))
        self.label_sh_e.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_sh_e.setWordWrap(True)
        self.label_sh_e.setObjectName("label_sh_e")
        self.label_sh_f = QtWidgets.QLabel(self.tab_shafts)
        self.label_sh_f.setGeometry(QtCore.QRect(470, 50, 261, 31))
        self.label_sh_f.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_sh_f.setWordWrap(True)
        self.label_sh_f.setObjectName("label_sh_f")
        self.lineEdit_sh_f = QtWidgets.QLineEdit(self.tab_shafts)
        self.lineEdit_sh_f.setGeometry(QtCore.QRect(740, 60, 113, 27))
        self.lineEdit_sh_f.setObjectName("lineEdit_sh_f")
        self.label_sh_b = QtWidgets.QLabel(self.tab_shafts)
        self.label_sh_b.setGeometry(QtCore.QRect(470, 90, 261, 31))
        self.label_sh_b.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_sh_b.setObjectName("label_sh_b")
        self.comboBox_sh_b = QtWidgets.QComboBox(self.tab_shafts)
        self.comboBox_sh_b.setGeometry(QtCore.QRect(740, 90, 111, 27))
        self.comboBox_sh_b.setObjectName("comboBox_sh_b")
        self.label = QtWidgets.QLabel(self.tab_shafts)
        self.label.setGeometry(QtCore.QRect(510, 130, 131, 21))
        font = QtGui.QFont()
        font.setItalic(True)
        self.label.setFont(font)
        self.label.setObjectName("label")
        self.lineEdit_sh_add_section = QtWidgets.QLineEdit(self.tab_shafts)
        self.lineEdit_sh_add_section.setGeometry(QtCore.QRect(510, 150, 113, 27))
        self.lineEdit_sh_add_section.setObjectName("lineEdit_sh_add_section")
        self.pushButton_sh_add_section = QtWidgets.QPushButton(self.tab_shafts)
        self.pushButton_sh_add_section.setGeometry(QtCore.QRect(630, 150, 99, 27))
        self.pushButton_sh_add_section.setObjectName("pushButton_sh_add_section")
        self.comboBox_sh_sections = QtWidgets.QComboBox(self.tab_shafts)
        self.comboBox_sh_sections.setGeometry(QtCore.QRect(510, 190, 111, 27))
        self.comboBox_sh_sections.setObjectName("comboBox_sh_sections")
        self.lineEdit_sh_section_i = QtWidgets.QLineEdit(self.tab_shafts)
        self.lineEdit_sh_section_i.setGeometry(QtCore.QRect(690, 230, 113, 27))
        self.lineEdit_sh_section_i.setObjectName("lineEdit_sh_section_i")
        self.lineEdit_section_length = QtWidgets.QLineEdit(self.tab_shafts)
        self.lineEdit_section_length.setGeometry(QtCore.QRect(690, 260, 113, 27))
        self.lineEdit_section_length.setObjectName("lineEdit_section_length")
        self.label_sh_section_i = QtWidgets.QLabel(self.tab_shafts)
        self.label_sh_section_i.setGeometry(QtCore.QRect(470, 230, 211, 21))
        self.label_sh_section_i.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_sh_section_i.setObjectName("label_sh_section_i")
        self.label_sh_section_length = QtWidgets.QLabel(self.tab_shafts)
        self.label_sh_section_length.setGeometry(QtCore.QRect(470, 260, 211, 21))
        self.label_sh_section_length.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_sh_section_length.setObjectName("label_sh_section_length")
        self.pushButton_sh_set_section = QtWidgets.QPushButton(self.tab_shafts)
        self.pushButton_sh_set_section.setGeometry(QtCore.QRect(700, 290, 99, 27))
        self.pushButton_sh_set_section.setObjectName("pushButton_sh_set_section")
        self.label_sh_c_dy = QtWidgets.QLabel(self.tab_shafts)
        self.label_sh_c_dy.setGeometry(QtCore.QRect(500, 320, 201, 51))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_sh_c_dy.setFont(font)
        self.label_sh_c_dy.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_sh_c_dy.setWordWrap(True)
        self.label_sh_c_dy.setObjectName("label_sh_c_dy")
        self.lineEdit_sh_c_dy = QtWidgets.QLineEdit(self.tab_shafts)
        self.lineEdit_sh_c_dy.setGeometry(QtCore.QRect(710, 340, 113, 27))
        self.lineEdit_sh_c_dy.setReadOnly(True)
        self.lineEdit_sh_c_dy.setObjectName("lineEdit_sh_c_dy")
        self.label_sh_r = QtWidgets.QLabel(self.tab_shafts)
        self.label_sh_r.setGeometry(QtCore.QRect(20, 360, 151, 31))
        self.label_sh_r.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_sh_r.setObjectName("label_sh_r")
        self.lineEdit_sh_r = QtWidgets.QLineEdit(self.tab_shafts)
        self.lineEdit_sh_r.setGeometry(QtCore.QRect(180, 360, 113, 27))
        self.lineEdit_sh_r.setText("")
        self.lineEdit_sh_r.setObjectName("lineEdit_sh_r")
        self.lineEdit_sh_d = QtWidgets.QLineEdit(self.tab_shafts)
        self.lineEdit_sh_d.setGeometry(QtCore.QRect(180, 400, 113, 27))
        self.lineEdit_sh_d.setText("")
        self.lineEdit_sh_d.setObjectName("lineEdit_sh_d")
        self.label_sh_d = QtWidgets.QLabel(self.tab_shafts)
        self.label_sh_d.setGeometry(QtCore.QRect(30, 390, 141, 31))
        self.label_sh_d.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_sh_d.setWordWrap(True)
        self.label_sh_d.setObjectName("label_sh_d")
        self.label_sh_c_sc_r = QtWidgets.QLabel(self.tab_shafts)
        self.label_sh_c_sc_r.setGeometry(QtCore.QRect(110, 470, 61, 41))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_sh_c_sc_r.setFont(font)
        self.label_sh_c_sc_r.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_sh_c_sc_r.setWordWrap(True)
        self.label_sh_c_sc_r.setObjectName("label_sh_c_sc_r")
        self.lineEdit_sh_c_sc_r = QtWidgets.QLineEdit(self.tab_shafts)
        self.lineEdit_sh_c_sc_r.setGeometry(QtCore.QRect(180, 480, 113, 27))
        self.lineEdit_sh_c_sc_r.setReadOnly(True)
        self.lineEdit_sh_c_sc_r.setObjectName("lineEdit_sh_c_sc_r")
        self.label_sh_D = QtWidgets.QLabel(self.tab_shafts)
        self.label_sh_D.setGeometry(QtCore.QRect(30, 430, 141, 31))
        self.label_sh_D.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_sh_D.setWordWrap(True)
        self.label_sh_D.setObjectName("label_sh_D")
        self.lineEdit_sh_D = QtWidgets.QLineEdit(self.tab_shafts)
        self.lineEdit_sh_D.setGeometry(QtCore.QRect(180, 440, 113, 27))
        self.lineEdit_sh_D.setText("")
        self.lineEdit_sh_D.setObjectName("lineEdit_sh_D")
        self.label_sh_c_sc_g = QtWidgets.QLabel(self.tab_shafts)
        self.label_sh_c_sc_g.setGeometry(QtCore.QRect(320, 470, 61, 41))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_sh_c_sc_g.setFont(font)
        self.label_sh_c_sc_g.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_sh_c_sc_g.setWordWrap(True)
        self.label_sh_c_sc_g.setObjectName("label_sh_c_sc_g")
        self.comboBox_sh_h_D = QtWidgets.QComboBox(self.tab_shafts)
        self.comboBox_sh_h_D.setGeometry(QtCore.QRect(390, 410, 111, 27))
        self.comboBox_sh_h_D.setObjectName("comboBox_sh_h_D")
        self.label_sh_h_D = QtWidgets.QLabel(self.tab_shafts)
        self.label_sh_h_D.setGeometry(QtCore.QRect(350, 410, 31, 31))
        self.label_sh_h_D.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_sh_h_D.setObjectName("label_sh_h_D")
        self.label_sh_h_r = QtWidgets.QLabel(self.tab_shafts)
        self.label_sh_h_r.setGeometry(QtCore.QRect(350, 440, 31, 31))
        self.label_sh_h_r.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_sh_h_r.setObjectName("label_sh_h_r")
        self.comboBox_sh_h_r = QtWidgets.QComboBox(self.tab_shafts)
        self.comboBox_sh_h_r.setGeometry(QtCore.QRect(390, 440, 111, 27))
        self.comboBox_sh_h_r.setObjectName("comboBox_sh_h_r")
        self.lineEdit_sh_c_sc_g = QtWidgets.QLineEdit(self.tab_shafts)
        self.lineEdit_sh_c_sc_g.setGeometry(QtCore.QRect(390, 480, 113, 27))
        self.lineEdit_sh_c_sc_g.setReadOnly(True)
        self.lineEdit_sh_c_sc_g.setObjectName("lineEdit_sh_c_sc_g")
        self.label_sh_c_sc = QtWidgets.QLabel(self.tab_shafts)
        self.label_sh_c_sc.setGeometry(QtCore.QRect(160, 530, 201, 41))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_sh_c_sc.setFont(font)
        self.label_sh_c_sc.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_sh_c_sc.setWordWrap(True)
        self.label_sh_c_sc.setObjectName("label_sh_c_sc")
        self.lineEdit_sh_c_sc = QtWidgets.QLineEdit(self.tab_shafts)
        self.lineEdit_sh_c_sc.setGeometry(QtCore.QRect(370, 540, 113, 27))
        self.lineEdit_sh_c_sc.setReadOnly(True)
        self.lineEdit_sh_c_sc.setObjectName("lineEdit_sh_c_sc")
        self.groupBox_sh_lambda_sh = QtWidgets.QGroupBox(self.tab_shafts)
        self.groupBox_sh_lambda_sh.setGeometry(QtCore.QRect(450, 640, 200, 63))
        font = QtGui.QFont()
        font.setUnderline(False)
        font.setStrikeOut(False)
        self.groupBox_sh_lambda_sh.setFont(font)
        self.groupBox_sh_lambda_sh.setObjectName("groupBox_sh_lambda_sh")
        self.lineEdit_sh_lambda_sh = QtWidgets.QLineEdit(self.groupBox_sh_lambda_sh)
        self.lineEdit_sh_lambda_sh.setGeometry(QtCore.QRect(0, 30, 200, 27))
        self.lineEdit_sh_lambda_sh.setReadOnly(True)
        self.lineEdit_sh_lambda_sh.setObjectName("lineEdit_sh_lambda_sh")
        self.line_50 = QtWidgets.QFrame(self.tab_shafts)
        self.line_50.setGeometry(QtCore.QRect(20, 580, 881, 20))
        self.line_50.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_50.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_50.setObjectName("line_50")
        self.line_51 = QtWidgets.QFrame(self.tab_shafts)
        self.line_51.setGeometry(QtCore.QRect(450, 0, 20, 351))
        self.line_51.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_51.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_51.setObjectName("line_51")
        self.lineEdit_sh_warning_box = QtWidgets.QLineEdit(self.tab_shafts)
        self.lineEdit_sh_warning_box.setGeometry(QtCore.QRect(10, 640, 330, 70))
        self.lineEdit_sh_warning_box.setReadOnly(True)
        self.lineEdit_sh_warning_box.setObjectName("lineEdit_sh_warning_box")
        self.comboBox_sh_e = QtWidgets.QComboBox(self.tab_shafts)
        self.comboBox_sh_e.setGeometry(QtCore.QRect(740, 10, 111, 27))
        self.comboBox_sh_e.setObjectName("comboBox_sh_e")
        self.lineEdit_sh_e = QtWidgets.QLineEdit(self.tab_shafts)
        self.lineEdit_sh_e.setGeometry(QtCore.QRect(870, 10, 113, 27))
        self.lineEdit_sh_e.setReadOnly(True)
        self.lineEdit_sh_e.setObjectName("lineEdit_sh_e")
        self.lineEdit_sh_b = QtWidgets.QLineEdit(self.tab_shafts)
        self.lineEdit_sh_b.setGeometry(QtCore.QRect(870, 90, 113, 27))
        self.lineEdit_sh_b.setReadOnly(True)
        self.lineEdit_sh_b.setObjectName("lineEdit_sh_b")
        self.label_sh_show_section_i = QtWidgets.QLabel(self.tab_shafts)
        self.label_sh_show_section_i.setGeometry(QtCore.QRect(750, 150, 21, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_sh_show_section_i.setFont(font)
        self.label_sh_show_section_i.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_sh_show_section_i.setObjectName("label_sh_show_section_i")
        self.label_sh_show_section_length = QtWidgets.QLabel(self.tab_shafts)
        self.label_sh_show_section_length.setGeometry(QtCore.QRect(876, 150, 61, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_sh_show_section_length.setFont(font)
        self.label_sh_show_section_length.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_sh_show_section_length.setObjectName("label_sh_show_section_length")
        self.label_sh_section_i_val = QtWidgets.QLabel(self.tab_shafts)
        self.label_sh_section_i_val.setGeometry(QtCore.QRect(780, 150, 67, 31))
        self.label_sh_section_i_val.setText("")
        self.label_sh_section_i_val.setObjectName("label_sh_section_i_val")
        self.label_sh_show_section_length_val = QtWidgets.QLabel(self.tab_shafts)
        self.label_sh_show_section_length_val.setGeometry(QtCore.QRect(950, 150, 67, 31))
        self.label_sh_show_section_length_val.setText("")
        self.label_sh_show_section_length_val.setObjectName("label_sh_show_section_length_val")
        self.addButton_c_me_shafts = QtWidgets.QPushButton(self.tab_shafts)
        self.addButton_c_me_shafts.setGeometry(QtCore.QRect(925, 650, 125, 50))
        self.addButton_c_me_shafts.setObjectName("addButton_c_me_shafts")
        self.tabWidget_mechanical_equipments.addTab(self.tab_shafts, "")
        self.tab_electric_motors = QtWidgets.QWidget()
        self.tab_electric_motors.setObjectName("tab_electric_motors")
        self.label_em_lambda_m_b = QtWidgets.QLabel(self.tab_electric_motors)
        self.label_em_lambda_m_b.setGeometry(QtCore.QRect(0, 50, 191, 21))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_em_lambda_m_b.setFont(font)
        self.label_em_lambda_m_b.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_em_lambda_m_b.setWordWrap(True)
        self.label_em_lambda_m_b.setObjectName("label_em_lambda_m_b")
        self.comboBox_em_motor_type = QtWidgets.QComboBox(self.tab_electric_motors)
        self.comboBox_em_motor_type.setGeometry(QtCore.QRect(200, 20, 201, 27))
        self.comboBox_em_motor_type.setObjectName("comboBox_em_motor_type")
        self.label_em_motor_type = QtWidgets.QLabel(self.tab_electric_motors)
        self.label_em_motor_type.setGeometry(QtCore.QRect(100, 20, 91, 21))
        self.label_em_motor_type.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_em_motor_type.setObjectName("label_em_motor_type")
        self.lineEdit_em_lambda_mb = QtWidgets.QLineEdit(self.tab_electric_motors)
        self.lineEdit_em_lambda_mb.setGeometry(QtCore.QRect(200, 50, 201, 27))
        self.lineEdit_em_lambda_mb.setReadOnly(True)
        self.lineEdit_em_lambda_mb.setObjectName("lineEdit_em_lambda_mb")
        self.label_em_insulation_class = QtWidgets.QLabel(self.tab_electric_motors)
        self.label_em_insulation_class.setGeometry(QtCore.QRect(480, 240, 111, 21))
        self.label_em_insulation_class.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_em_insulation_class.setObjectName("label_em_insulation_class")
        self.lineEdit_em_v_r = QtWidgets.QLineEdit(self.tab_electric_motors)
        self.lineEdit_em_v_r.setGeometry(QtCore.QRect(859, 60, 41, 27))
        self.lineEdit_em_v_r.setObjectName("lineEdit_em_v_r")
        self.comboBox_em_load_type = QtWidgets.QComboBox(self.tab_electric_motors)
        self.comboBox_em_load_type.setGeometry(QtCore.QRect(190, 110, 211, 27))
        self.comboBox_em_load_type.setObjectName("comboBox_em_load_type")
        self.label_em_v_r = QtWidgets.QLabel(self.tab_electric_motors)
        self.label_em_v_r.setGeometry(QtCore.QRect(819, 60, 31, 31))
        self.label_em_v_r.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_em_v_r.setObjectName("label_em_v_r")
        self.label_em_c_sf = QtWidgets.QLabel(self.tab_electric_motors)
        self.label_em_c_sf.setGeometry(QtCore.QRect(20, 130, 161, 51))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_em_c_sf.setFont(font)
        self.label_em_c_sf.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_em_c_sf.setWordWrap(True)
        self.label_em_c_sf.setObjectName("label_em_c_sf")
        self.lineEdit_em_li = QtWidgets.QLineEdit(self.tab_electric_motors)
        self.lineEdit_em_li.setGeometry(QtCore.QRect(200, 230, 211, 27))
        self.lineEdit_em_li.setObjectName("lineEdit_em_li")
        self.lineEdit_em_operating_altitude = QtWidgets.QLineEdit(self.tab_electric_motors)
        self.lineEdit_em_operating_altitude.setGeometry(QtCore.QRect(620, 170, 61, 27))
        self.lineEdit_em_operating_altitude.setObjectName("lineEdit_em_operating_altitude")
        self.lineEdit_em_c_v = QtWidgets.QLineEdit(self.tab_electric_motors)
        self.lineEdit_em_c_v.setGeometry(QtCore.QRect(859, 100, 113, 27))
        self.lineEdit_em_c_v.setReadOnly(True)
        self.lineEdit_em_c_v.setObjectName("lineEdit_em_c_v")
        self.label_em_c_t = QtWidgets.QLabel(self.tab_electric_motors)
        self.label_em_c_t.setGeometry(QtCore.QRect(930, 240, 41, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_em_c_t.setFont(font)
        self.label_em_c_t.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_em_c_t.setWordWrap(True)
        self.label_em_c_t.setObjectName("label_em_c_t")
        self.lineEdit_em_c_sf = QtWidgets.QLineEdit(self.tab_electric_motors)
        self.lineEdit_em_c_sf.setGeometry(QtCore.QRect(190, 150, 211, 27))
        self.lineEdit_em_c_sf.setReadOnly(True)
        self.lineEdit_em_c_sf.setObjectName("lineEdit_em_c_sf")
        self.lineEdit_em_lambda_wi = QtWidgets.QLineEdit(self.tab_electric_motors)
        self.lineEdit_em_lambda_wi.setGeometry(QtCore.QRect(850, 440, 171, 27))
        self.lineEdit_em_lambda_wi.setReadOnly(True)
        self.lineEdit_em_lambda_wi.setObjectName("lineEdit_em_lambda_wi")
        self.label_em_operating_altitude = QtWidgets.QLabel(self.tab_electric_motors)
        self.label_em_operating_altitude.setGeometry(QtCore.QRect(450, 170, 161, 31))
        self.label_em_operating_altitude.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_em_operating_altitude.setObjectName("label_em_operating_altitude")
        self.lineEdit_em_v_u = QtWidgets.QLineEdit(self.tab_electric_motors)
        self.lineEdit_em_v_u.setGeometry(QtCore.QRect(640, 100, 111, 27))
        self.lineEdit_em_v_u.setReadOnly(True)
        self.lineEdit_em_v_u.setObjectName("lineEdit_em_v_u")
        self.label_em_t_0 = QtWidgets.QLabel(self.tab_electric_motors)
        self.label_em_t_0.setGeometry(QtCore.QRect(740, 240, 51, 31))
        self.label_em_t_0.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_em_t_0.setObjectName("label_em_t_0")
        self.line_100 = QtWidgets.QFrame(self.tab_electric_motors)
        self.line_100.setGeometry(QtCore.QRect(10, 200, 401, 3))
        self.line_100.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_100.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_100.setObjectName("line_100")
        self.lineEdit_em_v_d = QtWidgets.QLineEdit(self.tab_electric_motors)
        self.lineEdit_em_v_d.setGeometry(QtCore.QRect(860, 20, 41, 27))
        self.lineEdit_em_v_d.setObjectName("lineEdit_em_v_d")
        self.label_em_v_u = QtWidgets.QLabel(self.tab_electric_motors)
        self.label_em_v_u.setGeometry(QtCore.QRect(590, 100, 41, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_em_v_u.setFont(font)
        self.label_em_v_u.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_em_v_u.setObjectName("label_em_v_u")
        self.label_em_c_v = QtWidgets.QLabel(self.tab_electric_motors)
        self.label_em_c_v.setGeometry(QtCore.QRect(809, 100, 41, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_em_c_v.setFont(font)
        self.label_em_c_v.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_em_c_v.setWordWrap(True)
        self.label_em_c_v.setObjectName("label_em_c_v")
        self.label_em_v_d = QtWidgets.QLabel(self.tab_electric_motors)
        self.label_em_v_d.setGeometry(QtCore.QRect(819, 20, 31, 31))
        self.label_em_v_d.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_em_v_d.setObjectName("label_em_v_d")
        self.label_em_li = QtWidgets.QLabel(self.tab_electric_motors)
        self.label_em_li.setGeometry(QtCore.QRect(60, 220, 131, 41))
        self.label_em_li.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_em_li.setWordWrap(True)
        self.label_em_li.setObjectName("label_em_li")
        self.lineEdit_em_c_alt = QtWidgets.QLineEdit(self.tab_electric_motors)
        self.lineEdit_em_c_alt.setGeometry(QtCore.QRect(750, 170, 113, 27))
        self.lineEdit_em_c_alt.setReadOnly(True)
        self.lineEdit_em_c_alt.setObjectName("lineEdit_em_c_alt")
        self.label_em_lambda_wi = QtWidgets.QLabel(self.tab_electric_motors)
        self.label_em_lambda_wi.setGeometry(QtCore.QRect(650, 430, 191, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_em_lambda_wi.setFont(font)
        self.label_em_lambda_wi.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_em_lambda_wi.setWordWrap(True)
        self.label_em_lambda_wi.setObjectName("label_em_lambda_wi")
        self.label_em_c_alt = QtWidgets.QLabel(self.tab_electric_motors)
        self.label_em_c_alt.setGeometry(QtCore.QRect(690, 170, 51, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_em_c_alt.setFont(font)
        self.label_em_c_alt.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_em_c_alt.setWordWrap(True)
        self.label_em_c_alt.setObjectName("label_em_c_alt")
        self.lineEdit_em_lambda_wi_b = QtWidgets.QLineEdit(self.tab_electric_motors)
        self.lineEdit_em_lambda_wi_b.setGeometry(QtCore.QRect(200, 270, 211, 27))
        self.lineEdit_em_lambda_wi_b.setReadOnly(True)
        self.lineEdit_em_lambda_wi_b.setObjectName("lineEdit_em_lambda_wi_b")
        self.lineEdit_em_c_t = QtWidgets.QLineEdit(self.tab_electric_motors)
        self.lineEdit_em_c_t.setGeometry(QtCore.QRect(970, 240, 113, 27))
        self.lineEdit_em_c_t.setReadOnly(True)
        self.lineEdit_em_c_t.setObjectName("lineEdit_em_c_t")
        self.line_101 = QtWidgets.QFrame(self.tab_electric_motors)
        self.line_101.setGeometry(QtCore.QRect(10, 90, 401, 16))
        self.line_101.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_101.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_101.setObjectName("line_101")
        self.groupBox_em_lambda_m = QtWidgets.QGroupBox(self.tab_electric_motors)
        self.groupBox_em_lambda_m.setGeometry(QtCore.QRect(435, 640, 230, 63))
        font = QtGui.QFont()
        font.setUnderline(False)
        font.setStrikeOut(False)
        self.groupBox_em_lambda_m.setFont(font)
        self.groupBox_em_lambda_m.setObjectName("groupBox_em_lambda_m")
        self.lineEdit_em_lambda_m = QtWidgets.QLineEdit(self.groupBox_em_lambda_m)
        self.lineEdit_em_lambda_m.setGeometry(QtCore.QRect(15, 30, 200, 27))
        self.lineEdit_em_lambda_m.setReadOnly(True)
        self.lineEdit_em_lambda_m.setObjectName("lineEdit_em_lambda_m")
        self.label_em_lambda_wi_b = QtWidgets.QLabel(self.tab_electric_motors)
        self.label_em_lambda_wi_b.setGeometry(QtCore.QRect(0, 260, 191, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_em_lambda_wi_b.setFont(font)
        self.label_em_lambda_wi_b.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_em_lambda_wi_b.setWordWrap(True)
        self.label_em_lambda_wi_b.setObjectName("label_em_lambda_wi_b")
        self.lineEdit_em_t_0 = QtWidgets.QLineEdit(self.tab_electric_motors)
        self.lineEdit_em_t_0.setGeometry(QtCore.QRect(800, 240, 113, 27))
        self.lineEdit_em_t_0.setObjectName("lineEdit_em_t_0")
        self.comboBox_em_insulation_class = QtWidgets.QComboBox(self.tab_electric_motors)
        self.comboBox_em_insulation_class.setGeometry(QtCore.QRect(600, 240, 111, 27))
        self.comboBox_em_insulation_class.setObjectName("comboBox_em_insulation_class")
        self.label_em_load_type = QtWidgets.QLabel(self.tab_electric_motors)
        self.label_em_load_type.setGeometry(QtCore.QRect(90, 110, 91, 21))
        self.label_em_load_type.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_em_load_type.setObjectName("label_em_load_type")
        self.groupBox_em_gr_lambda_g = QtWidgets.QGroupBox(self.tab_electric_motors)
        self.groupBox_em_gr_lambda_g.setGeometry(QtCore.QRect(125, 530, 200, 80))
        self.groupBox_em_gr_lambda_g.setObjectName("groupBox_em_gr_lambda_g")
        self.lineEdit_em_gr_lamda_g = QtWidgets.QLineEdit(self.groupBox_em_gr_lambda_g)
        self.lineEdit_em_gr_lamda_g.setGeometry(QtCore.QRect(0, 30, 200, 27))
        self.lineEdit_em_gr_lamda_g.setReadOnly(True)
        self.lineEdit_em_gr_lamda_g.setObjectName("lineEdit_em_gr_lamda_g")
        self.groupBox_em_be_lambda_be = QtWidgets.QGroupBox(self.tab_electric_motors)
        self.groupBox_em_be_lambda_be.setGeometry(QtCore.QRect(450, 530, 200, 63))
        font = QtGui.QFont()
        font.setUnderline(False)
        font.setStrikeOut(False)
        self.groupBox_em_be_lambda_be.setFont(font)
        self.groupBox_em_be_lambda_be.setObjectName("groupBox_em_be_lambda_be")
        self.lineEdit_em_be_lambda_be = QtWidgets.QLineEdit(self.groupBox_em_be_lambda_be)
        self.lineEdit_em_be_lambda_be.setGeometry(QtCore.QRect(0, 30, 200, 27))
        self.lineEdit_em_be_lambda_be.setReadOnly(True)
        self.lineEdit_em_be_lambda_be.setObjectName("lineEdit_em_be_lambda_be")
        self.groupBox_em_sh_lambda_sh = QtWidgets.QGroupBox(self.tab_electric_motors)
        self.groupBox_em_sh_lambda_sh.setGeometry(QtCore.QRect(775, 530, 200, 63))
        font = QtGui.QFont()
        font.setUnderline(False)
        font.setStrikeOut(False)
        self.groupBox_em_sh_lambda_sh.setFont(font)
        self.groupBox_em_sh_lambda_sh.setObjectName("groupBox_em_sh_lambda_sh")
        self.lineEdit_em_sh_lambda_sh = QtWidgets.QLineEdit(self.groupBox_em_sh_lambda_sh)
        self.lineEdit_em_sh_lambda_sh.setGeometry(QtCore.QRect(0, 30, 200, 27))
        self.lineEdit_em_sh_lambda_sh.setReadOnly(True)
        self.lineEdit_em_sh_lambda_sh.setObjectName("lineEdit_em_sh_lambda_sh")
        self.line_102 = QtWidgets.QFrame(self.tab_electric_motors)
        self.line_102.setGeometry(QtCore.QRect(20, 500, 1111, 16))
        self.line_102.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_102.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_102.setObjectName("line_102")
        self.line_103 = QtWidgets.QFrame(self.tab_electric_motors)
        self.line_103.setGeometry(QtCore.QRect(20, 600, 1101, 16))
        self.line_103.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_103.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_103.setObjectName("line_103")
        self.label_3 = QtWidgets.QLabel(self.tab_electric_motors)
        self.label_3.setGeometry(QtCore.QRect(520, 270, 141, 21))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_3.setFont(font)
        self.label_3.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_3.setObjectName("label_3")
        self.label_4 = QtWidgets.QLabel(self.tab_electric_motors)
        self.label_4.setGeometry(QtCore.QRect(430, 300, 231, 21))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_4.setFont(font)
        self.label_4.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_4.setObjectName("label_4")
        self.label_5 = QtWidgets.QLabel(self.tab_electric_motors)
        self.label_5.setGeometry(QtCore.QRect(460, 330, 201, 21))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_5.setFont(font)
        self.label_5.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_5.setObjectName("label_5")
        self.label_6 = QtWidgets.QLabel(self.tab_electric_motors)
        self.label_6.setGeometry(QtCore.QRect(520, 360, 141, 21))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_6.setFont(font)
        self.label_6.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_6.setObjectName("label_6")
        self.label_em_temp_rating = QtWidgets.QLabel(self.tab_electric_motors)
        self.label_em_temp_rating.setGeometry(QtCore.QRect(670, 270, 67, 21))
        self.label_em_temp_rating.setText("")
        self.label_em_temp_rating.setObjectName("label_em_temp_rating")
        self.label_em_assumed_ambient_temp = QtWidgets.QLabel(self.tab_electric_motors)
        self.label_em_assumed_ambient_temp.setGeometry(QtCore.QRect(670, 300, 67, 21))
        self.label_em_assumed_ambient_temp.setText("")
        self.label_em_assumed_ambient_temp.setObjectName("label_em_assumed_ambient_temp")
        self.label_allowable_temp_rise = QtWidgets.QLabel(self.tab_electric_motors)
        self.label_allowable_temp_rise.setGeometry(QtCore.QRect(670, 330, 67, 21))
        self.label_allowable_temp_rise.setText("")
        self.label_allowable_temp_rise.setObjectName("label_allowable_temp_rise")
        self.label_hot_spot_allow = QtWidgets.QLabel(self.tab_electric_motors)
        self.label_hot_spot_allow.setGeometry(QtCore.QRect(670, 360, 67, 21))
        self.label_hot_spot_allow.setText("")
        self.label_hot_spot_allow.setObjectName("label_hot_spot_allow")
        self.lineEdit_em_warning_box = QtWidgets.QLineEdit(self.tab_electric_motors)
        self.lineEdit_em_warning_box.setGeometry(QtCore.QRect(10, 640, 330, 70))
        self.lineEdit_em_warning_box.setReadOnly(True)
        self.lineEdit_em_warning_box.setObjectName("lineEdit_em_warning_box")
        self.lineEdit_em_greatest_volt_diff = QtWidgets.QLineEdit(self.tab_electric_motors)
        self.lineEdit_em_greatest_volt_diff.setGeometry(QtCore.QRect(640, 20, 113, 27))
        self.lineEdit_em_greatest_volt_diff.setObjectName("lineEdit_em_greatest_volt_diff")
        self.label_em_greatest_volt_diff = QtWidgets.QLabel(self.tab_electric_motors)
        self.label_em_greatest_volt_diff.setGeometry(QtCore.QRect(430, 10, 201, 41))
        self.label_em_greatest_volt_diff.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_em_greatest_volt_diff.setWordWrap(True)
        self.label_em_greatest_volt_diff.setObjectName("label_em_greatest_volt_diff")
        self.lineEdit_em_average_phase_volt = QtWidgets.QLineEdit(self.tab_electric_motors)
        self.lineEdit_em_average_phase_volt.setGeometry(QtCore.QRect(640, 60, 113, 27))
        self.lineEdit_em_average_phase_volt.setObjectName("lineEdit_em_average_phase_volt")
        self.label_em_average_phase_volt = QtWidgets.QLabel(self.tab_electric_motors)
        self.label_em_average_phase_volt.setGeometry(QtCore.QRect(470, 50, 161, 41))
        self.label_em_average_phase_volt.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_em_average_phase_volt.setWordWrap(True)
        self.label_em_average_phase_volt.setObjectName("label_em_average_phase_volt")
        self.line_15 = QtWidgets.QFrame(self.tab_electric_motors)
        self.line_15.setGeometry(QtCore.QRect(420, 20, 20, 461))
        self.line_15.setFrameShape(QtWidgets.QFrame.VLine)
        self.line_15.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_15.setObjectName("line_15")
        self.line_16 = QtWidgets.QFrame(self.tab_electric_motors)
        self.line_16.setGeometry(QtCore.QRect(10, 320, 401, 16))
        self.line_16.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_16.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_16.setObjectName("line_16")
        self.line_17 = QtWidgets.QFrame(self.tab_electric_motors)
        self.line_17.setGeometry(QtCore.QRect(450, 140, 681, 16))
        self.line_17.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_17.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_17.setObjectName("line_17")
        self.line_18 = QtWidgets.QFrame(self.tab_electric_motors)
        self.line_18.setGeometry(QtCore.QRect(450, 220, 681, 3))
        self.line_18.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_18.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_18.setObjectName("line_18")
        self.label_em_lambda_c = QtWidgets.QLabel(self.tab_electric_motors)
        self.label_em_lambda_c.setGeometry(QtCore.QRect(0, 350, 191, 31))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_em_lambda_c.setFont(font)
        self.label_em_lambda_c.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_em_lambda_c.setWordWrap(True)
        self.label_em_lambda_c.setObjectName("label_em_lambda_c")
        self.lineEdit_em_lambda_c = QtWidgets.QLineEdit(self.tab_electric_motors)
        self.lineEdit_em_lambda_c.setGeometry(QtCore.QRect(200, 360, 201, 27))
        self.lineEdit_em_lambda_c.setReadOnly(False)
        self.lineEdit_em_lambda_c.setObjectName("lineEdit_em_lambda_c")
        self.addButton_c_me_electric_motors = QtWidgets.QPushButton(self.tab_electric_motors)
        self.addButton_c_me_electric_motors.setGeometry(QtCore.QRect(925, 650, 125, 50))
        self.addButton_c_me_electric_motors.setObjectName("addButton_c_me_electric_motors")
        self.tabWidget_mechanical_equipments.addTab(self.tab_electric_motors, "")
        self.tab_mechanical_couplings = QtWidgets.QWidget()
        self.tab_mechanical_couplings.setObjectName("tab_mechanical_couplings")
        self.groupBox_cp_gr_lambda_g = QtWidgets.QGroupBox(self.tab_mechanical_couplings)
        self.groupBox_cp_gr_lambda_g.setGeometry(QtCore.QRect(420, 20, 301, 61))
        self.groupBox_cp_gr_lambda_g.setTitle("")
        self.groupBox_cp_gr_lambda_g.setObjectName("groupBox_cp_gr_lambda_g")
        self.lineEdit_cp_gr_lamda_g = QtWidgets.QLineEdit(self.groupBox_cp_gr_lambda_g)
        self.lineEdit_cp_gr_lamda_g.setGeometry(QtCore.QRect(180, 10, 113, 27))
        self.lineEdit_cp_gr_lamda_g.setReadOnly(True)
        self.lineEdit_cp_gr_lamda_g.setObjectName("lineEdit_cp_gr_lamda_g")
        self.label_cp_gr_lambda_g = QtWidgets.QLabel(self.groupBox_cp_gr_lambda_g)
        self.label_cp_gr_lambda_g.setGeometry(QtCore.QRect(0, 10, 171, 20))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_cp_gr_lambda_g.setFont(font)
        self.label_cp_gr_lambda_g.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_cp_gr_lambda_g.setObjectName("label_cp_gr_lambda_g")
        self.groupBox_cp_c_sf = QtWidgets.QGroupBox(self.tab_mechanical_couplings)
        self.groupBox_cp_c_sf.setGeometry(QtCore.QRect(30, 10, 361, 131))
        self.groupBox_cp_c_sf.setTitle("")
        self.groupBox_cp_c_sf.setObjectName("groupBox_cp_c_sf")
        self.label_cp_driven_machinery = QtWidgets.QLabel(self.groupBox_cp_c_sf)
        self.label_cp_driven_machinery.setGeometry(QtCore.QRect(10, 10, 131, 21))
        self.label_cp_driven_machinery.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_cp_driven_machinery.setObjectName("label_cp_driven_machinery")
        self.comboBox_cp_torque = QtWidgets.QComboBox(self.groupBox_cp_c_sf)
        self.comboBox_cp_torque.setGeometry(QtCore.QRect(150, 40, 201, 27))
        self.comboBox_cp_torque.setObjectName("comboBox_cp_torque")
        self.label_cp_c_sf = QtWidgets.QLabel(self.groupBox_cp_c_sf)
        self.label_cp_c_sf.setGeometry(QtCore.QRect(10, 60, 131, 61))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_cp_c_sf.setFont(font)
        self.label_cp_c_sf.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_cp_c_sf.setWordWrap(True)
        self.label_cp_c_sf.setObjectName("label_cp_c_sf")
        self.lineEdit_cp_c_sf = QtWidgets.QLineEdit(self.groupBox_cp_c_sf)
        self.lineEdit_cp_c_sf.setGeometry(QtCore.QRect(150, 80, 91, 27))
        self.lineEdit_cp_c_sf.setReadOnly(True)
        self.lineEdit_cp_c_sf.setObjectName("lineEdit_cp_c_sf")
        self.comboBox_cp_driven_machinery = QtWidgets.QComboBox(self.groupBox_cp_c_sf)
        self.comboBox_cp_driven_machinery.setGeometry(QtCore.QRect(150, 10, 201, 27))
        self.comboBox_cp_driven_machinery.setObjectName("comboBox_cp_driven_machinery")
        self.groupBox_cp_se_lambda_se = QtWidgets.QGroupBox(self.tab_mechanical_couplings)
        self.groupBox_cp_se_lambda_se.setGeometry(QtCore.QRect(740, 20, 301, 61))
        self.groupBox_cp_se_lambda_se.setTitle("")
        self.groupBox_cp_se_lambda_se.setObjectName("groupBox_cp_se_lambda_se")
        self.lineEdit_cp_se_lambda_se = QtWidgets.QLineEdit(self.groupBox_cp_se_lambda_se)
        self.lineEdit_cp_se_lambda_se.setGeometry(QtCore.QRect(180, 10, 113, 27))
        self.lineEdit_cp_se_lambda_se.setReadOnly(True)
        self.lineEdit_cp_se_lambda_se.setObjectName("lineEdit_cp_se_lambda_se")
        self.label_cp_se_lambda_se = QtWidgets.QLabel(self.groupBox_cp_se_lambda_se)
        self.label_cp_se_lambda_se.setGeometry(QtCore.QRect(0, 10, 171, 20))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_cp_se_lambda_se.setFont(font)
        self.label_cp_se_lambda_se.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_cp_se_lambda_se.setObjectName("label_cp_se_lambda_se")
        self.lineEdit_cp_warning_box = QtWidgets.QLineEdit(self.tab_mechanical_couplings)
        self.lineEdit_cp_warning_box.setGeometry(QtCore.QRect(10, 640, 330, 70))
        self.lineEdit_cp_warning_box.setReadOnly(True)
        self.lineEdit_cp_warning_box.setObjectName("lineEdit_cp_warning_box")
        self.groupBox_cp_lambda_cp = QtWidgets.QGroupBox(self.tab_mechanical_couplings)
        self.groupBox_cp_lambda_cp.setGeometry(QtCore.QRect(450, 640, 200, 63))
        font = QtGui.QFont()
        font.setUnderline(False)
        font.setStrikeOut(False)
        self.groupBox_cp_lambda_cp.setFont(font)
        self.groupBox_cp_lambda_cp.setObjectName("groupBox_cp_lambda_cp")
        self.lineEdit_cp_lambda_cp = QtWidgets.QLineEdit(self.groupBox_cp_lambda_cp)
        self.lineEdit_cp_lambda_cp.setGeometry(QtCore.QRect(0, 30, 200, 27))
        self.lineEdit_cp_lambda_cp.setReadOnly(True)
        self.lineEdit_cp_lambda_cp.setObjectName("lineEdit_cp_lambda_cp")
        self.addButton_c_me_mechanical_couplings = QtWidgets.QPushButton(self.tab_mechanical_couplings)
        self.addButton_c_me_mechanical_couplings.setGeometry(QtCore.QRect(925, 650, 125, 50))
        self.addButton_c_me_mechanical_couplings.setObjectName("addButton_c_me_mechanical_couplings")
        self.tabWidget_mechanical_equipments.addTab(self.tab_mechanical_couplings, "")
        self.tab_battery = QtWidgets.QWidget()
        self.tab_battery.setObjectName("tab_battery")
        self.groupBox_bat_lambda_bat = QtWidgets.QGroupBox(self.tab_battery)
        self.groupBox_bat_lambda_bat.setGeometry(QtCore.QRect(450, 640, 200, 63))
        font = QtGui.QFont()
        font.setUnderline(False)
        font.setStrikeOut(False)
        self.groupBox_bat_lambda_bat.setFont(font)
        self.groupBox_bat_lambda_bat.setObjectName("groupBox_bat_lambda_bat")
        self.lineEdit_bat_lambda_bat = QtWidgets.QLineEdit(self.groupBox_bat_lambda_bat)
        self.lineEdit_bat_lambda_bat.setGeometry(QtCore.QRect(0, 30, 200, 27))
        self.lineEdit_bat_lambda_bat.setReadOnly(True)
        self.lineEdit_bat_lambda_bat.setObjectName("lineEdit_bat_lambda_bat")
        self.lineEdit_bat_warning_box = QtWidgets.QLineEdit(self.tab_battery)
        self.lineEdit_bat_warning_box.setGeometry(QtCore.QRect(10, 640, 330, 70))
        self.lineEdit_bat_warning_box.setReadOnly(True)
        self.lineEdit_bat_warning_box.setObjectName("lineEdit_bat_warning_box")
        self.groupBox_bat_lambda_0 = QtWidgets.QGroupBox(self.tab_battery)
        self.groupBox_bat_lambda_0.setGeometry(QtCore.QRect(20, 20, 361, 131))
        self.groupBox_bat_lambda_0.setTitle("")
        self.groupBox_bat_lambda_0.setObjectName("groupBox_bat_lambda_0")
        self.label_bat_device_type = QtWidgets.QLabel(self.groupBox_bat_lambda_0)
        self.label_bat_device_type.setGeometry(QtCore.QRect(10, 10, 131, 21))
        self.label_bat_device_type.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_bat_device_type.setObjectName("label_bat_device_type")
        self.comboBox_bat_device_type_2 = QtWidgets.QComboBox(self.groupBox_bat_lambda_0)
        self.comboBox_bat_device_type_2.setGeometry(QtCore.QRect(150, 40, 201, 27))
        self.comboBox_bat_device_type_2.setObjectName("comboBox_bat_device_type_2")
        self.label_bat_lambda_0 = QtWidgets.QLabel(self.groupBox_bat_lambda_0)
        self.label_bat_lambda_0.setGeometry(QtCore.QRect(10, 60, 131, 61))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_bat_lambda_0.setFont(font)
        self.label_bat_lambda_0.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_bat_lambda_0.setWordWrap(True)
        self.label_bat_lambda_0.setObjectName("label_bat_lambda_0")
        self.lineEdit_bat_lambda_0 = QtWidgets.QLineEdit(self.groupBox_bat_lambda_0)
        self.lineEdit_bat_lambda_0.setGeometry(QtCore.QRect(150, 80, 91, 27))
        self.lineEdit_bat_lambda_0.setReadOnly(True)
        self.lineEdit_bat_lambda_0.setObjectName("lineEdit_bat_lambda_0")
        self.comboBox_bat_device_type = QtWidgets.QComboBox(self.groupBox_bat_lambda_0)
        self.comboBox_bat_device_type.setGeometry(QtCore.QRect(150, 10, 201, 27))
        self.comboBox_bat_device_type.setObjectName("comboBox_bat_device_type")
        self.addButton_c_me_battery = QtWidgets.QPushButton(self.tab_battery)
        self.addButton_c_me_battery.setGeometry(QtCore.QRect(925, 650, 125, 50))
        self.addButton_c_me_battery.setObjectName("addButton_c_me_battery")
        self.tabWidget_mechanical_equipments.addTab(self.tab_battery, "")
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
        self.tabWidget_mechanical_equipments.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        """
            Retranslate Ui Function
        """
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", self.title_name))
        self.label_sp_material_type.setText(_translate("MainWindow", "Material type:"))
        self.label_sp_material.setText(_translate("MainWindow", "Material:"))
        self.label_sp_c_g.setText(_translate("MainWindow", "Material rigidity modulus - CG →"))
        self.label_sp_d_w.setText(_translate("MainWindow", "Wire diameter - Dw (in):"))
        self.label_sp_c_dw.setText(_translate("MainWindow", "Effect of Dw - CDW →"))
        self.label_sp_c_dc.setText(_translate("MainWindow", "Effect of Dc - CDC →"))
        self.label_sp_d_c.setText(_translate("MainWindow", "Coil diameter - Dc (in):"))
        self.label_sp_c_n.setText(_translate("MainWindow", "Effect of Na - CN →"))
        self.label_sp_n_a.setText(_translate("MainWindow", "Active coils numb. - Na:"))
        self.label_sp_l_1.setText(_translate("MainWindow", "Spring deflection - L1 (in):"))
        self.label_sp_l1_l2.setText(_translate("MainWindow", "Effect of L1-L2 - CL →"))
        self.label_sp_l_2.setText(_translate("MainWindow", "Spring deflection - L2 (in):"))
        self.label_sp_material_2.setText(_translate("MainWindow", "Material:"))
        self.label_sp_c_y.setText(_translate("MainWindow", "Effect of material strength - CY →"))
        self.label_sp_c_k.setText(_translate("MainWindow", "Spring concentration factor - CK →"))
        self.label_sp_c_r.setText(_translate("MainWindow", "Spring cycle rate - CR:"))
        self.label_sp_c_cs.setText(_translate("MainWindow", "Effect of CR - CCS →"))
        self.label_sp_c_m.setText(_translate("MainWindow", "CM:"))
        self.groupBox_sp_lambda_sp.setTitle(_translate("MainWindow", "Spring failure rate - λSP"))
        self.addButton_c_me_springs.setText(_translate("MainWindow", "Add Equipment"))
        self.tabWidget_mechanical_equipments.setTabText(self.tabWidget_mechanical_equipments.indexOf(self.tab_springs), _translate("MainWindow", "Springs"))
        self.groupBox_gr_lambda_g.setTitle(_translate("MainWindow", "Gear failure rate - λG"))
        self.label_gr_revolutions.setText(_translate("MainWindow", "Revolutions:"))
        self.label_gr_rpm.setText(_translate("MainWindow", "RPM:"))
        self.label_gr_lambda_g_b.setText(_translate("MainWindow", "Base failure rate - λG,B →"))
        self.label_gr_v_d.setText(_translate("MainWindow", "Design speed - Vd:"))
        self.label_gr_v_0.setText(_translate("MainWindow", "Operating speed - V0:"))
        self.label_gr_c_gs.setText(_translate("MainWindow", "Multiplying factor - CGS →"))
        self.label_gr_l_d.setText(_translate("MainWindow", "Design load - Ld:"))
        self.label_gr_l_0.setText(_translate("MainWindow", "Operating load - L0:"))
        self.label_gr_c_gp.setText(_translate("MainWindow", "Actual gear loading - CGP →"))
        self.label_gr_a_e.setText(_translate("MainWindow", "Misalignment angle - AE (rad):"))
        self.label_gr_c_ga.setText(_translate("MainWindow", "Misalignment - CGA →"))
        self.label_gr_vl.setText(_translate("MainWindow", "Viscosity of lubricant used - VL:"))
        self.label_gr_v0.setText(_translate("MainWindow", "Viscosity of specification lubricant - V0:"))
        self.label_gr_c_gl.setText(_translate("MainWindow", "Lubrication deviation - CGL →"))
        self.label_gr_t_at.setText(_translate("MainWindow", "Operationg temperature - TAT (F):"))
        self.label_gr_c_gt.setText(_translate("MainWindow", "Operating temperature - CGT →"))
        self.label_gr_load_character.setText(_translate("MainWindow", "Character of load:"))
        self.label_gr_prime_mover.setText(_translate("MainWindow", "Prime mover:"))
        self.label_gr_c_gv.setText(_translate("MainWindow", "AGMA Service factor - CGV →"))
        self.addButton_c_me_gears.setText(_translate("MainWindow", "Add Equipment"))
        self.tabWidget_mechanical_equipments.setTabText(self.tabWidget_mechanical_equipments.indexOf(self.tab_gears), _translate("MainWindow", "Gears"))
        self.label_be_c_r.setText(_translate("MainWindow", "Life adjustment factor for reliability - CR →"))
        self.label_be_r.setText(_translate("MainWindow", "R:"))
        self.label_be_c_r_2.setText(_translate("MainWindow", "Life adjustment factor for lubricant - CV →"))
        self.label_be_v0.setText(_translate("MainWindow", "Viscosity of specification fluid - V0:"))
        self.label_be_vl.setText(_translate("MainWindow", "Viscosity of lubricant used - VL:"))
        self.label_be_c_cw.setText(_translate("MainWindow", "Multiplying factor for water contaminant level - CCW →"))
        self.label_be_cw.setText(_translate("MainWindow", "Percentage of water in the lubricant - CW:"))
        self.label_be_c_t.setText(_translate("MainWindow", "Multiplying factor for operating temperature - CT →"))
        self.label_be_t0.setText(_translate("MainWindow", "Operating temperature of bearing - T0 (C):"))
        self.label_be_type_of_application.setText(_translate("MainWindow", "Type of application:"))
        self.radioButton_be_ball_bearing.setText(_translate("MainWindow", "Ball Bearing"))
        self.radioButton_be_roller_bearing.setText(_translate("MainWindow", "Roller Bearing"))
        self.label_be_c_sf.setText(_translate("MainWindow", "Bearing service factor - CSF →"))
        self.label_be_type_of_application_2.setText(_translate("MainWindow", "Type of application:"))
        self.label_be_bearing_diameter.setText(_translate("MainWindow", "Bearing diameter (mm):"))
        self.label_be_c_c.setText(_translate("MainWindow", "Bearing contamination level - CC →"))
        self.label_be_c_y.setText(_translate("MainWindow", "Multiplying factor applied load - CY →"))
        self.label_be_l_s.setText(_translate("MainWindow", "Dynamic load rating - Ls:"))
        self.label_be_l_a.setText(_translate("MainWindow", "Equivalent radial load - LA:"))
        self.label_be_n.setText(_translate("MainWindow", "Operation speed - n:"))
        self.label_be_lambda_be_b.setText(_translate("MainWindow", "Base failure rate - λBE,B →"))
        self.groupBox_be_lambda_be.setTitle(_translate("MainWindow", "Bearing failure rate - λBE"))
        self.label_2.setText(_translate("MainWindow", "Select the bearing type."))
        self.addButton_c_me_bearings.setText(_translate("MainWindow", "Add Equipment"))
        self.tabWidget_mechanical_equipments.setTabText(self.tabWidget_mechanical_equipments.indexOf(self.tab_bearings), _translate("MainWindow", "Bearings"))
        self.groupBox_ac_lambda_ac.setTitle(_translate("MainWindow", "Actuator failure rate - λAC"))
        self.label_ac_w_a.setText(_translate("MainWindow", "Axial load - WA (Ibf):"))
        self.label_ac_n_o.setText(_translate("MainWindow", "Number of cycles - No →"))
        self.label_ac_e_1.setText(_translate("MainWindow", "Modulus of elasticity, cylinder - E1 (lbs/in^2):"))
        self.label_ac_mu_2.setText(_translate("MainWindow", "Poisson\'s ratio, piston - η2:"))
        self.radioButton_ac_gama_0_54.setText(_translate("MainWindow", "Wear factor (γ) = 0.54"))
        self.label_ac_e_2.setText(_translate("MainWindow", "Modulus of elasticity, piston - E2 (lbs/in^2):"))
        self.label_ac_various_metals.setText(_translate("MainWindow", "Various metals:"))
        self.label_ac_lambda_ac_b.setText(_translate("MainWindow", "Base failure rate - λAC,B →"))
        self.label_ac_d_2.setText(_translate("MainWindow", "Diameter of piston - D2(in):"))
        self.radioButton_ac_gama_0_2.setText(_translate("MainWindow", "Wear factor (γ) = 0.20"))
        self.label_ac_f_y.setText(_translate("MainWindow", "Yield strength - Fy:"))
        self.label_ac_d_1.setText(_translate("MainWindow", "Diameter of cylinder - D1(in):"))
        self.label_ac_mu_1.setText(_translate("MainWindow", "Poisson\'s ratio, cylinder - η1:"))
        self.label_ac_h_p.setText(_translate("MainWindow", "Piston hardness - Hp:"))
        self.label_ac_material.setText(_translate("MainWindow", "Material:"))
        self.radioButton_ac_piston.setText(_translate("MainWindow", "Piston"))
        self.label_ac_h_c.setText(_translate("MainWindow", "Cylinder hardness - Hc:"))
        self.label_ac_c_s.setText(_translate("MainWindow", "Filtration multiplying factor - Cs:"))
        self.label_ac_c_cp.setText(_translate("MainWindow", "Contaminant multiplying factor - Ccp →"))
        self.radioButton_ac_cylinder.setText(_translate("MainWindow", "Cylinder"))
        self.label_ac_filter_size.setText(_translate("MainWindow", "Filter size (micron):"))
        self.label_ac_c_t.setText(_translate("MainWindow", "Temperature multiplying factor - CT →"))
        self.label_ac_t.setText(_translate("MainWindow", "Operating temperature - T (K):"))
        self.label_ac_teta.setText(_translate("MainWindow", "Activation energy constant - θ (K):"))
        self.addButton_c_me_actuators.setText(_translate("MainWindow", "Add Equipment"))
        self.tabWidget_mechanical_equipments.setTabText(self.tabWidget_mechanical_equipments.indexOf(self.tab_actuators), _translate("MainWindow", "Actuators"))
        self.label_sh_lambda_sh_b.setText(_translate("MainWindow", "Base failure rate - λSH,B →"))
        self.label_sh_n.setText(_translate("MainWindow", "Number of cycles - N:"))
        self.label_sh_surface_finish.setText(_translate("MainWindow", "Shaft surface finish:"))
        self.label_sh_c_f.setText(_translate("MainWindow", "Shaft surface finish factor - Cf →"))
        self.label_sh_t_s.setText(_translate("MainWindow", "Tensile strength of material - Ts:"))
        self.label_sh_t_at.setText(_translate("MainWindow", "Operating temperature - TAT:"))
        self.label_sh_c_t.setText(_translate("MainWindow", "Material temperature multiplying factor - CT→"))
        self.label_sh_e.setText(_translate("MainWindow", "Modulus of elasticity of shaft material- E (Ibs/in^2):"))
        self.label_sh_f.setText(_translate("MainWindow", "Fluid radial unbalance force or load weight - F (Ib):"))
        self.label_sh_b.setText(_translate("MainWindow", "Shaft deflection - b (in):"))
        self.label.setText(_translate("MainWindow", "Add shaft section."))
        self.pushButton_sh_add_section.setText(_translate("MainWindow", "Add"))
        self.label_sh_section_i.setText(_translate("MainWindow", "Moment of inertia - I (in^4):"))
        self.label_sh_section_length.setText(_translate("MainWindow", "Length (in):"))
        self.pushButton_sh_set_section.setText(_translate("MainWindow", "Set"))
        self.label_sh_c_dy.setText(_translate("MainWindow", "Shaft displacement multiplying factor - CDY →"))
        self.label_sh_r.setText(_translate("MainWindow", "Radius of fillet - r (in):"))
        self.label_sh_d.setText(_translate("MainWindow", "Transitioned shaft diameter - d (in):"))
        self.label_sh_c_sc_r.setText(_translate("MainWindow", "CSC-R→"))
        self.label_sh_D.setText(_translate("MainWindow", "Initial shaft diameter - D (in):"))
        self.label_sh_c_sc_g.setText(_translate("MainWindow", "CSC-G→"))
        self.label_sh_h_D.setText(_translate("MainWindow", "h/D:"))
        self.label_sh_h_r.setText(_translate("MainWindow", "h/r:"))
        self.label_sh_c_sc.setText(_translate("MainWindow", "Shaft stress concentration factor factor - CSC→"))
        self.groupBox_sh_lambda_sh.setTitle(_translate("MainWindow", "Shaft failure rate - λSH"))
        self.label_sh_show_section_i.setText(_translate("MainWindow", "I:"))
        self.label_sh_show_section_length.setText(_translate("MainWindow", "Length:"))
        self.addButton_c_me_shafts.setText(_translate("MainWindow", "Add Equipment"))
        self.tabWidget_mechanical_equipments.setTabText(self.tabWidget_mechanical_equipments.indexOf(self.tab_shafts), _translate("MainWindow", "Shafts"))
        self.label_em_lambda_m_b.setText(_translate("MainWindow", "Base failure rate - λM,B →"))
        self.label_em_motor_type.setText(_translate("MainWindow", "Motor type:"))
        self.label_em_insulation_class.setText(_translate("MainWindow", "Insulation class:"))
        self.label_em_v_r.setText(_translate("MainWindow", "VR:"))
        self.label_em_c_sf.setText(_translate("MainWindow", "Load service factor - CSF →"))
        self.label_em_c_t.setText(_translate("MainWindow", "CT →"))
        self.label_em_operating_altitude.setText(_translate("MainWindow", "Operating altitude (ft):"))
        self.label_em_t_0.setText(_translate("MainWindow", "T0 (°C):"))
        self.label_em_v_u.setText(_translate("MainWindow", "VU →"))
        self.label_em_c_v.setText(_translate("MainWindow", "CV →"))
        self.label_em_v_d.setText(_translate("MainWindow", "VD:"))
        self.label_em_li.setText(_translate("MainWindow", "Expected winding life - LI (hours):"))
        self.label_em_lambda_wi.setText(_translate("MainWindow", "Failure rate of elc. mot. wind. - λWI →"))
        self.label_em_c_alt.setText(_translate("MainWindow", "Calt →"))
        self.groupBox_em_lambda_m.setTitle(_translate("MainWindow", "Electric motors failure rate - λM"))
        self.label_em_lambda_wi_b.setText(_translate("MainWindow", "Base failure rate of elc. mot. wind. - λWI,B →"))
        self.label_em_load_type.setText(_translate("MainWindow", "Load type:"))
        self.groupBox_em_gr_lambda_g.setTitle(_translate("MainWindow", "Gear failure rate - λG"))
        self.groupBox_em_be_lambda_be.setTitle(_translate("MainWindow", "Bearing failure rate - λBE"))
        self.groupBox_em_sh_lambda_sh.setTitle(_translate("MainWindow", "Shaft failure rate - λSH"))
        self.label_3.setText(_translate("MainWindow", "Temperature rating:"))
        self.label_4.setText(_translate("MainWindow", " Assumed ambient temperature:"))
        self.label_5.setText(_translate("MainWindow", "Allowable temperature rise:"))
        self.label_6.setText(_translate("MainWindow", "Hot spot allowance:"))
        self.label_em_greatest_volt_diff.setText(_translate("MainWindow", "Greatest voltage difference:"))
        self.label_em_average_phase_volt.setText(_translate("MainWindow", "Average phase voltage:"))
        self.label_em_lambda_c.setText(_translate("MainWindow", "Failure rate of capacitor (if applicable)- λc →"))
        self.addButton_c_me_electric_motors.setText(_translate("MainWindow", "Add Equipment"))
        self.tabWidget_mechanical_equipments.setTabText(self.tabWidget_mechanical_equipments.indexOf(self.tab_electric_motors), _translate("MainWindow", "Electric Motors"))
        self.label_cp_gr_lambda_g.setText(_translate("MainWindow", "Gear failure rate - λG →"))
        self.label_cp_driven_machinery.setText(_translate("MainWindow", "Driven Machinery:"))
        self.label_cp_c_sf.setText(_translate("MainWindow", "Service factor for coupling - CSF →"))
        self.label_cp_se_lambda_se.setText(_translate("MainWindow", "Seal failure rate - λSE →"))
        self.groupBox_cp_lambda_cp.setTitle(_translate("MainWindow", "Coupling failure rate - λCP"))
        self.addButton_c_me_mechanical_couplings.setText(_translate("MainWindow", "Add Equipment"))
        self.tabWidget_mechanical_equipments.setTabText(self.tabWidget_mechanical_equipments.indexOf(self.tab_mechanical_couplings), _translate("MainWindow", "Mechanical Couplings"))
        self.groupBox_bat_lambda_bat.setTitle(_translate("MainWindow", "Battery hazard rate - λ"))
        self.label_bat_device_type.setText(_translate("MainWindow", "Device type:"))
        self.label_bat_lambda_0.setText(_translate("MainWindow", "λ0 →"))
        self.addButton_c_me_battery.setText(_translate("MainWindow", "Add Equipment"))
        self.tabWidget_mechanical_equipments.setTabText(self.tabWidget_mechanical_equipments.indexOf(self.tab_battery), _translate("MainWindow", "Battery"))

# ---------------------------------------------------------------------

    def gui_main(self):
        """
            Mechanical Equipments Gui Main Functions
        """
        # CONFIGURATION // MECHANICAL EQUIPMENTS
        self.spring_tab_func()
        self.gear_tab_func()
        self.bearing_tab_func()
        self.actuator_tab_func()
        self.shaft_tab_func()
        self.em_tab_func()
        self.couplings_tab_func()
        self.bat_tab_func()
        self.mechanical_equipments_general_events_func()

        self.mechanical_equipments_button_click_event_func()


    def mechanical_equipments_button_click_event_func(self):
        """
            Mechanical Equipments Gui Button Click Events Functions
        """
        self.addButton_c_me_springs.clicked.connect(self.clicked_button_c_me_springs_func)
        self.addButton_c_me_gears.clicked.connect(self.clicked_button_c_me_gears_func)
        self.addButton_c_me_bearings.clicked.connect(self.clicked_button_c_me_bearings_func)
        self.addButton_c_me_actuators.clicked.connect(self.clicked_button_c_me_actuators_func)
        self.addButton_c_me_shafts.clicked.connect(self.clicked_button_c_me_shafts_func)
        self.addButton_c_me_electric_motors.clicked.connect(self.clicked_button_c_me_electric_motors_func)
        self.addButton_c_me_mechanical_couplings.clicked.connect(self.clicked_button_c_me_mechanical_couplings_func)
        self.addButton_c_me_battery.clicked.connect(self.clicked_button_c_me_battery_func)


    def clicked_main_func(self, set_data):
        """
            Set mechanical equipments value to PHM Gui functions
        """
        create_object = eval(str("self.ui_class." + str(self.data_path)))
        create_object.setText(str(set_data))

        if self.equipment_type:
            self.ui_class.add_equipment_sub_module_window.close()

        else:
            self.ui_class.add_equipment_component_window.close()


    def clicked_button_c_me_springs_func(self):
        """
            Set springs value function
        """
        self.clicked_main_func(self.lineEdit_sp_lamda_sp.text())


    def clicked_button_c_me_gears_func(self):
        """
            Set gears value function
        """
        self.clicked_main_func(self.lineEdit_gr_lamda_g.text())


    def clicked_button_c_me_bearings_func(self):
        """
            Set bearings value function
        """
        self.clicked_main_func(self.lineEdit_be_lambda_be.text())


    def clicked_button_c_me_actuators_func(self):
        """
            Set actuators value function
        """
        self.clicked_main_func(self.lineEdit_ac_lambda_ac.text())


    def clicked_button_c_me_shafts_func(self):
        """
            Set shafts value function
        """
        self.clicked_main_func(self.lineEdit_sh_lambda_sh.text())


    def clicked_button_c_me_electric_motors_func(self):
        """
            Set electric_motors value function
        """
        self.clicked_main_func(self.lineEdit_em_lambda_m.text())


    def clicked_button_c_me_mechanical_couplings_func(self):
        """
            Set mechanical_couplings value function
        """
        self.clicked_main_func(self.lineEdit_cp_lambda_cp.text())


    def clicked_button_c_me_battery_func(self):
        """
            Set battery value function
        """
        self.clicked_main_func(self.lineEdit_bat_lambda_bat.text())



# "CONTROL FUNCTIONS OF INDEX CHANGES" - START -
  # CONFIGURATION // MECHANICAL EQUIPMENTS
    def mechanical_equipments_general_events_func(self):
        """
            Mechanical Equipments General Events Func
        """
     # SPRING TAB CONFIGURATION
        self.comboBox_sp_material_type.currentIndexChanged.connect(self.spring_tab_func_2)
        self.comboBox_sp_material.currentIndexChanged.connect(self.spring_c_g_func)
        self.lineEdit_sp_d_w.textChanged.connect(self.spring_c_dw_func)
        self.lineEdit_sp_d_w.textChanged.connect(self.spring_c_k_func)
        self.lineEdit_sp_d_c.textChanged.connect(self.spring_c_dc_func)
        self.lineEdit_sp_d_c.textChanged.connect(self.spring_c_k_func)
        self.lineEdit_sp_n_a.textChanged.connect(self.spring_c_n_func)
        self.lineEdit_sp_l_1.textChanged.connect(self.spring_c_l_func)
        self.lineEdit_sp_l_2.textChanged.connect(self.spring_c_l_func)
        self.comboBox_sp_material_2.currentIndexChanged.connect(self.spring_c_y_func)
        self.lineEdit_sp_c_r.textChanged.connect(self.spring_c_cs_func)
        self.lineEdit_sp_c_m.textChanged.connect(self.spring_c_m_func)
        self.lineEdit_sp_c_g.textChanged.connect(self.spring_hazard_rate_func)
        self.lineEdit_sp_c_dw.textChanged.connect(self.spring_hazard_rate_func)
        self.lineEdit_sp_c_dc.textChanged.connect(self.spring_hazard_rate_func)
        self.lineEdit_sp_c_k.textChanged.connect(self.spring_hazard_rate_func)
        self.lineEdit_sp_c_n.textChanged.connect(self.spring_hazard_rate_func)
        self.lineEdit_sp_l1_l2.textChanged.connect(self.spring_hazard_rate_func)
        self.lineEdit_sp_c_y.textChanged.connect(self.spring_hazard_rate_func)
        self.lineEdit_sp_c_r.textChanged.connect(self.spring_hazard_rate_func)
        self.lineEdit_sp_c_m.textChanged.connect(self.spring_hazard_rate_func)
        self.lineEdit_sp_c_cs.textChanged.connect(self.spring_hazard_rate_func)


     # GEAR TAB CONFIGURATION
        self.lineEdit_gr_revolutions.textChanged.connect(self.gear_lambda_gr_b_func)
        self.lineEdit_gr_rpm.textChanged.connect(self.gear_lambda_gr_b_func)
        self.lineEdit_gr_v0.textChanged.connect(self.gear_c_gs_func)
        self.lineEdit_gr_v_d.textChanged.connect(self.gear_c_gs_func)
        self.lineEdit_gr_l_0.textChanged.connect(self.gear_c_gp_func)
        self.lineEdit_gr_l_d.textChanged.connect(self.gear_c_gp_func)
        self.lineEdit_gr_a_e.textChanged.connect(self.gear_c_ga_func)
        self.lineEdit_gr_v0_4.textChanged.connect(self.gear_c_gl_func)
        self.lineEdit_gr_vl.textChanged.connect(self.gear_c_gl_func)
        self.lineEdit_gr_t_at.textChanged.connect(self.gear_c_gt_func)
        self.comboBox_gr_prime_mover.currentIndexChanged.connect(self.gear_c_gv_func)
        self.comboBox_gr_load_character.currentIndexChanged.connect(self.gear_c_gv_func_2)
        self.lineEdit_gr_lambda_g_b.textChanged.connect(self.gear_hazard_rate_func)
        self.lineEdit_gr_c_gs.textChanged.connect(self.gear_hazard_rate_func)
        self.lineEdit_gr_c_gp.textChanged.connect(self.gear_hazard_rate_func)
        self.lineEdit_gr_c_ga.textChanged.connect(self.gear_hazard_rate_func)
        self.lineEdit_gr_c_gl.textChanged.connect(self.gear_hazard_rate_func)
        self.lineEdit_gr_c_gt.textChanged.connect(self.gear_hazard_rate_func)
        self.lineEdit_gr_c_gv.textChanged.connect(self.gear_hazard_rate_func)


     # BEARING TAB CONFIGURATION
        self.radioButton_be_ball_bearing.toggled.connect(self.bearing_lambda_be_b_and_be_c_y_func)
        self.radioButton_be_roller_bearing.toggled.connect(self.bearing_lambda_be_b_and_be_c_y_func)
        self.lineEdit_be_l_a.textChanged.connect(self.bearing_lambda_be_b_and_be_c_y_func)
        self.lineEdit_be_l_s.textChanged.connect(self.bearing_lambda_be_b_and_be_c_y_func)
        self.lineEdit_be_n.textChanged.connect(self.bearing_lambda_be_b_and_be_c_y_func)
        self.lineEdit_be_r.textChanged.connect(self.bearing_c_r_func)
        self.lineEdit_be_v0.textChanged.connect(self.bearing_c_v_func)
        self.lineEdit_be_vl.textChanged.connect(self.bearing_c_v_func)
        self.lineEdit_be_cw.textChanged.connect(self.bearing_c_cw_func)
        self.lineEdit_be_t0.textChanged.connect(self.bearing_c_t_func)
        self.comboBox_be_type_of_application.currentIndexChanged.connect(self.bearing_c_sf_func)
        self.comboBox_be_type_of_application_2.currentIndexChanged.connect(self.bearing_c_c_func)
        self.lineEdit_be_bearing_diameter.textChanged.connect(self.bearing_c_c_func)
        self.lineEdit_be_lambda_be_b.textChanged.connect(self.bearing_hazard_rate_func)
        self.lineEdit_be_c_y.textChanged.connect(self.bearing_hazard_rate_func)
        self.lineEdit_be_c_r.textChanged.connect(self.bearing_hazard_rate_func)
        self.lineEdit_be_c_v.textChanged.connect(self.bearing_hazard_rate_func)
        self.lineEdit_be_c_cw.textChanged.connect(self.bearing_hazard_rate_func)
        self.lineEdit_be_c_t.textChanged.connect(self.bearing_hazard_rate_func)
        self.lineEdit_be_c_sf.textChanged.connect(self.bearing_hazard_rate_func)
        self.lineEdit_be_c_c.textChanged.connect(self.bearing_hazard_rate_func)


     # ACTUATORS TAB CONFIGURATION
        self.radioButton_ac_gama_0_2.toggled.connect(self.actuatior_gama_func)
        self.radioButton_ac_gama_0_54.toggled.connect(self.actuatior_gama_func)
        self.comboBox_ac_various_metals.currentIndexChanged.connect(self.actuator_f_y_func)
        self.comboBox_ac_materials.currentIndexChanged.connect(self.actuator_tab_func_2)
        self.radioButton_ac_gama_0_2.toggled.connect(self.actuator_n_o_func)
        self.radioButton_ac_gama_0_54.toggled.connect(self.actuator_n_o_func)
        self.lineEdit_ac_f_y.textChanged.connect(self.actuator_n_o_func)
        self.lineEdit_ac_w_a.textChanged.connect(self.actuator_n_o_func)
        self.lineEdit_ac_d_1.textChanged.connect(self.actuator_n_o_func)
        self.lineEdit_ac_d_2.textChanged.connect(self.actuator_n_o_func)
        self.lineEdit_ac_mu_1.textChanged.connect(self.actuator_n_o_func)
        self.lineEdit_ac_mu_2.textChanged.connect(self.actuator_n_o_func)
        self.lineEdit_ac_e_1.textChanged.connect(self.actuator_n_o_func)
        self.lineEdit_ac_e_2.textChanged.connect(self.actuator_n_o_func)
        self.lineEdit_ac_n_o.textChanged.connect(self.actuator_lambda_ac_b_func)
        self.radioButton_ac_cylinder.toggled.connect(self.actuator_piston_cylinder_rad_but_control)
        self.radioButton_ac_piston.toggled.connect(self.actuator_piston_cylinder_rad_but_control)
        self.comboBox_ac_materials.currentIndexChanged.connect(self.actutor_hardness_func)
        self.comboBox_material_types.currentIndexChanged.connect(self.actutor_hardness_func)
        self.lineEdit_ac_h_c.textChanged.connect(self.actuator_c_h_func)
        self.lineEdit_ac_h_p.textChanged.connect(self.actuator_c_h_func)
        self.lineEdit_ac_filter_size.textChanged.connect(self.actuator_c_s_func)
        self.lineEdit_ac_h_c.textChanged.connect(self.actuator_c_cp_func)
        self.lineEdit_ac_h_p.textChanged.connect(self.actuator_c_cp_func)
        self.lineEdit_ac_c_s.textChanged.connect(self.actuator_c_cp_func)
        self.lineEdit_ac_t.textChanged.connect(self.actuator_c_t_func)
        self.lineEdit_ac_teta.textChanged.connect(self.actuator_c_t_func)
        self.lineEdit_ac_lambda_ac_b.textChanged.connect(self.actuator_hazard_rate_func)
        self.lineEdit_ac_c_cp.textChanged.connect(self.actuator_hazard_rate_func)
        self.lineEdit_ac_c_t.textChanged.connect(self.actuator_hazard_rate_func)


     # SHAFT TAB CONFIGURATION
        self.lineEdit_sh_n.textChanged.connect(self.shaft_lambda_s_h_b_func)
        self.comboBox_sh_surface_finish.currentIndexChanged.connect(self.shaft_c_f_func)
        self.lineEdit_sh_t_s.textChanged.connect(self.shaft_c_f_func)
        self.lineEdit_sh_t_at.textChanged.connect(self.shaft_c_t_func)
        self.lineEdit_sh_r.textChanged.connect(self.shaft_c_sc_r_func)
        self.lineEdit_sh_D.textChanged.connect(self.shaft_c_sc_r_func)
        self.lineEdit_sh_d.textChanged.connect(self.shaft_c_sc_r_func)
        self.comboBox_sh_h_D.currentIndexChanged.connect(self.shaft_c_sc_g_func)
        self.comboBox_sh_h_r.currentIndexChanged.connect(self.shaft_c_sc_g_func_2)
        self.comboBox_sh_h_D.currentIndexChanged.connect(self.shaft_c_sc_func)
        self.comboBox_sh_h_r.currentIndexChanged.connect(self.shaft_c_sc_func)
        self.lineEdit_sh_r.textChanged.connect(self.shaft_c_sc_func)
        self.lineEdit_sh_D.textChanged.connect(self.shaft_c_sc_func)
        self.lineEdit_sh_d.textChanged.connect(self.shaft_c_sc_func)
        self.comboBox_sh_e.currentIndexChanged.connect(self.shaft_sh_e_control)
        self.comboBox_sh_b.currentIndexChanged.connect(self.shaft_sh_b_control)
        self.lineEdit_sh_f.textChanged.connect(self.shaft_sh_c_dy_func)
        self.pushButton_sh_add_section.clicked.connect(self.shaft_sh_add_section_control)
        self.pushButton_sh_set_section.clicked.connect(self.shaft_sh_set_section_control)
        self.pushButton_sh_set_section.clicked.connect(self.shaft_sh_c_dy_func)
        self.comboBox_sh_sections.currentIndexChanged.connect(self.shaft_sh_sections_show_values)


     # ELECTRIC MOTORS TAB CONFIGURATION
        self.comboBox_em_motor_type.currentIndexChanged.connect(self.em_lambda_m_b_func)
        self.comboBox_em_motor_type.currentIndexChanged.connect(self.em_c_v_func)
        self.comboBox_em_load_type.currentIndexChanged.connect(self.em_c_s_f_func)
        self.lineEdit_em_li.textChanged.connect(self.em_lambda_wi_b_func)
        self.comboBox_em_insulation_class.currentIndexChanged.connect(self.em_c_t_func)
        self.lineEdit_em_t_0.textChanged.connect(self.em_c_t_func)
        self.lineEdit_em_greatest_volt_diff.textChanged.connect(self.em_v_u_func)
        self.lineEdit_em_average_phase_volt.textChanged.connect(self.em_v_u_func)
        self.lineEdit_em_v_d.textChanged.connect(self.em_c_v_func)
        self.lineEdit_em_v_r.textChanged.connect(self.em_c_v_func)
        self.lineEdit_em_v_u.textChanged.connect(self.em_c_v_func)
        self.lineEdit_em_operating_altitude.textChanged.connect(self.em_c_alt_func)
        self.lineEdit_em_lambda_c.textChanged.connect(self.em_lambda_c_func)
        self.lineEdit_em_lambda_mb.textChanged.connect(self.em_hazard_rate_func)
        self.lineEdit_em_c_sf.textChanged.connect(self.em_hazard_rate_func)
        self.lineEdit_em_lambda_c.textChanged.connect(self.em_hazard_rate_func)
        self.lineEdit_em_lambda_wi.textChanged.connect(self.em_hazard_rate_func)
        self.lineEdit_em_gr_lamda_g.textChanged.connect(self.em_hazard_rate_func)
        self.lineEdit_em_be_lambda_be.textChanged.connect(self.em_hazard_rate_func)
        self.lineEdit_em_sh_lambda_sh.textChanged.connect(self.em_hazard_rate_func)


     # MECHANICAL COUPLINGS TAB CONFIGURATION
        self.comboBox_cp_driven_machinery.currentIndexChanged.connect(self.couplings_tab_func_2)
        self.comboBox_cp_torque.currentIndexChanged.connect(self.couplings_c_sf_func)
        self.lineEdit_cp_se_lambda_se.setText(str(0.0))
        self.lineEdit_cp_c_sf.textChanged.connect(self.couplings_hazard_rate_func)
        self.lineEdit_cp_gr_lamda_g.textChanged.connect(self.couplings_hazard_rate_func)
        self.lineEdit_cp_se_lambda_se.textChanged.connect(self.couplings_hazard_rate_func)


     # BATTERY TAB CONFIGURATION
        self.comboBox_bat_device_type.currentIndexChanged.connect(self.bat_tab_func_2)
        self.comboBox_bat_device_type_2.currentIndexChanged.connect(self.bat_tab_func_3)
        self.lineEdit_bat_lambda_0.textChanged.connect(self.bat_hazard_rate_func)


# "CONTROL FUNCTIONS OF INDEX CHANGES" - END -


# "CONFIGURATION // MECHANICAL EQUIPMENTS FUNCTIONS" - START -
  # SPRINGS
    def spring_tab_func(self):
        """
            Springs Tab Function
        """
        self.comboBox_sp_material_type.clear()
        self.comboBox_sp_material.clear()
        self.comboBox_sp_material_2.clear()

        self.comboBox_sp_material_type.addItem("None")
        self.comboBox_sp_material.addItem("None")
        self.comboBox_sp_material_2.addItem("None")

        self.comboBox_sp_material_type.addItems(self.mechanical_equipments_dict['Springs']['Material c_g'].keys())
        self.comboBox_sp_material_2.addItems(self.mechanical_equipments_dict['Springs']['Material c_y'])

        self.spring_hazard_rate_func()


    def spring_tab_func_2(self):
        """
            Springs Tab Function 2
        """
        self.comboBox_sp_material.clear()
        current_material_type = self.comboBox_sp_material_type.currentText()

        if current_material_type != "" and current_material_type != "None":
            self.comboBox_sp_material.addItems(self.mechanical_equipments_dict['Springs']['Material c_g'][str(current_material_type)].keys())

        else:
            self.lineEdit_sp_c_g.clear()
            self.m_eq.sp_c_g = 1.0
            self.lineEdit_sp_c_g.setText(str(self.m_eq.sp_c_g))


    def spring_c_g_func(self):
        """
            Springs C_g Function
        """
        self.lineEdit_sp_c_g.clear()
        current_material_type = self.comboBox_sp_material_type.currentText()
        current_material = self.comboBox_sp_material.currentText()

        if current_material != "" and current_material_type != "" and current_material != 'None' and current_material_type != 'None':
            self.m_eq.sp_c_g = self.mechanical_equipments_dict['Springs']['Material c_g'][str(current_material_type)][str(current_material)]

        else:
            self.m_eq.sp_c_g = 1.0

        self.lineEdit_sp_c_g.setText(str(self.m_eq.sp_c_g))


    def spring_c_dw_func(self):
        """
            Springs C_dw Function
        """
        try:
            self.lineEdit_sp_c_dw.clear()
            sp_d_w = self.lineEdit_sp_d_w.text()

            if sp_d_w != "":
                self.m_eq.sp_c_dw_func(float(sp_d_w))
                self.lineEdit_sp_c_dw.setText(str(self.m_eq.get_sp_c_dw()))

            else:
                self.m_eq.sp_c_dw = 1.0
                self.lineEdit_sp_c_dw.setText(str(self.m_eq.get_sp_c_dw()))

        except Exception as err:
            print(err)


    def spring_c_dc_func(self):
        """
            Springs C_dc Function
        """
        try:
            self.lineEdit_sp_c_dc.clear()
            sp_d_c = self.lineEdit_sp_d_c.text()

            if sp_d_c != "":
                self.m_eq.sp_c_dc_func(float(sp_d_c))

            else:
                self.m_eq.sp_c_dc = 1.0

            self.lineEdit_sp_c_dc.setText(str(self.m_eq.get_sp_c_dc()))

        except Exception as err:
            print(err)


    def spring_c_n_func(self):
        """
            Springs C_n Function
        """
        try:
            self.lineEdit_sp_c_n.clear()
            n_a = self.lineEdit_sp_n_a.text()

            if n_a != "":
                self.m_eq.sp_c_n_func(float(n_a))
            else:
                self.m_eq.sp_c_n = 1.0

            self.lineEdit_sp_c_n.setText(str(self.m_eq.get_sp_c_n()))

        except Exception as err:
            print(err)


    def spring_c_l_func(self):
        """
            Springs C_l Function
        """
        try:
            self.lineEdit_sp_l1_l2.clear()
            l1_value = self.lineEdit_sp_l_1.text()
            l2_value = self.lineEdit_sp_l_2.text()

            if l1_value != "" and l2_value != "":
                self.m_eq.sp_c_l_func(float(l1_value), float(l2_value))
                self.lineEdit_sp_l1_l2.setText(str(self.m_eq.get_sp_c_l()))
            else:
                self.m_eq.sp_c_l = 1.0
                self.lineEdit_sp_l1_l2.setText(str(self.m_eq.get_sp_c_l()))

        except Exception as err:
            print(err)


    def spring_c_y_func(self):
        """
            Springs C_y Function
        """
        try:
            self.lineEdit_sp_c_y.clear()
            current_material_2 = self.comboBox_sp_material_2.currentText()
            if current_material_2 != "" and current_material_2 != "None":
                c_y = self.mechanical_equipments_dict['Springs']['Material c_y'][str(current_material_2)]

            else:
                c_y = 1.0

            self.m_eq.sp_c_y = float(c_y)
            self.lineEdit_sp_c_y.setText(str(self.m_eq.sp_c_y))

        except Exception as err:
            print(err)


    def spring_c_k_func(self):
        """
            Springs C_k Function
        """
        try:
            self.lineEdit_sp_c_k.clear()

            d_c = self.lineEdit_sp_d_c.text()
            d_w = self.lineEdit_sp_d_w.text()

            if d_c != "" and d_w != "":
                self.m_eq.sp_c_k_func(float(d_c), float(d_w))
            else:
                self.m_eq.sp_c_k = 1.0

            self.lineEdit_sp_c_k.setText(str(self.m_eq.get_sp_c_k()))

        except Exception as err:
            print(err)


    def spring_c_cs_func(self):
        """
            Springs C_cs Function
        """
        try:
            self.lineEdit_sp_c_cs.clear()
            c_r = self.lineEdit_sp_c_r.text()

            if c_r != "":
                self.m_eq.sp_c_cs_func(float(c_r))
                self.m_eq.sp_c_r = float(c_r)

            else:
                self.m_eq.sp_c_cs_func(1.0)
                self.m_eq.sp_c_r = 1.0

            self.lineEdit_sp_c_cs.setText(str(self.m_eq.get_sp_c_cs()))

        except Exception as err:
            print(err)


    def spring_c_m_func(self):
        """
            Springs C_m Function
        """
        try:
            c_m = self.lineEdit_sp_c_m.text()

            if c_m != "":
                self.m_eq.sp_c_m = float(c_m)

            else:
                self.m_eq.sp_c_m = 1.0

        except Exception as err:
            print(err)


    def spring_hazard_rate_func(self):
        """
            Springs Hazard Rate Function
        """
        self.m_eq.lambda_sp_func()

        self.lineEdit_sp_lamda_sp.setText(str(self.m_eq.get_sp_lambda()))


  # GEARS
    def gear_lambda_gr_b_func(self):
        """
            Gears Lambda_gr_b Function
        """
        self.lineEdit_gr_warning_box.clear()
        gr_rpm = self.lineEdit_gr_rpm.text()
        gr_revolutions = self.lineEdit_gr_revolutions.text()

        try:
            if gr_rpm != "" and gr_revolutions != "":
                self.m_eq.gr_lambda_gr_b_func(float(gr_rpm), float(gr_revolutions), True)

            else:
                self.m_eq.gr_lambda_gr_b_func(1, 1, False)

            self.lineEdit_gr_lambda_g_b.setText(str(self.m_eq.get_gr_lambda_gr_b()))

        except ZeroDivisionError:
            self.lineEdit_gr_warning_box.setText("Revolutions 0'dan farklı bir değer olmalı.")

        except Exception as err:
            print(err)


    def gear_c_gs_func(self):
        """
            Gears C_gs Function
        """
        self.lineEdit_gr_warning_box.clear()

        gr_v0 = self.lineEdit_gr_v0.text()
        gr_vd = self.lineEdit_gr_v_d.text()

        try:
            if gr_v0 != "" and gr_vd != "":
                self.m_eq.c_gs_func(float(gr_v0), float(gr_vd), True)

            else:
                self.m_eq.c_gs_func(1, 1, False)

            self.lineEdit_gr_c_gs.setText(str(self.m_eq.get_gr_c_gs()))

        except ZeroDivisionError:
            self.lineEdit_gr_warning_box.setText("Vd 0'dan farklı bir değer olmalı.")

        except Exception as err:
            print(err)


    def gear_c_gp_func(self):
        """
            Gears C_gp Function
        """
        self.lineEdit_gr_warning_box.clear()

        gr_l0 = self.lineEdit_gr_l_0.text()
        gr_ld = self.lineEdit_gr_l_d.text()

        try:
            if gr_l0 != "" and gr_ld != "":
                self.m_eq.c_gp_func(float(gr_l0), float(gr_ld), True)

            else:
                self.m_eq.c_gp_func(1, 1, False)

            self.lineEdit_gr_c_gp.setText(str(self.m_eq.get_gr_c_gp()))

        except ZeroDivisionError:
            self.lineEdit_gr_warning_box.setText("Ld 0'dan farklı bir değer olmalı.")

        except Exception as err:
            print(err)


    def gear_c_ga_func(self):
        """
            Gears C_ga Function
        """
        try:
            self.lineEdit_gr_warning_box.clear()

            gr_ae = self.lineEdit_gr_a_e.text()

            if gr_ae != "":
                self.m_eq.c_ga_func(float(gr_ae), True)

            else:
                self.m_eq.c_ga_func(1, False)

            self.lineEdit_gr_c_ga.setText(str(self.m_eq.get_gr_c_ga()))

        except Exception as err:
            print(err)


    def gear_c_gl_func(self):
        """
            Gears C_gl Function
        """
        self.lineEdit_gr_warning_box.clear()

        gr_v0 = self.lineEdit_gr_v0_4.text()
        gr_vl = self.lineEdit_gr_vl.text()

        try:
            if gr_v0 != "" and gr_vl != "":
                self.m_eq.c_gl_func(float(gr_v0), float(gr_vl), True)

            else:
                self.m_eq.c_gl_func(float(gr_v0), float(gr_vl), False)

            self.lineEdit_gr_c_gl.setText(str(self.m_eq.get_gr_c_gl()))

        except ZeroDivisionError:
            self.lineEdit_gr_warning_box.setText("Vl 0'dan farklı bir değer olmalı.")

        except Exception as err:
            print(err)


    def gear_c_gt_func(self):
        """
            Gears C_gt Function
        """
        try:
            self.lineEdit_gr_warning_box.clear()

            gr_t_at = self.lineEdit_gr_t_at.text()

            if gr_t_at != "":
                self.m_eq.c_gt_func(float(gr_t_at))

            else:
                self.m_eq.c_gt_func(float(1))
                self.lineEdit_gr_c_gt.setText(str(self.m_eq.get_gr_c_gt()))

        except Exception as err:
            print(err)


    def gear_tab_func(self):
        """
            Gears Tab Function
        """
        self.comboBox_gr_prime_mover.clear()
        self.comboBox_gr_load_character.clear()

        self.comboBox_gr_prime_mover.addItem("None")
        self.comboBox_gr_load_character.addItem("None")

        self.comboBox_gr_prime_mover.addItems(self.mechanical_equipments_dict['Gears']['Prime Mover'].keys())

        self.gear_hazard_rate_func()


    def gear_c_gv_func(self):
        """
            Gears C_gv Function
        """
        self.comboBox_gr_load_character.clear()

        current_prime_mover = self.comboBox_gr_prime_mover.currentText()

        if current_prime_mover != "None" and current_prime_mover != "":
            self.comboBox_gr_load_character.addItems(self.mechanical_equipments_dict['Gears']['Prime Mover'][str(current_prime_mover)])
        else:
            self.m_eq.gr_c_gv = float(1.0)
            self.lineEdit_gr_c_gv.setText(str(self.m_eq.gr_c_gv))


    def gear_c_gv_func_2(self):
        """
            Gears C_gv Function 2
        """
        self.lineEdit_gr_c_gv.clear()

        current_prime_mover = self.comboBox_gr_prime_mover.currentText()
        current_char_load = self.comboBox_gr_load_character.currentText()

        if current_char_load != "None" and current_prime_mover != "None" and current_char_load != "" and current_prime_mover != "":
            self.m_eq.gr_c_gv = self.mechanical_equipments_dict['Gears']['Prime Mover'][str(current_prime_mover)][str(current_char_load)]
            self.lineEdit_gr_c_gv.setText(str(self.m_eq.gr_c_gv))

        else:
            self.m_eq.gr_c_gv = 1.0
            self.lineEdit_gr_c_gv.setText(str(self.m_eq.gr_c_gv))


    def gear_hazard_rate_func(self):
        """
            Gears Hazard Rate Function
        """
        self.m_eq.lambda_gr_func()

        self.lineEdit_gr_lamda_g.setText(str(self.m_eq.get_gr_lambda()))
        self.lineEdit_em_gr_lamda_g.setText(str(self.m_eq.get_gr_lambda()))
        self.lineEdit_cp_gr_lamda_g.setText(str(self.m_eq.get_gr_lambda()))


  # BEARINGS

    def bearing_lambda_be_b_and_be_c_y_func(self):
        """
            Bearings Lambda_be_b and Be_c_y Function
        """
        self.lineEdit_be_warning_box.clear()

        be_l_s = str(self.lineEdit_be_l_s.text())
        be_l_a = str(self.lineEdit_be_l_a.text())
        be_n = str(self.lineEdit_be_n.text())

        if self.radioButton_be_ball_bearing.isChecked():
            self.y = float(3)
        elif self.radioButton_be_roller_bearing.isChecked():
            self.y = float(3.3)

        try:
            if be_l_s != "" and be_l_a != "":
                self.m_eq.be_c_y_func(float(self.y), float(be_l_a), float(be_l_s))
                self.lineEdit_be_c_y.setText(str(self.m_eq.get_be_c_y()))

                if be_n != "":
                    self.m_eq.be_lambda_be_b_func(float(self.y), float(be_l_s), float(be_l_a), float(be_n), 1)
                    self.lineEdit_be_lambda_be_b.setText(str(self.m_eq.get_be_lambda_be_b()))

                else:
                    self.m_eq.be_lambda_be_b_func(float(self.y), float(be_l_s), float(be_l_a), float(1), 1)
                    self.lineEdit_be_lambda_be_b.setText(str(self.m_eq.get_be_lambda_be_b()))

            else:
                self.m_eq.be_c_y_func(float(1), float(1), float(1))
                self.lineEdit_be_c_y.setText(str(self.m_eq.get_be_c_y()))

                self.m_eq.be_lambda_be_b_func(1, 1, 1, None, 1)
                self.lineEdit_be_lambda_be_b.setText(str(self.m_eq.get_be_lambda_be_b()))


        except ZeroDivisionError:
            self.lineEdit_be_warning_box.setText("LS, LA ve n 0'dan farklı bir değer olmalı.")

        except TypeError:
            pass

        except ValueError:
            pass

        except Exception as err:
            print(err)


    def bearing_c_r_func(self):
        """
            Bearings C_r Function
        """
        try:
            self.lineEdit_be_warning_box.clear()
            self.lineEdit_be_c_r.clear()

            be_r = str(self.lineEdit_be_r.text())

            if be_r != "":
                self.m_eq.be_c_r_func(float(be_r), True)

            else:
                self.m_eq.be_c_r_func(1, False)

            self.lineEdit_be_c_r.setText(str(self.m_eq.get_be_c_r()))

        except ZeroDivisionError:
            self.lineEdit_be_warning_box.setText("R 0 ve 100 arasında bir değer olmalı.")
        except TypeError:
            self.lineEdit_be_warning_box.setText("R 0 ve 100 arasında bir değer olmalı.")

        except Exception as err:
            print(err)


    def bearing_c_v_func(self):
        """
            Bearings C_v Function
        """
        try:
            self.lineEdit_be_warning_box.clear()
            self.lineEdit_be_c_v.clear()

            be_v_0 = self.lineEdit_be_v0.text()
            be_v_l = self.lineEdit_be_vl.text()

            if be_v_0 != "" and be_v_l != "":
                self.m_eq.be_c_v_func(float(be_v_0), float(be_v_l), True)

            else:
                self.m_eq.be_c_v_func(1, 1, False)

            self.lineEdit_be_c_v.setText(str(self.m_eq.get_be_c_v()))

        except ZeroDivisionError:
            self.lineEdit_be_warning_box.setText("VL 0'dan farklı bir değer olmalı.")

        except Exception as err:
            print(err)


    def bearing_c_cw_func(self):
        """
            Bearings C_cw Function
        """
        try:
            self.lineEdit_be_warning_box.clear()
            self.lineEdit_be_c_cw.clear()

            be_c_w = self.lineEdit_be_cw.text()

            if be_c_w != "":
                self.m_eq.be_c_cw_func(float(be_c_w), True)

            else:
                self.m_eq.be_c_cw_func(1, False)

            self.lineEdit_be_c_cw.setText(str(self.m_eq.get_be_c_cw()))

        except ValueError:
            self.lineEdit_be_warning_box.setText(str(ValueError))

        except Exception as err:
            print(err)


    def bearing_c_t_func(self):
        """
            Bearings C_t Function
        """
        try:
            self.lineEdit_be_warning_box.clear()
            self.lineEdit_be_c_t.clear()

            be_t0 = self.lineEdit_be_t0.text()

            if be_t0 != "":
                self.m_eq.be_c_t_func(float(be_t0))

            else:
                self.m_eq.be_c_t_func(1)

            self.lineEdit_be_c_t.setText(str(self.m_eq.get_be_c_t()))

        except ValueError:
            self.lineEdit_be_warning_box.setText(str(ValueError))

        except Exception as err:
            print(err)


    def bearing_tab_func(self):
        """
            Bearings Tab Function
        """
        self.comboBox_be_type_of_application.clear()
        self.comboBox_be_type_of_application_2.clear()

        self.comboBox_be_type_of_application.addItem("None")
        self.comboBox_be_type_of_application_2.addItem("None")

        self.comboBox_be_type_of_application.addItems(self.mechanical_equipments_dict['Bearings']['Bearing Service Factors'].keys())
        self.comboBox_be_type_of_application_2.addItems(self.mechanical_equipments_dict['Bearings']['Bearing Contamination Level'].keys())

        self.bearing_hazard_rate_func()


    def bearing_c_sf_func(self):
        """
            Bearings C_sf Function
        """
        self.lineEdit_be_c_sf.clear()
        self.lineEdit_be_warning_box.clear()
        current_type = self.comboBox_be_type_of_application.currentText()

        if current_type != "None":

            if self.radioButton_be_ball_bearing.isChecked():
                try:
                    self.m_eq.be_c_sf = self.mechanical_equipments_dict['Bearings']['Bearing Service Factors'][str(current_type)]['Ball bearing']
                    self.lineEdit_be_c_sf.setText(str(self.m_eq.be_c_sf))
                except KeyError:
                    self.lineEdit_be_warning_box.setText('{} has no {}.'.format(str(current_type), 'Ball bearing'))

            elif self.radioButton_be_roller_bearing.isChecked():
                try:
                    self.m_eq.be_c_sf = self.mechanical_equipments_dict['Bearings']['Bearing Service Factors'][str(current_type)]['Roller bearing']
                    self.lineEdit_be_c_sf.setText(str(self.m_eq.be_c_sf))
                except KeyError:
                    self.lineEdit_be_warning_box.setText('{} has no {}.'.format(str(current_type), 'Roller bearing'))

        else:
            self.m_eq.be_c_sf = 1.0
            self.lineEdit_be_c_sf.setText(str(self.m_eq.be_c_sf))


    def bearing_c_c_func(self):
        """
            Bearings C_c Function
        """
        self.lineEdit_be_warning_box.clear()
        current_type = self.comboBox_be_type_of_application_2.currentText()
        bearing_diameter = self.lineEdit_be_bearing_diameter.text()

        if current_type != "None":
            if bearing_diameter != "":
                if float(bearing_diameter) < 100:
                    self.m_eq.be_c_c = self.mechanical_equipments_dict['Bearings']['Bearing Contamination Level'][str(current_type)]['s_bearing_diameter_100_mm']
                    self.lineEdit_be_c_c.setText(str(self.m_eq.be_c_c))
                else:
                    self.m_eq.be_c_c = self.mechanical_equipments_dict['Bearings']['Bearing Contamination Level'][str(current_type)]['g_bearing_diameter_100_mm']
                    self.lineEdit_be_c_c.setText(str(self.m_eq.be_c_c))
        else:
            self.m_eq.be_c_c = 1.0
            self.lineEdit_be_c_c.setText(str(self.m_eq.be_c_c))


    def bearing_hazard_rate_func(self):
        """
            Bearings Hazard Rate Function
        """
        self.m_eq.lambda_be_func()

        self.lineEdit_be_lambda_be.setText(str(self.m_eq.get_be_lambda()))
        self.lineEdit_em_be_lambda_be.setText(str(self.m_eq.get_be_lambda()))


  # ACTUATORS
    def actuator_tab_func(self):
        """
            Actuators Tab Function
        """
        self.comboBox_ac_various_metals.clear()
        self.comboBox_ac_materials.clear()

        self.comboBox_ac_various_metals.addItem("None")
        self.comboBox_ac_materials.addItem("None")

        self.comboBox_ac_various_metals.addItems(self.mechanical_equipments_dict["Actuators"]["Yield strengths"].keys())
        self.comboBox_ac_materials.addItems(self.mechanical_equipments_dict["Actuators"]["Material hardness"].keys())

        self.actuator_hazard_rate_func()


    def actuator_tab_func_2(self):
        """
            Actuators Tab Function 2
        """
        self.comboBox_material_types.clear()
        self.comboBox_material_types.addItem("None")

        current_material = self.comboBox_ac_materials.currentText()
        if current_material != "None" and current_material != "":
            self.comboBox_material_types.addItems(self.mechanical_equipments_dict["Actuators"]["Material hardness"][str(current_material)].keys())


    def actuatior_gama_func(self):
        """
            Actuators Gama Function
        """
        if self.radioButton_ac_gama_0_2.isChecked():
            self.m_eq.ac_gama = float(0.2)
        elif self.radioButton_ac_gama_0_54.isChecked():
            self.m_eq.ac_gama = float(0.54)
        else:
            self.m_eq.ac_gama = float(1.0)


    def actuator_f_y_func(self):
        """
            Actuators F_y Function
        """
        self.lineEdit_ac_f_y.clear()
        metal = self.comboBox_ac_various_metals.currentText()
        if metal != "None":
            self.m_eq.ac_f_y = self.mechanical_equipments_dict["Actuators"]["Yield strengths"][str(metal)]
        else:
            self.m_eq.ac_f_y = 1.0
        self.lineEdit_ac_f_y.setText(str(self.m_eq.ac_f_y))


    def actuator_n_o_func(self):
        """
            Actuators N_o Function
        """
        self.lineEdit_ac_n_o.clear()

        w_a = self.lineEdit_ac_w_a.text()
        d_1 = self.lineEdit_ac_d_1.text()
        d_2 = self.lineEdit_ac_d_2.text()
        mu_1 = self.lineEdit_ac_mu_1.text()
        mu_2 = self.lineEdit_ac_mu_2.text()
        e_1 = self.lineEdit_ac_e_1.text()
        e_2 = self.lineEdit_ac_e_2.text()

        try:
            if (w_a != "") and (d_1 != "") and (d_2 != "") and (mu_1 != "") and (mu_2 != "") and (e_1 != "") and  (e_2 != ""):
                self.m_eq.ac_n_o_func(float(w_a), float(d_1), float(d_2), float(mu_1), float(mu_2), float(e_1), float(e_2))
            else:
                self.m_eq.ac_n_o = float(1.0)

            self.lineEdit_ac_n_o.setText(str(self.m_eq.ac_n_o))

        except ZeroDivisionError as err:
            self.lineEdit_ac_warning_box.setText(str(err))

        except Exception as err:
            print(err)


    def actuator_lambda_ac_b_func(self):
        """
            Actuators Lambda_ac_b Function
        """
        self.lineEdit_ac_lambda_ac_b.clear()
        if self.lineEdit_ac_n_o.text() != "":
            self.m_eq.ac_lambda_ac_b_func()
            self.lineEdit_ac_lambda_ac_b.setText(str(self.m_eq.get_ac_lambda_ac_b()))


    def actuator_piston_cylinder_rad_but_control(self):
        """
            Actuators Piston Cylinder Function
        """
        if self.radioButton_ac_cylinder.isChecked():
            self.lineEdit_ac_h_p.setDisabled(True)
            self.label_ac_h_p.setDisabled(True)
            self.lineEdit_ac_h_c.setDisabled(False)
            self.label_ac_h_c.setDisabled(False)

        elif self.radioButton_ac_piston.isChecked():
            self.lineEdit_ac_h_p.setDisabled(False)
            self.label_ac_h_p.setDisabled(False)
            self.lineEdit_ac_h_c.setDisabled(True)
            self.label_ac_h_c.setDisabled(True)


    def actutor_hardness_func(self):
        """
            Actuators Hardness Function
        """
        val = str()
        current_material = self.comboBox_ac_materials.currentText()
        if current_material != "None" and current_material != "":
            current_material_type = self.comboBox_material_types.currentText()
            if current_material_type != "None" and current_material_type != "":
                val = self.mechanical_equipments_dict["Actuators"]["Material hardness"][str(current_material)][str(current_material_type)]

        if self.radioButton_ac_cylinder.isChecked():
            self.lineEdit_ac_h_c.setText(str(val))
        elif self.radioButton_ac_piston.isChecked():
            self.lineEdit_ac_h_p.setText(str(val))


    def actuator_c_h_func(self):
        """
            Actuators C_h Function
        """
        try:
            h_p = self.lineEdit_ac_h_p.text()
            h_c = self.lineEdit_ac_h_c.text()

            if h_c != "" and h_p != "":
                self.m_eq.ac_c_h_func(float(h_p), float(h_c), True)

            else:
                self.m_eq.ac_c_h_func(1, 1, False)

        except Exception as err:
            print(err)


    def actuator_c_s_func(self):
        """
            Actuators C_s Function
        """
        try:
            self.lineEdit_ac_c_s.clear()
            filter_size = self.lineEdit_ac_filter_size.text()
            if filter_size != "":
                self.m_eq.ac_c_s_func(float(filter_size))

            else:
                self.m_eq.ac_c_s_func(10)

            self.lineEdit_ac_c_s.setText(str(self.m_eq.ac_c_s))

        except Exception as err:
            print(err)


    def actuator_c_cp_func(self):
        """
            Actuators C_cp Function
        """
        try:
            h_p = self.lineEdit_ac_h_p.text()
            h_c = self.lineEdit_ac_h_c.text()
            c_s = self.lineEdit_ac_c_s.text()

            if h_c != "" and h_p != "" and c_s != "":
                self.m_eq.ac_c_cp_func(True)

            else:
                self.m_eq.ac_c_cp_func(False)

            self.lineEdit_ac_c_cp.setText(str(self.m_eq.get_ac_c_cp()))

        except Exception as err:
            print(err)


    def actuator_c_t_func(self):
        """
            Actuators C_t Function
        """
        self.lineEdit_ac_c_t.setText(str(1.0))


    def actuator_hazard_rate_func(self):
        """
            Actuators Hazard Rate Function
        """
        self.m_eq.lambda_ac_func()

        self.lineEdit_ac_lambda_ac.setText(str(self.m_eq.get_ac_lambda()))


  # SHAFT

    def shaft_lambda_s_h_b_func(self):
        """
            Shafts Sh_b Function
        """
        self.lineEdit_sh_lambda_sh_b.clear()
        self.lineEdit_sh_warning_box.clear()

        sh_n = self.lineEdit_sh_n.text()

        try:
            if sh_n != "":
                self.m_eq.sh_lambda_sh_b_func(float(sh_n))

            else:
                self.m_eq.sh_lambda_sh_b_func(1)

            self.lineEdit_sh_lambda_sh_b.setText(str(self.m_eq.get_sh_lambda_sh_b()))

        except ZeroDivisionError:
            self.lineEdit_sh_warning_box.setText("N 0'dan farklı olmalı.")

        except Exception as err:
            print(err)

        self.shaft_hazard_rate_func()

    def shaft_tab_func(self):
        """
            Shafts Tab Function
        """
        self.comboBox_sh_surface_finish.clear()
        self.comboBox_sh_surface_finish.addItem("None")
        self.comboBox_sh_surface_finish.addItems(list(self.mechanical_equipments_dict['Shafts']['Shaft surface finish factor'].keys()))

        self.comboBox_sh_h_D.clear()
        self.comboBox_sh_h_r.clear()
        self.comboBox_sh_h_D.addItem("None")
        self.comboBox_sh_h_r.addItem("None")

        hd_list = list(self.mechanical_equipments_dict['Shafts']['Stress concentration factor'].keys())
        for item in hd_list:
            temp_hd = item.split('h_D_')
            self.comboBox_sh_h_D.addItem(temp_hd[1].replace('_', '.'))

        self.comboBox_sh_e.clear()
        self.comboBox_sh_e.addItem("None")
        self.comboBox_sh_e.addItems(list(self.mechanical_equipments_dict['Shafts']['Shaft material strength'].keys()))

        self.comboBox_sh_b.clear()
        self.comboBox_sh_b.addItem("None")
        self.comboBox_sh_b.addItems(list(self.mechanical_equipments_dict['Shafts']['Allowable shaft bending'].keys()))

        self.comboBox_sh_sections.clear()
        self.comboBox_sh_sections.addItem("None")

        self.shaft_hazard_rate_func()


    def shaft_c_f_func(self):
        """
            Shafts C_f Function
        """
        self.lineEdit_sh_warning_box.clear()
        self.lineEdit_sh_c_f.clear()

        current_finish = self.comboBox_sh_surface_finish.currentText()
        t_s = self.lineEdit_sh_t_s.text()

        try:
            if t_s != "":
                if current_finish == 'None':
                    select = 9
                    temp = 1.0

                elif current_finish == 'Hot Rolled':
                    select = 1
                    temp = t_s

                elif current_finish == 'Machined or Cold Drawn':
                    select = 2
                    temp = t_s

                elif current_finish == 'Forged':
                    select = 3
                    temp = t_s

                else:
                    select = 0
                    temp = float(self.mechanical_equipments_dict['Shafts']['Shaft surface finish factor'][str(current_finish)])

                self.m_eq.sh_c_f_func(temp, select)
                self.lineEdit_sh_c_f.setText(str(self.m_eq.get_sh_c_f()))

            else:
                if current_finish == "None":
                    self.m_eq.sh_c_f_func(0, 9)

                else:
                    temp = float(self.mechanical_equipments_dict['Shafts']['Shaft surface finish factor'][str(current_finish)])
                    self.m_eq.sh_c_f_func(temp, 0)

                self.lineEdit_sh_c_f.setText(str(self.m_eq.get_sh_c_f()))

        except Exception as err:
            print(err)

        self.shaft_hazard_rate_func()


    def shaft_c_t_func(self):
        """
            Shafts C_t Function
        """
        self.lineEdit_sh_warning_box.clear()
        self.lineEdit_sh_c_t.clear()

        t_at = self.lineEdit_sh_t_at.text()

        try:
            if t_at != "":
                self.m_eq.sh_c_t_func(float(t_at))

            else:
                self.m_eq.sh_c_t_func(1)

            self.lineEdit_sh_c_t.setText(str(self.m_eq.get_sh_c_t()))

        except Exception as err:
            print(err)

        self.shaft_hazard_rate_func()


    def shaft_c_sc_r_func(self):
        """
            Shafts C_sc_r Function
        """
        self.lineEdit_sh_warning_box.clear()
        self.lineEdit_sh_c_sc_r.clear()

        r_value = self.lineEdit_sh_r.text()
        b_d = self.lineEdit_sh_D.text()
        s_d = self.lineEdit_sh_d.text()

        try:
            if r_value != "" and b_d != "" and s_d != "":
                self.m_eq.sh_c_sc_r_func(float(r_value), float(b_d), float(s_d), True)

            else:
                self.m_eq.sh_c_sc_r_func(1, 1, 1, False)

            self.lineEdit_sh_c_sc_r.setText(str(self.m_eq.get_sh_c_sc_r()))

        except Exception as err:
            print(err)

        self.shaft_hazard_rate_func()


    def shaft_c_sc_g_func(self):
        """
            Shafts C_sc_g Function
        """
        self.comboBox_sh_h_r.clear()
        self.lineEdit_sh_c_sc_g.clear()

        current_h_d = self.comboBox_sh_h_D.currentText()

        if current_h_d != "None":
            temp_h_d = 'h_D_' + str(current_h_d.replace('.', '_'))
            csf_list = list(self.mechanical_equipments_dict['Shafts']['Stress concentration factor'][str(temp_h_d)].keys())

            for item in csf_list:
                temp_csf = item.split('h_r_')
                self.comboBox_sh_h_r.addItem(temp_csf[1].replace('_', '.'))
        else:
            self.m_eq.sh_c_sc_g = 0.0
            self.lineEdit_sh_c_sc_g.setText(str(self.m_eq.get_sh_c_sc_g()))

        self.shaft_hazard_rate_func()


    def shaft_c_sc_g_func_2(self):
        """
            Shafts C_sc_g Function 2
        """
        self.lineEdit_sh_c_sc_g.clear()

        current_h_d = self.comboBox_sh_h_D.currentText()
        current_h_r = self.comboBox_sh_h_r.currentText()

        if current_h_d != "None" and current_h_r != "None" and current_h_d != "" and current_h_r != "":
            temp_h_r = str('h_r_' + str(current_h_r.replace('.', '_')))
            temp_h_d = str('h_D_' + str(current_h_d.replace('.', '_')))

            self.m_eq.sh_c_sc_g = self.mechanical_equipments_dict['Shafts']['Stress concentration factor'][str(temp_h_d)][str(temp_h_r)]

        else:
            self.m_eq.sh_c_sc_g = 0.0

        self.lineEdit_sh_c_sc_g.setText(str(self.m_eq.get_sh_c_sc_g()))

        self.shaft_hazard_rate_func()

    def shaft_c_sc_func(self):
        """
            Shafts C_sc Function
        """
        self.lineEdit_sh_c_sc.clear()

        self.m_eq.sh_c_sc_func()
        self.lineEdit_sh_c_sc.setText(str(self.m_eq.get_sh_c_sc()))

        self.shaft_hazard_rate_func()


    def shaft_sh_e_control(self):
        """
            Shafts Sh_e Control Function
        """
        e_value = self.comboBox_sh_e.currentText()
        if e_value != "None" and e_value != "":
            self.lineEdit_sh_e.setText(str(self.mechanical_equipments_dict['Shafts']['Shaft material strength'][str(e_value)]))
        else:
            self.lineEdit_sh_e.setText(str(1))


    def shaft_sh_b_control(self):
        """
            Shafts Sh_b Control Function
        """
        b_value = self.comboBox_sh_b.currentText()
        if b_value != "None" and b_value != "":
            self.lineEdit_sh_b.setText(str(self.mechanical_equipments_dict['Shafts']['Allowable shaft bending'][str(b_value)]))
        else:
            self.lineEdit_sh_b.setText(str(0.007))


    def shaft_sh_add_section_control(self):
        """
            Shafts Sh Add Section Control Function
        """
        self.comboBox_sh_sections.clear()
        section = self.lineEdit_sh_add_section.text()

        if section != "":
            self.shaft_sections_dict[str(section)] = {'I': 'None', 'length': 'None'}

        self.comboBox_sh_sections.addItems(self.shaft_sections_dict.keys())


    def shaft_sh_set_section_control(self):
        """
            Shafts Sh Set Section Control Function
        """
        inertia = self.lineEdit_sh_section_i.text()
        length = self.lineEdit_section_length.text()

        if inertia != "" and length != "":
            current_section = self.comboBox_sh_sections.currentText()

            self.shaft_sections_dict[str(current_section)]['I'] = inertia
            self.shaft_sections_dict[str(current_section)]['length'] = length

            self.label_sh_section_i_val.setText(str(inertia))
            self.label_sh_show_section_length_val.setText(str(length))

        self.lineEdit_sh_section_i.clear()
        self.lineEdit_section_length.clear()


    def shaft_sh_sections_show_values(self):
        """
            Shafts Sh Section Show Values Function
        """
        current_section = self.comboBox_sh_sections.currentText()

        if current_section != "":
            inertia = self.shaft_sections_dict[str(current_section)]['I']
            length = self.shaft_sections_dict[str(current_section)]['length']

            self.label_sh_section_i_val.setText(str(inertia))
            self.label_sh_show_section_length_val.setText(str(length))


    def shaft_sh_c_dy_func(self):
        """
            Shafts C_dy Function
        """
        try:
            e_value = self.lineEdit_sh_e.text()
            f_value = self.lineEdit_sh_f.text()
            b_value = self.lineEdit_sh_b.text()

            new_dict = dict(self.shaft_sections_dict)

            for item in list(new_dict.keys()):
                inertia = new_dict[str(item)]['I']
                length = new_dict[str(item)]['length']

                if inertia == "None" and length == "None":
                    del new_dict[str(item)]

                else:
                    self.m_eq.sh_c_dy_func(float(e_value), float(f_value), float(b_value), new_dict)
                    self.lineEdit_sh_c_dy.setText(str(self.m_eq.get_sh_c_dy()))

        except Exception as err:
            print(err)

        self.shaft_hazard_rate_func()

    def shaft_hazard_rate_func(self):
        """
            Shafts Hazard Rate Function
        """
        self.m_eq.lambda_sh_func()

        self.lineEdit_sh_lambda_sh.setText(str(self.m_eq.get_sh_lambda()))
        self.lineEdit_em_sh_lambda_sh.setText(str(self.m_eq.get_sh_lambda()))


  # ELECTRIC MOTORS

    def em_tab_func(self):
        """
            Electric Motors Tab Function
        """
        self.comboBox_em_motor_type.clear()
        self.comboBox_em_motor_type.addItem("None")
        self.comboBox_em_motor_type.addItems(list(self.mechanical_equipments_dict['Electric Motors']['Type of Motor'].keys()))

        self.comboBox_em_load_type.clear()
        self.comboBox_em_load_type.addItem("None")
        self.comboBox_em_load_type.addItems(list(self.mechanical_equipments_dict['Electric Motors']['Load Type'].keys()))

        self.comboBox_em_insulation_class.clear()
        self.comboBox_em_insulation_class.addItem("None")
        self.comboBox_em_insulation_class.addItems(list(self.mechanical_equipments_dict['Electric Motors']['Insulation Class'].keys()))

        self.em_hazard_rate_func()


    def em_lambda_m_b_func(self):
        """
            Electric Motors Lambda_m_b Function
        """
        motor_type = self.comboBox_em_motor_type.currentText()

        if motor_type != "None" and motor_type != "":
            self.m_eq.lambda_m_b = float(self.mechanical_equipments_dict['Electric Motors']['Type of Motor'][str(motor_type)])

        else:
            self.m_eq.lambda_m_b = float(1.0)

        self.lineEdit_em_lambda_mb.setText(str(self.m_eq.lambda_m_b))


    def em_c_s_f_func(self):
        """
            Electric Motors C_s_f Function
        """
        load_type = self.comboBox_em_load_type.currentText()

        if load_type != "None" and load_type != "":
            self.m_eq.c_sf = float(self.mechanical_equipments_dict['Electric Motors']['Load Type'][str(load_type)])

        else:
            self.m_eq.c_sf = float(1.0)

        self.lineEdit_em_c_sf.setText(str(self.m_eq.c_sf))


    def em_lambda_wi_b_func(self):
        """
            Electric Motors Lambda_wi_b Function
        """
        try:
            l_i = self.lineEdit_em_li.text()

            if l_i != "" and l_i != "None":
                self.m_eq.elec_mot_lambda_wi_b_func(float(l_i))

            else:
                self.m_eq.elec_mot_lambda_wi_b_func(float(1.0))

            self.lineEdit_em_lambda_wi_b.setText(str(self.m_eq.get_em_lambda_wi_b()))

        except Exception as err:
            print(err)

        self.em_lambda_wi_func()

    def em_c_t_func(self):
        """
            Electric Motors C_t Function
        """
        ins_class = self.comboBox_em_insulation_class.currentText()

        if ins_class != "" and ins_class != "None":
            self.label_em_temp_rating.setText(str(self.mechanical_equipments_dict['Electric Motors']['Insulation Class'][str(ins_class)]['Temperature rating']) + "°C")
            self.label_em_assumed_ambient_temp.setText(str(self.mechanical_equipments_dict['Electric Motors']['Insulation Class'][str(ins_class)]['Assumed ambient temperature']) + "°C")
            self.label_allowable_temp_rise.setText(str(self.mechanical_equipments_dict['Electric Motors']['Insulation Class'][str(ins_class)]['Allowable temperature rise']) + "°C")
            self.label_hot_spot_allow.setText(str(self.mechanical_equipments_dict['Electric Motors']['Insulation Class'][str(ins_class)]['Hot spot allowance']) + "°C")

        else:
            self.label_em_temp_rating.clear()
            self.label_em_assumed_ambient_temp.clear()
            self.label_allowable_temp_rise.clear()
            self.label_hot_spot_allow.clear()

        try:
            t_0 = self.lineEdit_em_t_0.text()

            if t_0 != "":
                self.m_eq.elec_mot_c_t_func(float(t_0))

            else:
                self.m_eq.elec_mot_c_t_func(str(None))

            self.lineEdit_em_c_t.setText(str(self.m_eq.get_em_c_t()))

        except Exception as err:
            print(err)

        self.em_lambda_wi_func()

    def em_v_u_func(self):
        """
            Electric Motors V_u Function
        """
        self.lineEdit_em_v_u.clear()
        self.lineEdit_em_warning_box.clear()

        gvd = self.lineEdit_em_greatest_volt_diff.text()
        apv = self.lineEdit_em_average_phase_volt.text()

        try:
            if gvd != "" and apv != "":
                self.m_eq.elec_mot_v_u_func(float(gvd), float(apv), True)

            else:
                self.m_eq.elec_mot_v_u_func(1, 1, False)

            self.lineEdit_em_v_u.setText(str(self.m_eq.get_em_v_u()))

        except ZeroDivisionError:
            self.lineEdit_em_warning_box.setText("Average phase voltage cannot be zero.")

        except Exception as err:
            print(err)

        self.em_lambda_wi_func()


    def em_c_v_func(self):
        """
            Electric Motors C_v Function
        """
        self.lineEdit_em_c_v.clear()
        self.lineEdit_em_warning_box.clear()

        try:
            v_d = self.lineEdit_em_v_d.text()
            v_r = self.lineEdit_em_v_r.text()
            v_u = self.lineEdit_em_v_u.text()

            motor_type = self.comboBox_em_motor_type.currentText()

            if motor_type != "None" and v_d != "" and v_r != "" and v_u != "":
                if motor_type == 'AC Polyphase':
                    self.m_eq.em_motor_phase = 3

                else:
                    self.m_eq.em_motor_phase = 1

                self.m_eq.elec_mot_c_v_func(v_d, v_r, v_u, True)

            else:
                self.m_eq.elec_mot_c_v_func(1, 1, 1, False)

            self.lineEdit_em_c_v.setText(str(self.m_eq.get_em_c_v()))

        except ZeroDivisionError:
            self.lineEdit_em_warning_box.setText("VR değeri 0 olamaz.")

        except Exception as err:
            print(err)

        self.em_lambda_wi_func()


    def em_c_alt_func(self):
        """
            Electric Motors C_alt Function
        """
        self.lineEdit_em_c_alt.clear()
        self.lineEdit_em_warning_box.clear()

        try:
            altitude = self.lineEdit_em_operating_altitude.text()

            if altitude != "":
                self.m_eq.elec_mot_alt_func(float(altitude))
                self.lineEdit_em_c_alt.setText(str(self.m_eq.get_em_c_alt()))

        except Exception as err:
            print(err)

        self.em_lambda_wi_func()


    def em_lambda_wi_func(self):
        """
            Electric Motors Lambda_wi Function
        """
        self.m_eq.elec_mot_lambda_wi_func()
        self.lineEdit_em_lambda_wi.setText(str(self.m_eq.get_em_lambda_wi_func()))


    def em_lambda_c_func(self):
        """
            Electric Motors Lambda_c Function
        """
        self.lineEdit_em_warning_box.clear()
        em_lambda_c = self.lineEdit_em_lambda_c.text()
        try:
            if em_lambda_c != "":
                self.m_eq.elec_mot_lambda_c_func(float(em_lambda_c))

            else:
                self.m_eq.elec_mot_lambda_c_func(float(0.0))

        except ValueError as e:
            self.lineEdit_em_warning_box.setText(str(e))

        except Exception as err:
            print(err)


    def em_hazard_rate_func(self):
        """
            Electric Motors Hazard Rate Function
        """
        self.m_eq.electric_motor_system_failure_rate_func()

        self.lineEdit_em_lambda_m.setText(str(self.m_eq.get_em_lambda()))


  # MECHANICAL COUPLINGS
    def couplings_tab_func(self):
        """
            Mechanical Couplings Tab Function
        """
        self.comboBox_cp_driven_machinery.clear()
        self.comboBox_cp_driven_machinery.addItem("None")
        self.comboBox_cp_driven_machinery.addItems(self.mechanical_equipments_dict["Mechanical Couplings"]["Driven Machinery"].keys())

        self.couplings_hazard_rate_func()


    def couplings_tab_func_2(self):
        """
            Mechanical Couplings Tab Function 2
        """
        self.comboBox_cp_torque.clear()
        current_mach = self.comboBox_cp_driven_machinery.currentText()

        if current_mach != "None" and current_mach != "":
            self.comboBox_cp_torque.addItems(self.mechanical_equipments_dict["Mechanical Couplings"]["Driven Machinery"][str(current_mach)].keys())

        else:
            self.m_eq.cp_c_sf = float(1.0)
            self.lineEdit_cp_c_sf.setText(str(self.m_eq.cp_c_sf))


    def couplings_c_sf_func(self):
        """
            Mechanical Couplings C_sf Function
        """
        current_mach = self.comboBox_cp_driven_machinery.currentText()
        current_torque = self.comboBox_cp_torque.currentText()

        if current_mach != "None" and current_mach != "" and current_torque != "None" and current_torque != "":
            self.m_eq.cp_c_sf = self.mechanical_equipments_dict["Mechanical Couplings"]["Driven Machinery"][str(current_mach)][str(current_torque)]

        else:
            self.m_eq.cp_c_sf = float(1.0)

        self.lineEdit_cp_c_sf.setText(str(self.m_eq.cp_c_sf))


    def couplings_hazard_rate_func(self):
        """
            Mechanical Couplings Hazard Rate Function
        """
        self.m_eq.lambda_cp_func()

        self.lineEdit_cp_lambda_cp.setText(str(self.m_eq.get_cp_lambda()))


  # BATTERY
    def bat_tab_func(self):
        """
            Battery Tab Function
        """
        self.lineEdit_bat_lambda_0.clear()
        self.comboBox_bat_device_type.clear()
        self.comboBox_bat_device_type.addItem("None")
        self.comboBox_bat_device_type.addItems(self.mechanical_equipments_dict["Battery"]["Device type"].keys())

        self.bat_hazard_rate_func()


    def bat_tab_func_2(self):
        """
            Battery Tab Function 2
        """
        self.comboBox_bat_device_type_2.clear()
        current_dev_type = self.comboBox_bat_device_type.currentText()

        if current_dev_type != "" and current_dev_type != "None":

            val = self.mechanical_equipments_dict["Battery"]["Device type"][str(current_dev_type)]

            if type(val) is int:
                self.m_eq.bat_lambda_0 = float(val)
                self.lineEdit_bat_lambda_0.setText(str(val))

            else:
                self.comboBox_bat_device_type_2.addItems(val.keys())

        else:
            self.m_eq.bat_lambda_0 = float(1.0)
            self.lineEdit_bat_lambda_0.setText(str(self.m_eq.bat_lambda_0))


    def bat_tab_func_3(self):
        """
            Battery Tab Function 3
        """
        current_dev_type = self.comboBox_bat_device_type.currentText()
        current_dev_type_2 = self.comboBox_bat_device_type_2.currentText()

        if current_dev_type_2 != "" and current_dev_type_2 != "None":
            val = self.mechanical_equipments_dict["Battery"]["Device type"][str(current_dev_type)][str(current_dev_type_2)]

            if type(val) is int:
                self.m_eq.bat_lambda_0 = float(val)
                self.lineEdit_bat_lambda_0.setText(str(val))

        else:
            self.m_eq.bat_lambda_0 = float(1.0)
            self.lineEdit_bat_lambda_0.setText(str(self.m_eq.bat_lambda_0))


    def bat_hazard_rate_func(self):
        """
            Battery Hazard Rate Function
        """
        self.m_eq.lambda_bat_func()

        self.lineEdit_bat_lambda_bat.setText(str(self.m_eq.get_bat_lambda()))


# "CONFIGURATION // MECHANICAL EQUIPMENTS FUNCTIONS" - END -
