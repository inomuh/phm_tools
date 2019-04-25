#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'PHM.ui'
#
# Created by: PyQt5 UI code generator 5.5.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets
# from PyQt5.QtWidgets import QLabel, QVBoxLayout
from PyQt5.QtGui import QPixmap
import rospy
import sys
import os


class PhotoViewer(QtWidgets.QGraphicsView):
    photoClicked = QtCore.pyqtSignal(QtCore.QPoint)

    def __init__(self, parent):
        super(PhotoViewer, self).__init__(parent)
        self._zoom = 0
        self._empty = True
        self._scene = QtWidgets.QGraphicsScene(self)
        self._photo = QtWidgets.QGraphicsPixmapItem()
        self._scene.addItem(self._photo)
        self.setScene(self._scene)
        self.setTransformationAnchor(QtWidgets.QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QtWidgets.QGraphicsView.AnchorUnderMouse)
        self.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.setBackgroundBrush(QtGui.QBrush(QtGui.QColor(255, 255, 255)))
        self.setFrameShape(QtWidgets.QFrame.NoFrame)

    def has_photo(self):
        return not self._empty

    def fit_in_view(self, scale=True):
        rect = QtCore.QRectF(self._photo.pixmap().rect())
        if not rect.isNull():
            self.setSceneRect(rect)
            if self.has_photo():
                unity = self.transform().mapRect(QtCore.QRectF(0, 0, 1, 1))
                self.scale(1 / unity.width(), 1 / unity.height())
                viewrect = self.viewport().rect()
                scenerect = self.transform().mapRect(rect)
                factor = min(viewrect.width() / scenerect.width(),
                             viewrect.height() / scenerect.height())
                self.scale(factor, factor)
            self._zoom = 0

    def set_photo(self, pixmap=None):
        self._zoom = 0
        if pixmap and not pixmap.isNull():
            self._empty = False
            self.setDragMode(QtWidgets.QGraphicsView.ScrollHandDrag)
            self._photo.setPixmap(pixmap)
        else:
            self._empty = True
            self.setDragMode(QtWidgets.QGraphicsView.NoDrag)
            self._photo.setPixmap(QtGui.QPixmap())
        self.fit_in_view()

    def wheelEvent(self, event):
        if self.has_photo():
            if event.angleDelta().y() > 0:
                factor = 1.25
                self._zoom += 1
            else:
                factor = 0.8
                self._zoom -= 1
            if self._zoom > 0:
                self.scale(factor, factor)
            elif self._zoom == 0:
                self.fit_in_view()
            else:
                self._zoom = 0

    def toggleDragMode(self):
        if self.dragMode() == QtWidgets.QGraphicsView.ScrollHandDrag:
            self.setDragMode(QtWidgets.QGraphicsView.NoDrag)
        elif not self._photo.pixmap().isNull():
            self.setDragMode(QtWidgets.QGraphicsView.ScrollHandDrag)

    def mousePressEvent(self, event):
        if self._photo.isUnderMouse():
            self.photoClicked.emit(QtCore.QPoint(event.pos()))
        super(PhotoViewer, self).mousePressEvent(event)


class Ui_mainWindow(object):
    def setupUi(self, mainWindow):
        mainWindow.setObjectName("mainWindow")
        mainWindow.setEnabled(True)
        mainWindow.resize(820, 695)
        mainWindow.setWindowOpacity(1.0)
        self.centralwidget = QtWidgets.QWidget(mainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.tab_widget = QtWidgets.QTabWidget(self.centralwidget)
        self.tab_widget.setGeometry(QtCore.QRect(10, 10, 800, 590))
        self.tab_widget.setTabPosition(QtWidgets.QTabWidget.North)
        self.tab_widget.setObjectName("tab_widget")

# ------------------------------------------------------------------------------
#
#                               Configuration Tab
#
# ------------------------------------------------------------------------------

        self.tab_configuration = QtWidgets.QWidget()
        self.tab_configuration.setObjectName("tab_configuration")

# ---------------------------------- Addition Object ----------------------------------

        self.group_box_addition_object = QtWidgets.QGroupBox(self.tab_configuration)
        self.group_box_addition_object.setGeometry(QtCore.QRect(10, 10, 780, 200))
        self.group_box_addition_object.setObjectName("group_box_addition_object")
        self.button_addition_object = QtWidgets.QPushButton(self.group_box_addition_object)
        self.button_addition_object.setGeometry(QtCore.QRect(600, 150, 150, 40))
        self.button_addition_object.setObjectName("button_addition_object")
        self.combo_box_addition_select_module = QtWidgets.QComboBox(self.group_box_addition_object)
        self.combo_box_addition_select_module.setEnabled(True)
        self.combo_box_addition_select_module.setGeometry(QtCore.QRect(10, 100, 150, 40))
        self.combo_box_addition_select_module.setObjectName("combo_box_addition_select_module")
        self.combo_box_addition_select_object = QtWidgets.QComboBox(self.group_box_addition_object)
        self.combo_box_addition_select_object.setGeometry(QtCore.QRect(10, 40, 150, 40))
        self.combo_box_addition_select_object.setObjectName("combo_box_addition_select_object")

#                               Addition Object Info Group

        self.radio_button_addition_object_type_parallel = QtWidgets.QRadioButton(self.group_box_addition_object)
        self.radio_button_addition_object_type_parallel.setGeometry(QtCore.QRect(550, 100, 90, 40))
        self.radio_button_addition_object_type_parallel.setObjectName("radio_button_addition_object_type_parallel")
        self.spin_box_addition_object_count = QtWidgets.QSpinBox(self.group_box_addition_object)
        self.spin_box_addition_object_count.setGeometry(QtCore.QRect(200, 160, 60, 40))
        self.spin_box_addition_object_count.setMinimum(1)
        self.spin_box_addition_object_count.setMaximum(999)
        self.spin_box_addition_object_count.setObjectName("spin_box_addition_object_count")
        self.line_edit_addition_object = QtWidgets.QLineEdit(self.group_box_addition_object)
        self.line_edit_addition_object.setGeometry(QtCore.QRect(200, 100, 150, 40))
        self.line_edit_addition_object.setObjectName("line_edit_addition_object")
        self.radio_button_addition_object_type_serial = QtWidgets.QRadioButton(self.group_box_addition_object)
        self.radio_button_addition_object_type_serial.setGeometry(QtCore.QRect(450, 100, 90, 40))
        self.radio_button_addition_object_type_serial.setIconSize(QtCore.QSize(16, 16))
        self.radio_button_addition_object_type_serial.setChecked(True)
        self.radio_button_addition_object_type_serial.setObjectName("radio_button_addition_object_type_serial")
        self.line_edit_addition_object_failure_rate = QtWidgets.QLineEdit(self.group_box_addition_object)
        self.line_edit_addition_object_failure_rate.setGeometry(QtCore.QRect(300, 160, 150, 40))
        self.line_edit_addition_object_failure_rate.setObjectName("line_edit_addition_object_failure_rate")
        self.label_object_type = QtWidgets.QLabel(self.group_box_addition_object)
        self.label_object_type.setGeometry(QtCore.QRect(10, 20, 100, 20))
        self.label_object_type.setObjectName("label_object_type")
        self.label_select_module = QtWidgets.QLabel(self.group_box_addition_object)
        self.label_select_module.setGeometry(QtCore.QRect(10, 80, 100, 20))
        self.label_select_module.setObjectName("label_select_module")
        self.label_object_name = QtWidgets.QLabel(self.group_box_addition_object)
        self.label_object_name.setGeometry(QtCore.QRect(200, 80, 100, 20))
        self.label_object_name.setObjectName("label_object_name")
        self.label_object_count = QtWidgets.QLabel(self.group_box_addition_object)
        self.label_object_count.setGeometry(QtCore.QRect(200, 140, 100, 20))
        self.label_object_count.setObjectName("label_object_count")
        self.label_object_failure_rate = QtWidgets.QLabel(self.group_box_addition_object)
        self.label_object_failure_rate.setGeometry(QtCore.QRect(300, 140, 100, 20))
        self.label_object_failure_rate.setObjectName("label_object_failure_rate")
        self.label_select_object_type = QtWidgets.QLabel(self.group_box_addition_object)
        self.label_select_object_type.setGeometry(QtCore.QRect(480, 80, 150, 20))
        self.label_select_object_type.setObjectName("label_select_object_type")

#                           ---> Functions <---

        self.combo_box_addition_select_object.currentIndexChanged.connect(self.addition_control_combobox_select_module)

        self.addition_combobox_select_object_items()                                            # Items
        self.addition_control_combobox_select_module()
        self.button_addition_object.clicked.connect(self.addition_click_button_object)
        self.addition_object_button_control()
        self.line_edit_addition_object.textChanged.connect(self.addition_object_button_control)

# ---------------------------------- Edit Type ----------------------------------

        self.group_box_edit_type = QtWidgets.QGroupBox(self.tab_configuration)
        self.group_box_edit_type.setGeometry(QtCore.QRect(10, 220, 780, 380))
        self.group_box_edit_type.setObjectName("group_box_edit_type")

#                                  Component

        self.group_box_component = QtWidgets.QGroupBox(self.group_box_edit_type)
        self.group_box_component.setGeometry(QtCore.QRect(10, 150, 550, 170))
        self.group_box_component.setObjectName("group_box_component")
        self.combo_box_main_component = QtWidgets.QComboBox(self.group_box_component)
        self.combo_box_main_component.setGeometry(QtCore.QRect(20, 110, 150, 40))
        self.combo_box_main_component.setObjectName("combo_box_main_component")
        self.combo_box_other_component = QtWidgets.QComboBox(self.group_box_component)
        self.combo_box_other_component.setGeometry(QtCore.QRect(180, 110, 150, 40))
        self.combo_box_other_component.setObjectName("combo_box_other_component")
        self.radio_button_component_serial = QtWidgets.QRadioButton(self.group_box_component)
        self.radio_button_component_serial.setGeometry(QtCore.QRect(350, 110, 90, 40))
        self.radio_button_component_serial.setIconSize(QtCore.QSize(16, 16))
        self.radio_button_component_serial.setObjectName("radio_button_component_serial")
        self.radio_button_component_parallel = QtWidgets.QRadioButton(self.group_box_component)
        self.radio_button_component_parallel.setGeometry(QtCore.QRect(450, 110, 90, 40))
        self.radio_button_component_parallel.setObjectName("radio_button_component_parallel")
        self.check_box_relationship = QtWidgets.QCheckBox(self.group_box_component)
        self.check_box_relationship.setGeometry(QtCore.QRect(20, 30, 250, 30))
        self.check_box_relationship.setObjectName("check_box_relationship")
        self.label_select_component = QtWidgets.QLabel(self.group_box_component)
        self.label_select_component.setGeometry(QtCore.QRect(20, 80, 150, 20))
        self.label_select_component.setObjectName("label_select_component")
        self.label_select_component_type = QtWidgets.QLabel(self.group_box_component)
        self.label_select_component_type.setGeometry(QtCore.QRect(370, 80, 175, 20))
        self.label_select_component_type.setObjectName("label_select_component_type")
        self.label_select_other_component = QtWidgets.QLabel(self.group_box_component)
        self.label_select_other_component.setGeometry(QtCore.QRect(180, 80, 175, 20))
        self.label_select_other_component.setObjectName("label_select_other_component")

        self.check_box_relationship.stateChanged.connect(self.check_box_relationship_control)
        self.check_box_relationship_unchecked()

#                                  Module

        self.group_box_module = QtWidgets.QGroupBox(self.group_box_edit_type)
        self.group_box_module.setGeometry(QtCore.QRect(10, 30, 400, 100))
        self.group_box_module.setObjectName("group_box_module")
        self.radio_button_module_serial = QtWidgets.QRadioButton(self.group_box_module)
        self.radio_button_module_serial.setGeometry(QtCore.QRect(200, 57, 90, 25))
        self.radio_button_module_serial.setIconSize(QtCore.QSize(16, 16))
        self.radio_button_module_serial.setObjectName("radio_button_module_serial")
        self.radio_button_module_parallel = QtWidgets.QRadioButton(self.group_box_module)
        self.radio_button_module_parallel.setGeometry(QtCore.QRect(300, 57, 90, 25))
        self.radio_button_module_parallel.setObjectName("radio_button_module_parallel")
        self.combo_box_module = QtWidgets.QComboBox(self.group_box_module)
        self.combo_box_module.setGeometry(QtCore.QRect(20, 50, 150, 40))
        self.combo_box_module.setObjectName("combo_box_module")
        self.label_select_module_2 = QtWidgets.QLabel(self.group_box_module)
        self.label_select_module_2.setGeometry(QtCore.QRect(20, 30, 100, 20))
        self.label_select_module_2.setObjectName("label_select_module_2")
        self.label_select_module_type = QtWidgets.QLabel(self.group_box_module)
        self.label_select_module_type.setGeometry(QtCore.QRect(220, 30, 150, 20))
        self.label_select_module_type.setObjectName("label_select_module_type")
        self.button_edit_type = QtWidgets.QPushButton(self.group_box_edit_type)
        self.button_edit_type.setGeometry(QtCore.QRect(650, 225, 100, 40))
        self.button_edit_type.setObjectName("button_edit_type")
        self.tab_widget.addTab(self.tab_configuration, "")

#                           ---> Functions <---

        # Combo Box Kontrolleri
        self.edit_combobox_module_items()
        self.edit_combobox_component_items()
        # self.edit_change_type_module()
        # self.edit_change_type_component()

        self.combo_box_module.currentIndexChanged.connect(self.edit_change_type_module)     # Açık
        self.combo_box_main_component.currentIndexChanged.connect(self.edit_other_combobox_component_items)
        self.button_edit_type.clicked.connect(self.edit_type_click_button_object)

# ------------------------------------------------------------------------------
#
#                               Parameter Tab
#
# ------------------------------------------------------------------------------

        self.tab_parameter = QtWidgets.QWidget()
        self.tab_parameter.setEnabled(True)
        self.tab_parameter.setObjectName("tab_parameter")
        self.group_box_task = QtWidgets.QGroupBox(self.tab_parameter)
        self.group_box_task.setGeometry(QtCore.QRect(310, 320, 200, 230))
        self.group_box_task.setObjectName("group_box_task")
        self.text_task = QtWidgets.QTextBrowser(self.group_box_task)
        self.text_task.setGeometry(QtCore.QRect(0, 20, 200, 150))
        self.text_task.setReadOnly(False)
        self.text_task.setAcceptRichText(True)
        self.text_task.setObjectName("text_task")
        self.button_task = QtWidgets.QPushButton(self.group_box_task)
        self.button_task.setEnabled(True)
        self.button_task.setGeometry(QtCore.QRect(100, 180, 100, 50))
        self.button_task.setDefault(False)
        self.button_task.setObjectName("button_task")

        self.text_task.setText(str(self.read_task_field()))
        self.button_task.clicked.connect(self.click_button_task)

        self.group_box_load_temperature = QtWidgets.QGroupBox(self.tab_parameter)
        self.group_box_load_temperature.setGeometry(QtCore.QRect(520, 320, 260, 170))
        self.group_box_load_temperature.setObjectName("group_box_load_temperature")
        self.text_load_temperature = QtWidgets.QTextBrowser(self.group_box_load_temperature)
        self.text_load_temperature.setGeometry(QtCore.QRect(0, 20, 155, 150))
        self.text_load_temperature.setReadOnly(False)
        self.text_load_temperature.setObjectName("text_load_temperature")
        self.button_load_temperature = QtWidgets.QPushButton(self.group_box_load_temperature)
        self.button_load_temperature.setGeometry(QtCore.QRect(160, 20, 100, 50))
        self.button_load_temperature.setObjectName("button_load_temperature")

        self.group_box_parameter = QtWidgets.QGroupBox(self.tab_parameter)
        self.group_box_parameter.setEnabled(True)
        self.group_box_parameter.setGeometry(QtCore.QRect(0, 0, 300, 550))
        self.group_box_parameter.setCursor(QtGui.QCursor(QtCore.Qt.ArrowCursor))
        self.group_box_parameter.setMouseTracking(False)
        self.group_box_parameter.setAutoFillBackground(False)
        self.group_box_parameter.setFlat(True)
        self.group_box_parameter.setCheckable(False)
        self.group_box_parameter.setChecked(False)
        self.group_box_parameter.setObjectName("group_box_parameter")
        self.text_parameter = QtWidgets.QTextBrowser(self.group_box_parameter)
        self.text_parameter.setGeometry(QtCore.QRect(0, 20, 300, 470))
        self.text_parameter.setReadOnly(False)
        self.text_parameter.setObjectName("text_parameter")
        self.button_parameter = QtWidgets.QPushButton(self.group_box_parameter)
        self.button_parameter.setGeometry(QtCore.QRect(200, 500, 100, 50))
        self.button_parameter.setObjectName("button_parameter")

        self.text_parameter.setText(str(self.read_parameter_field()))
        self.button_parameter.clicked.connect(self.click_button_parameter)

        self.group_box_formula = QtWidgets.QGroupBox(self.tab_parameter)
        self.group_box_formula.setGeometry(QtCore.QRect(310, 0, 470, 310))
        self.group_box_formula.setObjectName("group_box_formula")
        self.text_formula = QtWidgets.QTextBrowser(self.group_box_formula)
        self.text_formula.setGeometry(QtCore.QRect(0, 20, 365, 120))
        self.text_formula.setReadOnly(False)
        self.text_formula.setObjectName("text_formula")
        self.group_box_variable = QtWidgets.QGroupBox(self.group_box_formula)
        self.group_box_variable.setGeometry(QtCore.QRect(0, 140, 365, 170))
        self.group_box_variable.setObjectName("group_box_variable")
        self.text_variable = QtWidgets.QTextBrowser(self.group_box_variable)
        self.text_variable.setGeometry(QtCore.QRect(0, 20, 365, 150))
        self.text_variable.setObjectName("text_variable")

        self.text_variable.setText(str(self.read_variable_field()))

        self.button_formula = QtWidgets.QPushButton(self.group_box_formula)
        self.button_formula.setGeometry(QtCore.QRect(370, 125, 100, 50))
        self.button_formula.setAutoDefault(False)
        self.button_formula.setDefault(False)
        self.button_formula.setFlat(False)
        self.button_formula.setObjectName("button_formula")

        self.text_formula.setText(str(self.read_formula_field()))
        self.button_formula.clicked.connect(self.click_button_formula)

# ------------------------------------------------------------------------------
#
#                               Monitoring Tab
#
# ------------------------------------------------------------------------------

        self.tab_widget.addTab(self.tab_parameter, "")
        self.tab_monitoring = QtWidgets.QWidget()
        self.tab_monitoring.setObjectName("tab_monitoring")

        self.group_box_monitoring = QtWidgets.QGroupBox(self.tab_monitoring)
        self.group_box_monitoring.setGeometry(QtCore.QRect(0, 100, 800, 460))
        self.group_box_monitoring.setObjectName("group_box_monitoring")
        self.combo_box_monitoring_graph = QtWidgets.QComboBox(self.tab_monitoring)
        self.combo_box_monitoring_graph.setGeometry(QtCore.QRect(10, 40, 150, 40))
        self.combo_box_monitoring_graph.setObjectName("combo_box_monitoring_graph")
        self.label_select_monitoring_object = QtWidgets.QLabel(self.tab_monitoring)
        self.label_select_monitoring_object.setGeometry(QtCore.QRect(10, 10, 200, 20))
        self.label_select_monitoring_object.setObjectName("label_select_monitoring_object")
        self.label_view = QtWidgets.QLabel(self.group_box_monitoring)

        self.label_view.setGeometry(0, 0, 800, 460)

        self.combo_box_monitoring_graph.currentIndexChanged.connect(self.change_monitoring_graph)
        self.combo_box_monitoring_graph_items()
        self.change_monitoring_graph()

        self.tab_widget.addTab(self.tab_monitoring, "")

        self.tab_widget.currentChanged.connect(self.combo_box_monitoring_graph_items)

# ------------------------------------------------------------------------------

        self.button_start_launch = QtWidgets.QPushButton(self.centralwidget)
        self.button_start_launch.setGeometry(QtCore.QRect(660, 610, 150, 75))
        self.button_start_launch.setAutoDefault(False)
        self.button_start_launch.setDefault(False)
        self.button_start_launch.setFlat(False)
        self.button_start_launch.setObjectName("button_start_launch")

        self.button_start_launch.clicked.connect(self.click_button_start_launch)
        self.button_start_launch.setEnabled(True)      # Buttonun Kullanilip Kullanilamayacagi

# ------------------------------------------------------------------------------

        mainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(mainWindow)
        self.tab_widget.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(mainWindow)

    def retranslateUi(self, mainWindow):
        _translate = QtCore.QCoreApplication.translate
        mainWindow.setWindowTitle(_translate("mainWindow", "Prognostic Health Management"))
        self.group_box_addition_object.setTitle(_translate("mainWindow", "Addition Object"))
        self.button_addition_object.setText(_translate("mainWindow", "Addition Object"))
        self.radio_button_addition_object_type_parallel.setText(_translate("mainWindow", "Parallel"))
        self.radio_button_addition_object_type_serial.setText(_translate("mainWindow", "Serial"))
        self.label_object_type.setText(_translate("mainWindow", "Object Type"))
        self.label_select_module.setText(_translate("mainWindow", "Select Module"))
        self.label_object_name.setText(_translate("mainWindow", "Object Name"))
        self.label_object_count.setText(_translate("mainWindow", "Task Hour"))
        self.label_object_failure_rate.setText(_translate("mainWindow", "Failure Rate"))
        self.label_select_object_type.setText(_translate("mainWindow", "Select Object Type"))
        self.group_box_edit_type.setTitle(_translate("mainWindow", "Edit Type"))
        self.group_box_component.setTitle(_translate("mainWindow", "Component"))
        self.radio_button_component_serial.setText(_translate("mainWindow", "Serial"))
        self.radio_button_component_parallel.setText(_translate("mainWindow", "Parallel"))
        self.check_box_relationship.setText(_translate("mainWindow", "Relationship Of Components"))
        self.label_select_component.setText(_translate("mainWindow", "Select Component"))
        self.label_select_component_type.setText(_translate("mainWindow", "Select Component Type"))
        self.label_select_other_component.setText(_translate("mainWindow", "Select Other Component"))
        self.group_box_module.setTitle(_translate("mainWindow", "Module"))
        self.radio_button_module_serial.setText(_translate("mainWindow", "Serial"))
        self.radio_button_module_parallel.setText(_translate("mainWindow", "Parallel"))
        self.label_select_module_2.setText(_translate("mainWindow", "Select Module"))
        self.label_select_module_type.setText(_translate("mainWindow", "Select Module Type"))
        self.button_edit_type.setText(_translate("mainWindow", "Save Change"))
        self.tab_widget.setTabText(
                                    self.tab_widget.indexOf(self.tab_configuration),
                                    _translate("mainWindow", "Configuration"))
        self.group_box_task.setTitle(_translate("mainWindow", "Task"))
        self.button_task.setText(_translate("mainWindow", "Confirm"))
        self.group_box_load_temperature.setTitle(_translate("mainWindow", "Load - Temperature"))
        self.button_load_temperature.setText(_translate("mainWindow", "Confirm"))
        self.group_box_parameter.setTitle(_translate("mainWindow", "Parameter"))
        self.button_parameter.setText(_translate("mainWindow", "Confirm"))
        self.group_box_formula.setTitle(_translate("mainWindow", "Formula"))
        self.group_box_variable.setTitle(_translate("mainWindow", "Variable"))
        self.button_formula.setText(_translate("mainWindow", "Confirm"))
        self.tab_widget.setTabText(self.tab_widget.indexOf(self.tab_parameter), _translate("mainWindow", "Parameter"))
        self.group_box_monitoring.setTitle(_translate("mainWindow", "Monitoring"))
        self.label_select_monitoring_object.setText(_translate(
                                                                "mainWindow",
                                                                "Select Monitoring Object"))
        self.tab_widget.setTabText(
                                   self.tab_widget.indexOf(self.tab_monitoring),
                                   _translate("mainWindow", "Monitoring"))
        self.button_start_launch.setText(_translate("mainWindow", "Start Launch"))

# ------------------------------------------------------------------------------
#
#                               Configuration Tab Functions
#
# ------------------------------------------------------------------------------

    def component_array(self):              # Yaml Filedaki Moduller
        system_component = dict(self.addition_object_read_system_function())
        return system_component.values()

    def module_array(self):
        system_module = dict(self.addition_object_read_system_function())
        return system_module.keys()

    def default_addition_click_button_object(self):
        if self.combo_box_addition_select_object.currentText() == "Module":
            self.combo_box_addition_select_module.clear()
            self.combo_box_addition_select_module.setEnabled(False)
            self.spin_box_addition_object_count.setValue(1)
            # self.spin_box_addition_object_count.setEnabled(False)
            # self.spin_box_addition_object_count.clear()
            self.line_edit_addition_object_failure_rate.setEnabled(False)
            self.line_edit_addition_object_failure_rate.clear()
            self.line_edit_addition_object.clear()

        else:
            self.spin_box_addition_object_count.setValue(1)
            self.line_edit_addition_object_failure_rate.setText("1")
            self.line_edit_addition_object.clear()

    def addition_click_button_object(self):
        select_object_type = self.combo_box_addition_select_object.currentText()
        object_name = self.line_edit_addition_object.text()                             # Get Object Name
        object_count = self.spin_box_addition_object_count.value()                      # Get Object Count
        object_failure_rate = self.line_edit_addition_object_failure_rate.text()        # Get Object Failure Rate
        radio_button_type = self.radio_button_addition_object_type_serial.isChecked()   # Radio Button Control

        if radio_button_type:
            object_type = "Serial"

        else:
            object_type = "Parallel"

        if select_object_type == "Component":
            select_object_module_type = self.combo_box_addition_select_module.currentText()

        else:
            select_object_module_type = ""
            task_hour = "\n" + str(object_name) + ": " + str(object_count)
            self.addition_task_hour(task_hour)

        # System Hazirlanmasi System**
        self.addition_object_system_function(
                                                select_object_type,
                                                select_object_module_type,
                                                object_name)

        # Info Hazirlanmasi Info**
        self.addition_object_info_function(
                                            select_object_type,
                                            select_object_module_type,
                                            object_name,
                                            object_count,
                                            object_failure_rate)

        # Type Hazirlanmasi Type**
        self.addition_object_type_function(select_object_type, select_object_module_type, object_name, object_type)

        # Button Click and Default Settings
        self.default_addition_click_button_object()

        # Module Combobox Doldurulması
        # self.edit_combobox_module_items()        # Açıktı

        self.radio_button_addition_object_type_serial.setChecked(True)

# ------------------------------------------------------------------------------

    def addition_object_system_function(self, select_object_type, select_object_module_type, object_name):
        system_module_system = "---\nSystem**: "
        system = dict(self.addition_object_read_system_function())

        if select_object_type == "Module":
            module_key = list(system.keys())

            if str(object_name) not in module_key:
                module_list = list()
                system[str(object_name)] = module_list

        else:
            component_value = list(system[str(select_object_module_type)])

            if str(object_name) not in component_value:
                system[str(select_object_module_type)].append(str(object_name))

        system_module_system += str(system) + "\n"
        self.addition_object_write_system_function(system_module_system)

    def addition_object_info_function(
                                        self,
                                        select_object_type,
                                        select_object_module_type,
                                        object_name,
                                        object_count,
                                        object_failure_rate):
        info_module_system = "---\nInfo**:"
        info = self.addition_object_read_info_function()

        control = "    " + str(object_name) + "*:"

        if info.find(str(control)) == -1:
            if select_object_type == "Module":
                temp_info = info_module_system + "\n    " + object_name + "*: ''\n" + info

            else:
                addition_object_info = {
                    "ObjectCount": int(object_count),
                    "ObjectFailureRate": float(object_failure_rate),
                }

                info_split = "\n    " + str(select_object_module_type) + "*:"
                read_info_temp = info.split(str(info_split), 1)
                read_info = str(read_info_temp[1])

                if (read_info[0:3] == " ''"):
                    read_info = read_info[3:]

                temp_info = info_module_system + read_info_temp[0] + "\n    " + select_object_module_type + "*:\n" \
                    + "        " + object_name + ": " + str(addition_object_info) + read_info
        else:
            temp_info = info_module_system + info

        self.addition_object_write_info_function(temp_info)

    def addition_object_type_function(self, select_object_type, select_object_module_type, object_name, object_type):
        type_module_system = "---\nType**:"
        system_type = self.addition_object_read_type_function()

        control = "    " + str(object_name) + "*:"

        if system_type.find(str(control)) == -1:

            if select_object_type == "Module":
                temp_type = type_module_system + "\n    " + object_name + "*:\n" + "        unrelated: " + "'***'" \
                    + "\n" + "        relation: " + "'*'" + "\n" + "        type*: " + object_type + "\n" \
                    + system_type

            else:
                module_split = str("\n    " + str(select_object_module_type) + "*:")

                read_type_temp = system_type.split(module_split, 1)
                read_unrelated_temp_1 = str(read_type_temp[1]).split("unrelated: ", 1)

                control = str(read_unrelated_temp_1[1]).split("relation: ", 1)

                if str(control[0]).find("'***'") == -1:
                    read_unrelated_temp_2 = str(read_unrelated_temp_1[1]).split("\n", 1)        # unrelated list
                    temp_read_type_1 = str(read_unrelated_temp_2[1]).split("relation: ", 1)
                    temp_read_type_2 = str(temp_read_type_1[1]).split("}}", 1)                  # relation dict
                    temp_read_unrelated_list = read_unrelated_temp_2[0]
                    temp_read_dict = temp_read_type_2[0] + "}}"
                    temp_read_type_dict = dict(eval(temp_read_dict))                            # dict olarak tutuyor
                    read_unrelated_list = list(eval(temp_read_unrelated_list))                  # list olarak tutuyor

                    temp_list_1 = list()
                    temp_list_2 = list()
                    temp_dict = {
                        "Serial": temp_list_1,
                        "Parallel": temp_list_2,
                    }

                    temp_read_type_dict[str(object_name)] = temp_dict

                    read_unrelated_list.append(str(object_name))

                    read_unrelated_temp_2[0] = str(read_unrelated_temp_2[0]).replace(
                                                                                        str(temp_read_unrelated_list),
                                                                                        str(read_unrelated_list),
                                                                                        1)
                    temp_read_type_1[1] = str(temp_read_type_1[1]).replace(
                                                                            str(temp_read_dict),
                                                                            str(temp_read_type_dict),
                                                                            1)

                    read_type = read_unrelated_temp_1[0] + "unrelated: " + read_unrelated_temp_2[0] + "\n" \
                        + "        relation: " + temp_read_type_1[1]

                    temp_type = type_module_system + read_type_temp[0] + "\n    " + select_object_module_type + "*:\n" \
                        + "        " + object_name + ":\n" + "            " + "type: " + str(object_type) \
                        + str(read_type)

                else:
                    read_unrelated_temp_2 = str(read_unrelated_temp_1[1]).split("\n", 1)
                    temp_read_type_1 = str(read_unrelated_temp_2[1]).split("relation: ", 1)
                    temp_list_1 = list()
                    temp_list_2 = list()
                    temp_dict = {
                        "Serial": temp_list_1,
                        "Parallel": temp_list_2,
                    }

                    temp_read_type_dict = {
                        str(object_name): dict(temp_dict),
                    }

                    read_unrelated_list = list()
                    read_unrelated_list.append(str(object_name))

                    read_unrelated_temp_2[0] = str(read_unrelated_temp_2[0]).replace(
                                                                                        "'***'",
                                                                                        str(read_unrelated_list),
                                                                                        1)

                    temp_read_type_1[1] = str(temp_read_type_1[1]).replace(
                                                                            "'*'",
                                                                            str(temp_read_type_dict),
                                                                            1)

                    read_type = read_unrelated_temp_1[0] + "unrelated: " + read_unrelated_temp_2[0] + "\n" \
                        + "        relation: " + temp_read_type_1[1]

                    temp_type = type_module_system + read_type_temp[0] + "\n    " + select_object_module_type + "*:\n" \
                        + "        " + object_name + ":\n" + "            " + "type: " + str(object_type) \
                        + read_type

        else:
            temp_type = type_module_system + system_type

        self.addition_object_write_type_function(temp_type)

# ------------------------------------------------------------------------------

    def edit_type_click_button_object(self):
        select_module = self.combo_box_module.currentText()
        select_main_component = self.combo_box_main_component.currentText()
        select_other_component = self.combo_box_other_component.currentText()

        # Module Type Radio Button Control
        module_radio_button_type = self.radio_button_module_serial.isChecked()

        # Component Type Radio Button Control
        component_radio_button_type = self.radio_button_component_serial.isChecked()

        # Module Radio Button
        if module_radio_button_type:
            module_object_type = "Serial"

        else:
            module_object_type = "Parallel"

        # Component Radio Button
        if component_radio_button_type:
            component_object_type = "Serial"

        else:
            component_object_type = "Parallel"

        # Module Type ın değiştirilmesi
        checkBox_state = self.check_box_relationship.isChecked()

        if checkBox_state:
            # Component Type ın değişitilimesi
            self.edit_type_component_function(
                                                select_module,
                                                select_main_component,
                                                select_other_component,
                                                component_object_type)

        """
        module_type_control = self.show_module_type_control()
        component_type_control = self.show_component_type_control()

        # Module Type farklıysa
        if module_object_type != module_type_control:
            self.change_module_type_control(module_object_type)

        if component_object_type != component_type_control:
            self.change_component_type_control(component_object_type)
        """

# ------------------------------------------------------------------------------

    def edit_type_component_function(
                                        self,
                                        select_module,
                                        select_main_component,
                                        select_other_component,
                                        component_object_type):
        try:
            if select_main_component != "None" and select_other_component != "None":
                type_module_system = "---\nType**:"
                system_type = self.addition_object_read_type_function()

                module_split = str("\n    " + select_module + "*:")
                read_type_temp = system_type.split(module_split, 1)
                read_unrelated_temp_1 = str(read_type_temp[1]).split("unrelated: ", 1)
                read_unrelated_temp_2 = str(read_unrelated_temp_1[1]).split("\n", 1)    # unrelated list
                temp_read_type_1 = str(read_unrelated_temp_2[1]).split("relation: ", 1)
                temp_read_type_2 = str(temp_read_type_1[1]).split("}}", 1)
                temp_read_dict = temp_read_type_2[0] + "}}"
                temp_read_unrelated_list = read_unrelated_temp_2[0]             # dict olarak tutuyor
                read_unrelated_list = list(eval(temp_read_unrelated_list))      # list olarak tutuyor
                temp_read_type_dict = dict(eval(temp_read_dict))
                # dict_keys = list(temp_read_type_dict.keys())
                temp_dict = temp_read_type_dict

                select_type = component_object_type

                if select_type == "Serial":
                    other_type = "Parallel"

                else:
                    other_type = "Serial"

                if str(select_main_component) in read_unrelated_list:

                    if str(select_other_component) in read_unrelated_list:
                        read_unrelated_list.remove(str(select_other_component))

                    read_unrelated_list.remove(str(select_main_component))
                    temp_dict[str(select_main_component)][str(select_type)].append(str(select_other_component))
                    temp_dict[str(select_other_component)][str(select_type)].append(str(select_main_component))

                elif str(select_other_component) in read_unrelated_list:

                    if str(select_main_component) in read_unrelated_list:
                        read_unrelated_list.remove(str(select_main_component))

                    read_unrelated_list.remove(str(select_other_component))
                    temp_dict[str(select_main_component)][str(select_type)].append(str(select_other_component))
                    temp_dict[str(select_other_component)][str(select_type)].append(str(select_main_component))

                else:
                    temp_dict[str(select_main_component)][str(other_type)].remove(str(select_other_component))
                    temp_dict[str(select_main_component)][str(select_type)].append(str(select_other_component))

                    temp_dict[str(select_other_component)][str(other_type)].remove(str(select_main_component))
                    temp_dict[str(select_other_component)][str(select_type)].append(str(select_main_component))

                replace_data = module_split + read_unrelated_temp_1[0] + "unrelated: " \
                    + str(temp_read_unrelated_list) + "\n" + "        relation: " + str(temp_read_dict)

                new_data = module_split + read_unrelated_temp_1[0] + "unrelated: " \
                    + str(read_unrelated_list) + "\n" + "        relation: " + str(temp_dict)

                temp_write = str(system_type).replace(str(replace_data), str(new_data), 1)
                write_type_system = type_module_system + temp_write

                self.addition_object_write_type_function(write_type_system)

        except:
            pass

# ------------------------------------------------------------------------------

    def check_box_relationship_control(self, state):
        if state:
            self.check_box_relationship_checked()

        else:
            self.check_box_relationship_unchecked()

    def check_box_relationship_checked(self):
        self.combo_box_other_component.setEnabled(True)
        self.edit_other_combobox_component_items()
        self.label_select_other_component.setEnabled(True)
        self.label_select_component_type.setText("Select Relationship Type")

    def check_box_relationship_unchecked(self):
        self.combo_box_other_component.setEnabled(False)
        self.combo_box_other_component.clear()
        self.label_select_other_component.setEnabled(False)
        self.label_select_component_type.setText("Select Component Type")

# ------------------------------------------------------------------------------

    def addition_object_write_system_function(self, value):
        write_value = value

        write_value = write_value.replace('],', '],\n          ')
        with open('/home/hakan/catkin_phm_tools/src/phm_tools/phm_reliability/params/reliability_system_parameters.yaml', 'w+') as f:
            f.write(write_value)

        f.close()

    def addition_object_read_system_function(self):
        f = open('/home/hakan/catkin_phm_tools/src/phm_tools/phm_reliability/params/reliability_system_parameters.yaml', 'r+')
        string = ""
        while 1:
            line = f.readline()
            if not line:
                break

            string += line

        f.close()

        data = string.split("**: ")

        if data[0] == "---\nSystem":
            return eval(data[1])

        else:
            return ""

# ------------------------------------------------------------------------------

    def addition_object_write_info_function(self, value):
        write_value = value

        with open('/home/hakan/catkin_phm_tools/src/phm_tools/phm_reliability/params/reliability_info_parameters.yaml', 'w+') as f:
            f.write(write_value)

        f.close()

    def addition_object_read_info_function(self):
        f = open('/home/hakan/catkin_phm_tools/src/phm_tools/phm_reliability/params/reliability_info_parameters.yaml', 'r+')
        string = ""
        while 1:
            line = f.readline()
            if not line:
                break

            string += line

        f.close()

        data = string.split("**:")

        if data[0] == "---\nInfo":
            return str(data[1])

        else:
            return ""

# ------------------------------------------------------------------------------

    def addition_object_write_type_function(self, value):
        write_value = value

        write_value = write_value.replace('}, ', '},\n                   ')
        with open('/home/hakan/catkin_phm_tools/src/phm_tools/phm_reliability/params/reliability_type_parameters.yaml', 'w+') as f:
            f.write(write_value)

        f.close()

    def addition_object_read_type_function(self):
        f = open('/home/hakan/catkin_phm_tools/src/phm_tools/phm_reliability/params/reliability_type_parameters.yaml', 'r+')
        string = ""
        while 1:
            line = f.readline()
            if not line:
                break

            string += line

        f.close()

        data = string.split("**:")

        if data[0] == "---\nType":
            return str(data[1])

        else:
            return ""

    def addition_task_hour(self, write_value):
        with open(
                    '/home/hakan/catkin_phm_tools/src/phm_tools/phm_task_plan/params/module_during_the_task.yaml',
                    'a+') as f:
            f.write(str(write_value))

        f.close()

# ------------------------------------------------------------------------------

    def addition_object_button_control(self):
        if self.line_edit_addition_object.text() == "":
            self.button_addition_object.setEnabled(False)

        else:
            self.button_addition_object.setEnabled(True)

    def addition_combobox_select_object_items(self):
        addition_select_object_items = ["Module", "Component"]
        self.combo_box_addition_select_object.clear()
        self.combo_box_addition_select_object.addItems(addition_select_object_items)

    def addition_control_combobox_select_module(self):
        selected_object_text = self.combo_box_addition_select_object.currentText()

        if selected_object_text == "Component":
            module = self.module_array()

            self.combo_box_addition_select_module.setEnabled(True)
            self.combo_box_addition_select_module.clear()
            self.combo_box_addition_select_module.addItems(module)
            self.label_object_count.setText("Count")
            self.spin_box_addition_object_count.setValue(1)
            self.line_edit_addition_object_failure_rate.setEnabled(True)
            self.line_edit_addition_object_failure_rate.setText("1")

        else:
            self.label_object_count.setText("Task Hour")
            self.spin_box_addition_object_count.setValue(1)
            self.combo_box_addition_select_module.clear()
            self.combo_box_addition_select_module.setEnabled(False)
            self.line_edit_addition_object_failure_rate.setEnabled(False)
            self.line_edit_addition_object_failure_rate.clear()

    def edit_change_type_module(self):       # Module Radio Buttonlarını işaretlemek için
        self.edit_combobox_component_items()
        # Açık
        selected_type = self.show_module_type_control()

        if selected_type == 'Serial':
            self.radio_button_module_serial.setChecked(True)

        if selected_type == 'Parallel':
            self.radio_button_module_parallel.setChecked(True)

    def edit_change_type_component(self):    # Component Radio Buttonlarını işaretlemek için

        selected_type = self.show_component_type_control()

        if selected_type == 'Serial':
            self.radio_button_component_serial.setChecked(True)

        if selected_type == 'Parallel':
            self.radio_button_component_parallel.setChecked(True)

    # Module Combo Box' ının içini dolduruyor
    def edit_combobox_module_items(self):
        module = self.module_array()

        self.combo_box_module.clear()
        self.combo_box_module.addItems(module)

    def select_module_component_value(self):
        component_list = self.component_array()
        selected_module_index = self.combo_box_module.currentIndex()

        if len(component_list) < 1:
            return ""
        else:
            selected_module = component_list[selected_module_index]

            return selected_module

    # Main Component Combo Box' ının içini dolduruyor
    def edit_combobox_component_items(self):
        selected_module = self.select_module_component_value()

        if len(selected_module) < 1:
            self.combo_box_main_component.clear()
            self.combo_box_main_component.addItem("None")
            self.combo_box_other_component.clear()
            self.combo_box_other_component.addItem("None")

        else:
            self.combo_box_main_component.clear()
            self.combo_box_main_component.addItems(selected_module)

    # Other Component Combo Box' ının içini dolduruyor
    def edit_other_combobox_component_items(self):
        selected_module = self.select_module_component_value()

        if len(selected_module) < 1:
            self.combo_box_other_component.clear()

        else:
            select_component_value = self.combo_box_main_component.currentIndex()

            if len(selected_module) > 1:
                selected_module.pop(select_component_value)
                self.combo_box_other_component.clear()
                self.combo_box_other_component.addItems(selected_module)

            else:
                self.combo_box_other_component.clear()
                self.combo_box_other_component.addItem("None")

        self.edit_change_type_component()     # Açık

# ------------------------------------------------------------------------------

    """
    # Module Radio Buttons Select Type
    def radio_button_module_serial_select(self, enabled):
        if enabled:
            self.change_module_type_control("Serial")

    def radio_button_module_parallel_select(self, enabled):
        if enabled:
            self.change_module_type_control("Parallel")
    """

# ------------------------------------------------------------------------------

    def show_module_type_control(self):
        select_module = self.combo_box_module.currentText()

        temp_type = self.addition_object_read_type_function()

        if temp_type != "":

            module_split = str(select_module) + "*:"

            temp_read_1 = temp_type.split(str(module_split), 1)
            temp_read_2 = str(temp_read_1[1]).split("type*: ", 1)
            temp_read_3 = str(temp_read_2[1]).split(str("\n"), 1)
            temp_read_type = str(temp_read_3[0])

            return temp_read_type

    def show_component_type_control(self):
        select_module = self.combo_box_module.currentText()
        select_component = ""
        select_component = self.combo_box_main_component.currentText()

        temp_type = self.addition_object_read_type_function()

        if temp_type != "" and select_component != "":

            module_split = str(select_module) + "*:"

            temp_read_1 = temp_type.split(str(module_split), 1)
            temp_read_2 = str(temp_read_1[1]).split(str(select_component), 1)
            temp_read_3 = str(temp_read_2[1]).split("type: ", 1)
            temp_read_4 = str(temp_read_3[1]).split(str("\n"), 1)
            temp_read_type = str(temp_read_4[0])

            return temp_read_type

    def change_module_type_control(self, module_type):
        select_module = self.combo_box_module.currentText()

        type_module_system = "---\nType**:"
        temp_type = self.addition_object_read_type_function()

        module_split = str(select_module) + "*:"

        temp_read = temp_type.split(str(module_split), 1)

        if module_type == "Serial":
            temp_read[1] = str(temp_read[1]).replace("type*: Parallel", "type*: Serial", 1)

        else:
            temp_read[1] = str(temp_read[1]).replace("type*: Serial", "type*: Parallel", 1)

        write_type = type_module_system + temp_read[0] + module_split + temp_read[1]

        self.addition_object_write_type_function(write_type)

    def change_component_type_control(self, component_type):
        select_module = self.combo_box_module.currentText()
        select_component = ""
        select_component = self.combo_box_main_component.currentText()

        if select_component != "":
            type_module_system = "---\nType**:"
            temp_type = self.addition_object_read_type_function()

            module_split = str(select_module) + "*:"
            component_split = "        " + str(select_component) + ":"

            temp_read_1 = temp_type.split(str(module_split), 1)
            temp_read_2 = str(temp_read_1[1]).split(str(component_split), 1)

            if component_type == "Serial":
                temp_read_2[1] = str(temp_read_2[1]).replace("type: Parallel", "type: Serial", 1)

            else:
                temp_read_2[1] = str(temp_read_2[1]).replace("type: Serial", "type: Parallel", 1)

            write_type = type_module_system + temp_read_1[0] + module_split + temp_read_2[0] + component_split \
                + temp_read_2[1]

            self.addition_object_write_type_function(write_type)

# ------------------------------------------------------------------------------
#
#                               Parameter Tab Functions
#
# ------------------------------------------------------------------------------

    def read_parameter_field(self):
        f = open('/home/hakan/catkin_phm_tools/src/phm_tools/phm_reliability/params/reliability_info_parameters.yaml', 'r')
        string = ""
        while 1:
            line = f.readline()
            if not line:
                break

            string += line

        f.close()

        return string

    def click_button_parameter(self):
        write_value = self.text_parameter.toPlainText()
        with open('/home/hakan/catkin_phm_tools/src/phm_tools/phm_reliability/params/reliability_info_parameters.yaml', 'w') as f:
            f.write(write_value)

        self.text_parameter.setText(str(self.read_parameter_field()))

# ------------------------------------------------------------------------------

    def read_task_field(self):
        f = open('/home/hakan/catkin_phm_tools/src/phm_tools/phm_task_plan/params/module_during_the_task.yaml', 'r')
        string = ""
        while 1:
            line = f.readline()
            if not line:
                break

            string += line

        f.close()

        return string

    def click_button_task(self):
        write_value = self.text_task.toPlainText()
        with open(
                    '/home/hakan/catkin_phm_tools/src/phm_tools/phm_task_plan/params/module_during_the_task.yaml',
                    'w') as f:
            f.write(write_value)

        self.text_task.setText(str(self.read_task_field()))

# ------------------------------------------------------------------------------

    def read_formula_field(self):
        f = open('/home/hakan/catkin_phm_tools/src/phm_tools/phm_reliability/params/formula.yaml', 'r')
        string = ""
        while 1:
            line = f.readline()
            if not line:
                break

            string += line

        f.close()

        return string

    def read_variable_field(self):
        f = open('/home/hakan/catkin_phm_tools/src/phm_tools/phm_reliability/params/variable.yaml', 'r')
        string = ""
        while 1:
            line = f.readline()
            if not line:
                break

            string += line

        f.close()

        return string

    def click_button_formula(self):
        write_formula = self.text_formula.toPlainText()
        with open('/home/hakan/catkin_phm_tools/src/phm_tools/phm_reliability/params/formula.yaml', 'w') as f:
            f.write(write_formula)

        self.text_formula.setText(str(self.read_formula_field()))

        write_variable = self.text_variable.toPlainText()
        with open('/home/hakan/catkin_phm_tools/src/phm_tools/phm_reliability/params/variable.yaml', 'w') as v:
            v.write(write_variable)

        self.text_variable.setText(str(self.read_variable_field()))

# ------------------------------------------------------------------------------
#
#                               Parameter Tab Functions
#
# ------------------------------------------------------------------------------

    def combo_box_monitoring_graph_items(self):
        module = self.module_array()

        self.combo_box_monitoring_graph.clear()
        self.combo_box_monitoring_graph.addItem("System")
        self.combo_box_monitoring_graph.addItems(module)

        self.change_monitoring_graph()

    def change_monitoring_graph(self):
        select_monitoring_object = self.combo_box_monitoring_graph.currentText()

        self.label_view.clear()
        self.monitoring_graph(select_monitoring_object)

    def monitoring_graph(self, image_name):
        if image_name == "System":
            self.imagePath = "/home/hakan/catkin_phm_tools/src/phm_tools/phm_reliability/source/" \
                + str(image_name) + "_Reliability.png"

        else:
            self.imagePath = "/home/hakan/catkin_phm_tools/src/phm_tools/phm_reliability/source/module/" \
                + str(image_name) + ".png"

        self.viewer = PhotoViewer(self.label_view)
        self.viewer.set_photo(QtGui.QPixmap(self.imagePath))
        self.viewer.setGeometry(0, 0, 800, 460)
        self.viewer.showFullScreen()
        self.viewer.show()

# ------------------------------------------------------------------------------

    def click_button_start_launch(self):
        os.system("gnome-terminal -x roslaunch phm_start start_phm.launch")

# ------------------------------------------------------------------------------


if __name__ == '__main__':
    rospy.init_node('phm_gui')

    app = QtWidgets.QApplication(sys.argv)
    mainWindow = QtWidgets.QMainWindow()
    ui = Ui_mainWindow()
    ui.setupUi(mainWindow)
    mainWindow.show()

    sys.exit(app.exec_())
