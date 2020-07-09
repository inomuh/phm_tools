#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
    PHM Graph Ui Class
"""

from PyQt5 import QtCore, QtWidgets
from class_real_time_plot import StaticPlot


class Ui_StaticGraphWindow(object):
    """
        Static Graph Window Class
    """
    def __init__(self, x_label_value, y_label_value, title):
        self.x_value = x_label_value
        self.y_value = y_label_value
        self.title = str(title)


    def setupUi(self, GraphWindow):
        """
            Static Graph Window Setup Ui Function
        """
        GraphWindow.setObjectName("GraphWindow")
        GraphWindow.resize(450, 300)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(GraphWindow.sizePolicy().hasHeightForWidth())
        GraphWindow.setSizePolicy(sizePolicy)
        self.centralwidget = QtWidgets.QWidget(GraphWindow)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.centralwidget.sizePolicy().hasHeightForWidth())
        self.centralwidget.setSizePolicy(sizePolicy)
        self.centralwidget.setObjectName("centralwidget")
        self.gridLayout = QtWidgets.QGridLayout(self.centralwidget)
        self.gridLayout.setObjectName("gridLayout")
        self.widget_graph = QtWidgets.QWidget(self.centralwidget)
        self.widget_graph.setEnabled(True)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.widget_graph.sizePolicy().hasHeightForWidth())
        self.widget_graph.setSizePolicy(sizePolicy)
        self.widget_graph.setObjectName("widget_graph")
        self.gridLayout.addWidget(self.widget_graph, 0, 0, 1, 1)
        GraphWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(GraphWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 890, 25))
        self.menubar.setObjectName("menubar")
        GraphWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(GraphWindow)
        self.statusbar.setObjectName("statusbar")
        GraphWindow.setStatusBar(self.statusbar)


    # ---------------------------------------------------------------------------------------------


        self.gui_main()


    # ---------------------------------------------------------------------------------------------

        self.retranslateUi(GraphWindow)
        QtCore.QMetaObject.connectSlotsByName(GraphWindow)

    def retranslateUi(self, GraphWindow):
        """
            Static Graph Window Retranslate Ui Function
        """
        _translate = QtCore.QCoreApplication.translate
        GraphWindow.setWindowTitle(_translate("GraphWindow", self.title))


    def gui_main(self):
        """
            Static Graph Window Main Function
        """
        layout = QtWidgets.QVBoxLayout(self.widget_graph)
        static_canvas = StaticPlot(self.x_value, self.y_value, self.widget_graph, width=5, height=4, dpi=100)
        layout.addWidget(static_canvas)
        self.widget_graph.setFocus()
