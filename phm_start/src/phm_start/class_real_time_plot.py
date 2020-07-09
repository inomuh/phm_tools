#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""

    Real Time Plot Class

"""

from PyQt5 import QtCore, QtWidgets
import numpy as np
import matplotlib
matplotlib.use("Qt5Agg")
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure


class MainPlot(FigureCanvas):
    """Ultimately, this is a QWidget (as well as a FigureCanvasAgg, etc.)."""
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)

        # We want the axes cleared every time plot() is called
        self.axes.hold(True)

        self.compute_initial_figure()

        FigureCanvas.__init__(self, fig)
        self.setParent(parent)

        FigureCanvas.setSizePolicy(self,
                                   QtWidgets.QSizePolicy.Expanding,
                                   QtWidgets.QSizePolicy.Expanding)
        FigureCanvas.updateGeometry(self)

    def compute_initial_figure(self):
        """
            Compute Initial Figure Function
        """


class StaticPlot(MainPlot):
    """
        Create Static Plot
    """

    def __init__(self, x_list, y_list, *args, **kwargs):
        MainPlot.__init__(self, *args, **kwargs)

        self.x_list = x_list
        self.y_list = y_list
        self.create_plot()


    def compute_initial_figure(self):
        pass


    def create_plot(self):
        """
            Create Plot
        """
        self.axes.set_ylim(0, 1)
        self.axes.set_xlim(0, len(self.x_list)-1)
        self.axes.plot(self.x_list, self.y_list, '-r')


class POTCPlot(MainPlot):
    """
        Create POTC Plot
    """

    def __init__(self, ui_class, potc_main_dict, *args, **kwargs):
        MainPlot.__init__(self, *args, **kwargs)
        self.ui_class = ui_class
        self.potc_main_dict = potc_main_dict

        timer = QtCore.QTimer(self)
        timer.timeout.connect(self.update_figure)
        timer.start(1000)

    def compute_initial_figure(self):
        pass


    def update_figure(self):
        """
            Update Figure
        """
        self.axes.clear()

        predict_list = list(eval("self.ui_class." + self.potc_main_dict + "['Predict']['Nominal']['POTC']"))
        predict_time_line = list(eval("self.ui_class." + self.potc_main_dict + "['Predict']['Nominal']['Time']"))

        actual_list = list(eval("self.ui_class." + self.potc_main_dict + "['Actual']['Nominal']['POTC']"))
        actual_time_line = list(eval("self.ui_class." + self.potc_main_dict + "['Actual']['Nominal']['Time']"))

        predict_sb_list = list(eval("self.ui_class." + self.potc_main_dict + "['Predict']['Sensor Based']['POTC']"))
        predict_sb_time_line = list(eval("self.ui_class." + self.potc_main_dict + "['Predict']['Sensor Based']['Time']"))

        actual_sb_list = list(eval("self.ui_class." + self.potc_main_dict + "['Actual']['Sensor Based']['POTC']"))
        actual_sb_time_line = list(eval("self.ui_class." + self.potc_main_dict + "['Actual']['Sensor Based']['Time']"))

        self.axes.set_ylim(0, 1)

        if len(predict_time_line) >= len(actual_time_line):
            self.axes.set_xlim(0, len(predict_time_line)-1)

        else:
            self.axes.set_xlim(0, len(actual_time_line)-1)

        self.axes.plot(predict_time_line, predict_list, '-or', label="Predict")
        self.axes.plot(actual_time_line, actual_list, '-ob', label="Actual")

        self.axes.plot(predict_sb_time_line, predict_sb_list, '-oy', label="Predict SB")
        self.axes.plot(actual_sb_time_line, actual_sb_list, '-og', label="Actual SB")

        self.axes.legend(loc='lower left', fontsize='small')
        self.draw()


class SimulationPOTCPlot(MainPlot):
    """
        Create Simulation POTC Plot
    """

    def __init__(self, ui_class, x_list, y_list, *args, **kwargs):
        MainPlot.__init__(self, *args, **kwargs)
        self.ui_class = ui_class
        self.x_list_name = x_list
        self.y_list_name = y_list
        self.x_list = list()
        self.y_list = list()

        timer = QtCore.QTimer(self)
        timer.timeout.connect(self.update_figure)
        timer.start(500)


    def compute_initial_figure(self):
        pass


    def create_plot(self):
        """
            Create Static Plot
        """
        self.axes.clear()
        self.x_list = list(eval("self.ui_class." + self.x_list_name))
        self.y_list = list(eval("self.ui_class." + self.y_list_name))

        self.axes.set_ylim(0, 1)
        self.axes.set_xlim(0, len(self.x_list)-1)

        self.axes.plot(self.x_list, self.y_list, '-r')
        self.draw()


    def update_figure(self):
        """
            Update Figure
        """
        self.axes.clear()
        self.x_list = list(eval("self.ui_class." + self.x_list_name))
        self.y_list = list(eval("self.ui_class." + self.y_list_name))

        self.axes.set_ylim(0, 1)
        self.axes.set_xlim(0, len(self.x_list)-1)


        self.axes.plot(self.x_list, self.y_list, '-r')
        self.draw()


class DynamicPlot(MainPlot):
    """
        Create Dynamic Plot
    """

    def __init__(self, ui_class, ui_parameter, ui_time_parameter, *args, **kwargs):
        MainPlot.__init__(self, *args, **kwargs)
        self.ui_class = ui_class
        self.ui_parameter = ui_parameter
        self.ui_time_parameter = ui_time_parameter
        self.amount = 30
        timer = QtCore.QTimer(self)
        timer.timeout.connect(self.update_figure)
        timer.start(500)

    def compute_initial_figure(self):
        self.x_list = [0, 0, 0]
        self.y_list = [0, 0, 0]

        self.axes.plot(self.x_list, self.y_list, 'r')

    @classmethod
    def num_zeros(cls, decimal_number):
        """
            Find count of 0
        """
        return np.floor(np.abs(np.log10(decimal_number)))


    def update_figure(self):
        """
            Update Figure
        """
        try:
            plot_time = float(eval("self.ui_class." + self.ui_time_parameter))

            self.x_list.append(plot_time)
            self.y_list.append(float(eval("self.ui_class." + self.ui_parameter + ".text()")))

            read_value = str(eval("self.ui_class." + self.ui_parameter + ".text()"))

            if read_value not in ("", "0.0"):
                self.axes.clear()

                if read_value.find('e') != -1:
                    temp_limit_1 = read_value.split('e-')
                    value_min = int(temp_limit_1[1]) - 1
                    value_max = value_min + 2

                else:
                    zero_count = self.num_zeros(float(read_value))
                    value_min = int(zero_count)
                    value_max = value_min + 1

                self.axes.set_ylim((1.0 * pow(10, (-1 * value_max))), (1.0 * pow(10, (-1 * value_min))))

                right = self.x_list[-1]

                if len(self.x_list) > self.amount:
                    left = self.x_list[-self.amount]

                else:
                    left = self.x_list[0]

                self.axes.set_xlim(left, right)
                self.axes.plot(self.x_list[-self.amount:], self.y_list[-self.amount:], 'r')
                self.draw()

        except Exception as err:
            print("\nError: DynamicPlot\n")
            print(err)


class ReliabilityPlot(MainPlot):
    """
        Create Reliability Plot
    """

    def __init__(self, ui_class, ui_parameter, ui_time_parameter, scale_list, *args, **kwargs):
        MainPlot.__init__(self, *args, **kwargs)
        self.ui_class = ui_class
        self.ui_parameter = ui_parameter
        self.ui_time_parameter = ui_time_parameter
        self.amount = 30
        self.scale_list = scale_list
        timer = QtCore.QTimer(self)
        timer.timeout.connect(self.update_figure)
        timer.start(500)

    def compute_initial_figure(self):
        self.x_list = [0, 0, 0]
        self.y_list = [0, 0, 0]

        self.axes.plot(self.x_list, self.y_list, 'r')

    def update_figure(self):
        """
            Update Figure
        """
        plot_time = float(eval("self.ui_class." + self.ui_time_parameter))

        self.x_list.append(plot_time)
        self.y_list.append(float(eval("self.ui_class." + self.ui_parameter + ".text()")))

        self.axes.clear()

        if self.scale_list:
            self.axes.set_ylim(0, 1)

        else:
            self.axes.set_ylim(self.scale_list[0], self.scale_list[1])

        right = self.x_list[-1]

        if len(self.x_list) > self.amount:
            left = self.x_list[-self.amount]

        else:
            left = self.x_list[0]

        self.axes.set_xlim(left, right)
        self.axes.plot(self.x_list[-self.amount:], self.y_list[-self.amount:], 'r')
        self.draw()
