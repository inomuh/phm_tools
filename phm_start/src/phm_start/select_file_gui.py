#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
    Select File Gui Class
"""

from PyQt5 import QtWidgets


class SelectFileGui:
    """
        Select File Gui Class
    """

    @classmethod
    def get_file_path(cls, caption="Open File", filefilter="", is_app=False):
        """
            "Images (*.png *.xpm *.jpg);;Text files (*.txt);;XML files (*.xml)"
        """

        if not is_app:
            import sys
            QtWidgets.QApplication(sys.argv)

        files = QtWidgets.QFileDialog.getOpenFileNames(caption=caption, filter=filefilter)
        file_list = list()

        for file in files:
            file_list.append(str(file))

        return file_list
