import os
import rospy
import rospkg
import threading

from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from PyQt4.uic import loadUiType
import numpy as np
from PyQt4 import QtGui
import PyQt4.QtCore as qtc

from .mpl_dyanamic import MyMplCanvas, MyDynamicMplCanvas
 
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt4agg import (
    FigureCanvasQTAgg as FigureCanvas,
    NavigationToolbar2QT as NavigationToolbar)

import random

from .window_manager import WindowManager

# class that handles the data graphs window and ui
class DataGraphsWindowManager(WindowManager):

  # Initialize the WindowManager base class. The WindowManager class
  # creates the _widget object that will be used by this window and
  # guarantees successful shutdown of rqt upon program termination.
  # Args:
  #	   pr2_interface: the single pr2 interface plug in object
  def __init__(self, pr2_interface):

    super(DataGraphsWindowManager, self).__init__(pr2_interface)

    # Get path to UI file which should be in the "resource" folder of this package
    ui_file = os.path.join(
        rospkg.RosPack().get_path(
            'rqt_pr2_hand_syntouch_sensor_interface'), 'resource', 'datagraphs.ui')

    # Extend the widget with all attributes and children from UI file
    loadUi(ui_file, self._widget)

    # Give QObjects reasonable names
    self._widget.setObjectName('DataGraphsWindow')

    self._widget.setWindowTitle('Data Graphs')

    dc = MyDynamicMplCanvas()
    dc.set_pr2_interface(self._pr2_interface)
    dc.set_type('x')
    self._widget.xMPLlayout.addWidget(dc)
    """dc = MyDynamicMplCanvas()
    dc.set_type('y')
    self._widget.yMPLlayout.addWidget(dc)
    dc = MyDynamicMplCanvas()
    dc.set_type('z')
    self._widget.zMPLlayout.addWidget(dc)"""

    # Add widget to the user interface
    user_interface = pr2_interface.get_user_interface()
    user_interface.add_widget(self._widget)

  # calls the function to readd the widgets if the window was closed
  def reopen(self):
    user_interface = self._pr2_interface.get_user_interface()
    user_interface.add_widget(self._widget)
