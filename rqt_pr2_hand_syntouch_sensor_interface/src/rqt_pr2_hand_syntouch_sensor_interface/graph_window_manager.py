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
 
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt4agg import (
    FigureCanvasQTAgg as FigureCanvas,
    NavigationToolbar2QT as NavigationToolbar)

import random

from .window_manager import WindowManager

# class that handles the data graphs window and ui
class DataGraphsWindowManager(WindowManager):

  def addmplx(self, fig):
    self._widget.canvasX = FigureCanvas(fig)
    self._widget.xMPLlayout.addWidget(self._widget.canvasX)
    self._widget.canvasX.draw()

  def addmply(self, fig):
    self._widget.canvasY = FigureCanvas(fig)
    self._widget.yMPLlayout.addWidget(self._widget.canvasY)
    self._widget.canvasY.draw()

  def addmplz(self, fig):
    self._widget.canvasZ = FigureCanvas(fig)
    self._widget.zMPLlayout.addWidget(self._widget.canvasZ)
    self._widget.canvasZ.draw()

  def rmmplx(self):
    self._widget.xMPLlayout.removeWidget(self._widget.canvasX)
    self._widget.canvasX.close()

  def rmmply(self):
    self._widget.yMPLlayout.removeWidget(self._widget.canvasY)
    self._widget.canvasY.close()

  def rmmplz(self):
    self._widget.zMPLlayout.removeWidget(self._widget.canvasZ)
    self._widget.canvasZ.close()

  def update(self):
    print "huzah"
    self.rmmplx()
    self.rmmply()
    self.rmmplz()

    fig1 = Figure()
    ax1f1 = fig1.add_subplot(111)
    ax1f1.plot(np.random.rand(5))
    self.addmplx(fig1)

    fig1 = Figure()
    ax1f1 = fig1.add_subplot(111)
    ax1f1.plot(np.random.rand(5))
    self.addmply(fig1)

    fig1 = Figure()
    ax1f1 = fig1.add_subplot(111)
    ax1f1.plot(np.random.rand(5))
    self.addmplz(fig1)

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

    fig1 = Figure()
    ax1f1 = fig1.add_subplot(111)
    ax1f1.plot(np.random.rand(5))
    self.addmplx(fig1)

    fig1 = Figure()
    ax1f1 = fig1.add_subplot(111)
    ax1f1.plot(np.random.rand(5))
    self.addmply(fig1)

    fig1 = Figure()
    ax1f1 = fig1.add_subplot(111)
    ax1f1.plot(np.random.rand(5))
    self.addmplz(fig1)

    self.timer = qtc.QTimer()
    self.timer.timeout.connect(self.update)
    self.timer.start(100)

    # Add widget to the user interface
    user_interface = pr2_interface.get_user_interface()
    user_interface.add_widget(self._widget)

  # calls the function to readd the widgets if the window was closed
  def reopen(self):
    user_interface = self._pr2_interface.get_user_interface()
    user_interface.add_widget(self._widget)
