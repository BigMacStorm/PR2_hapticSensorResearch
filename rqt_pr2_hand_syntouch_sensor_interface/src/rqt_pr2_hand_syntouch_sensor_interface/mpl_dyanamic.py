#based on http://matplotlib.org/1.4.3/examples/user_interfaces/embedding_in_qt4.html

import sys
import os
import random
import rospkg

from matplotlib.backends import qt4_compat

use_pyside = qt4_compat.QT_API == qt4_compat.QT_API_PYSIDE

if use_pyside:
  from PySide import QtGui, QtCore
else:
  from PyQt4 import QtGui, QtCore

from numpy import arange, sin, pi
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

class MyMplCanvas(FigureCanvas):
  # This function initializes the MyMplCanvas with a figure and it also adds axes
  # Args:
  # 	parent: the parent of this particular mpl canvas
  # 	width: the width of the canvas
  # 	height: the height of the canvas
  # 	dpi: the dpi of the canvas
  def __init__(self, parent=None, width=5, height=4, dpi=100):
    fig = Figure(figsize=(width, height), dpi=dpi)
    self.axes = fig.add_subplot(111)

    # We want the axes cleared every time plot() is called
    self.axes.hold(False)

    # initialize the plot axes with specified data
    self.compute_initial_figure()

    # Initialize the FigureCanvas and set the parent of self
    FigureCanvas.__init__(self, fig)
    self.setParent(parent)

    # set the size policy for the FigureCanvas which makes it able to exapnd if more
    # window room is available and then updateGeometry sets the size of the canvas
    FigureCanvas.setSizePolicy(self,
                               QtGui.QSizePolicy.Expanding,
                               QtGui.QSizePolicy.Expanding)
    FigureCanvas.updateGeometry(self)

  # This is an abstract function, this is done liek this so it can be implemented in 
  # MyDynamicMplCanvas
  def compute_initial_figure(self):
    pass

class MyDynamicMplCanvas(MyMplCanvas):
  # This function initializes the MyMplCanvas by first calling the MyMplCanvas __init__
  # function and then it creates a timer and gives self a type
  def __init__(self, *args, **kwargs):
    MyMplCanvas.__init__(self, *args, **kwargs)
    self.timer = QtCore.QTimer(self)
    self.timer.timeout.connect(self.update_figure)
    self.type = 'x'

  # This function initializes the figure with a specific set of data to start
  def compute_initial_figure(self):
    self.axes.plot([0, 1, 2, 3], [1, 2, 0, 4], 'r')

  # This function sets the type of self
  # Args:
  # 	sent: The type that you are setting self to
  def set_type(self, sent):
    self.type = sent

  # This function gives the canvas access to the pr2_controller and starts a timer
  # Args:
  # 	sent_pr2_controller: The pr2_controller
  def set_pr2_interface(self, sent_pr2_interface):
    self.pr2_interface = sent_pr2_interface
    self.timer.start(1000)

  def update_figure(self):
    if self.type == 'x':
      print "update x graph"
      temp = self.pr2_interface.get_data_range(-5)
      x_range = []
      t_range = []
      current_time = temp[-1].get_t_recv()
      for x in temp:
        x_range.append(x.get_x())
        t_range.append(x.get_t_recv() - current_time)
      self.axes.plot(t_range, x_range, 'r')
      self.draw()
    elif self.type == 'y':
      print "update y graph"
      temp = self.pr2_interface.get_data_range(-5)
      y_range = []
      t_range = []
      current_time = temp[-1].get_t_recv()
      for x in temp:
        y_range.append(x.get_y())
        t_range.append(x.get_t_recv() - current_time)
      self.axes.plot(t_range, y_range, 'r')
      self.draw()
    elif self.type == 'z':
      print "update z graph"
      temp = self.pr2_interface.get_data_range(-5)
      z_range = []
      t_range = []
      current_time = temp[-1].get_t_recv()
      for x in temp:
        z_range.append(x.get_z())
        t_range.append(x.get_t_recv() - current_time)
      self.axes.plot(t_range, z_range, 'r')
      self.draw()
