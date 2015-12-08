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
  """Ultimately, this is a QWidget (as well as a FigureCanvasAgg, etc.)."""
  def __init__(self, parent=None, width=5, height=4, dpi=100):
    fig = Figure(figsize=(width, height), dpi=dpi)
    self.axes = fig.add_subplot(111)
    # We want the axes cleared every time plot() is called
    self.axes.hold(False)

    self.compute_initial_figure()

    FigureCanvas.__init__(self, fig)
    self.setParent(parent)

    FigureCanvas.setSizePolicy(self,
                               QtGui.QSizePolicy.Expanding,
                               QtGui.QSizePolicy.Expanding)
    FigureCanvas.updateGeometry(self)

  def compute_initial_figure(self):
    pass

class MyDynamicMplCanvas(MyMplCanvas):
  """A canvas that updates itself every second with a new plot."""
  def __init__(self, *args, **kwargs):
    MyMplCanvas.__init__(self, *args, **kwargs)
    self.timer = QtCore.QTimer(self)
    self.timer.timeout.connect(self.update_figure)
    self.type = 'x'

  def compute_initial_figure(self):
    self.axes.plot([0, 1, 2, 3], [1, 2, 0, 4], 'r')

  def set_type(self, sent):
    self.type = sent

  def set_pr2_interface(self, sent_pr2_interface):
    self.pr2_interface = sent_pr2_interface
    self.timer.start(1000)

  def update_figure(self):
    if self.type == 'x':
      print "update x graph"
      temp = self.pr2_interface.get_data_range(-5)
      x_range = []
      y_range = []
      for x in temp:
        x_range.append(x.get_x())
        y_range.append(x.get_t_recv())
      for i in range(len(x_range)):
          print '%s %s' % (str(x_range[i]), str(y_range[i]))
      self.axes.plot(y_range, x_range, 'r')
      self.draw()
    elif self.type == 'y':
      print "updating y graph"
    elif self.type == 'z':
      print "updating z graph"

    """
    # Build a list of 4 random integers between 0 and 10 (both inclusive)
    l = [random.randint(0, 10) for i in range(4)]

    self.axes.plot([0, 1, 2, 3], l, 'r')
    self.draw()"""
