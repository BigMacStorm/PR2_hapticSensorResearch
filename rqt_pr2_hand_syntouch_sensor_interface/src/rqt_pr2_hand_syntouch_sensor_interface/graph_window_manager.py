import os
import rospy
import rospkg
import threading

from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from PyQt4 import QtGui
import PyQt4.QtCore as QtCore
import pyqtgraph

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

    self._x_plot = pyqtgraph.PlotWidget()
    self._widget.xMPLlayout.addWidget(self._x_plot)

    self._y_plot = pyqtgraph.PlotWidget()
    self._widget.yMPLlayout.addWidget(self._y_plot)

    self._z_plot = pyqtgraph.PlotWidget()
    self._widget.zMPLlayout.addWidget(self._z_plot)

    self._graph_timer = QtCore.QTimer()
    self._graph_timer.timeout.connect(self.update_graphs)
    self._graph_timer.start(33) # 60 frames per second.  

    # Add widget to the user interface
    user_interface = pr2_interface.get_user_interface()
    user_interface.add_widget(self._widget)


  def update_graphs(self):
    # Plot the last 5 seconds of data from the PR2.
    # Get the last 5.5 seconds of data for cleaner edge of the graph.
    recent_data = self._pr2_interface.get_data_range(-5.5)
    current_time = rospy.get_rostime().to_nsec()

    if not self._destroyed:
      self._x_plot.clear()
      self._x_plot.plot(
          [(value.get_t_recv() - current_time)/1e9 for value in recent_data],
          [value.get_force() for value in recent_data])
      self._x_plot.setXRange(-5, 0)
      self._y_plot.clear()
      self._y_plot.plot(
          [(value.get_t_recv() - current_time)/1e9 for value in recent_data],
          [value.get_temperature() for value in recent_data])
      self._y_plot.setXRange(-5, 0)
      self._z_plot.clear()
      self._z_plot.plot(
          [(value.get_t_recv() - current_time)/1e9 for value in recent_data],
          [value.get_thermal_flux() for value in recent_data])
      self._z_plot.setXRange(-5, 0)

  # calls the function to readd the widgets if the window was closed
  def reopen(self):
    user_interface = self._pr2_interface.get_user_interface()
    user_interface.add_widget(self._widget)
