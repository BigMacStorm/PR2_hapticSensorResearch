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
    self._graph_1_index = 0
    self._graph_2_index = 0
    self._graph_3_index = 0
    self._widget.comboBox_1.currentIndexChanged.connect(self.change_graph_type)
    self._widget.comboBox_2.currentIndexChanged.connect(self.change_graph_type)
    self._widget.comboBox_3.currentIndexChanged.connect(self.change_graph_type)
    self._graph_timer.start(33) # 60 frames per second.  

    # Add widget to the user interface
    user_interface = pr2_interface.get_user_interface()
    user_interface.add_widget(self._widget)

  def change_graph_type(self):
    self._graph_1_index = self._widget.comboBox_1.currentIndex()
    self._graph_2_index = self._widget.comboBox_2.currentIndex()
    self._graph_3_index = self._widget.comboBox_3.currentIndex()
    print "%d %d %d" % (self._graph_1_index, self._graph_2_index, self._graph_3_index)

  def value_by_index(self, DTT, index):
    if index == 0:
      return DTT.get_force(0)
    if index == 1:
      return DTT.get_force(1)
    if index == 2:
      return DTT.get_temperature(0)
    if index == 3:
      return DTT.get_temperature(1)
    if index == 4:
      return DTT.get_thermal_flux(0)
    if index == 5:
      return DTT.get_thermal_flux(1)
    if index == 6:
      return DTT.get_microvibration(0)
    if index == 7:
      return DTT.get_microvibration(1)
    if index == 8:
      return DTT.get_fluid_pressure(0)
    if index == 9:
      return DTT.get_fluid_pressure(1)

  def update_graphs(self):
    # Plot the last 5 seconds of data from the PR2.
    # Get the last 5.5 seconds of data for cleaner edge of the graph.
    recent_data = self._pr2_interface.get_data_range(-5.5)
    current_time = rospy.get_rostime().to_nsec()

    if not self._destroyed:
      self._x_plot.clear()
      self._x_plot.plot(
          [(value.get_t_recv() - current_time)/1e9 for value in recent_data],
          [self.value_by_index(value, self._graph_1_index) for value in recent_data])
      self._x_plot.setXRange(-5, 0)
      self._y_plot.clear()
      self._y_plot.plot(
          [(value.get_t_recv() - current_time)/1e9 for value in recent_data],
          [self.value_by_index(value, self._graph_2_index) for value in recent_data])
      self._y_plot.setXRange(-5, 0)
      self._z_plot.clear()
      self._z_plot.plot(
          [(value.get_t_recv() - current_time)/1e9 for value in recent_data],
          [self.value_by_index(value, self._graph_3_index) for value in recent_data])
      self._z_plot.setXRange(-5, 0)

  # calls the function to readd the widgets if the window was closed
  def reopen(self):
    user_interface = self._pr2_interface.get_user_interface()
    user_interface.add_widget(self._widget)
