import os
import rospy
import rospkg
import pyqtgraph

from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
import PyQt4.QtCore as QtCore

from .window_manager import WindowManager

# class that manages the pulse analysis window and ui
class PulseAnalysisWindowManager(WindowManager):

  # Initialize the WindowManager base class. The WindowManager class
  # creates the _widget object that will be used by this window and
  # guarantees successful shutdown of rqt upon program termination.
  # Args:
  #	pr2_interface: the single pr2 interface plug in object
  def __init__(self, pr2_interface):
    super(PulseAnalysisWindowManager, self).__init__(pr2_interface)

    # Get path to UI file which should be in the "resource" folder of this package
    ui_file = os.path.join(
        rospkg.RosPack().get_path(
            'rqt_pr2_hand_syntouch_sensor_interface'), 'resource', 
            'pulsedetection.ui')

    # Extend the widget with all attributes and children from UI file
    loadUi(ui_file, self._widget)

    # Give QObjects reasonable names
    self._widget.setObjectName('PulseAnalysis')

    self._widget.setWindowTitle('Pulse Analyzer')

    self._pulse_graph = pyqtgraph.PlotWidget()
    self._widget.pulse_layout.addWidget(self._pulse_graph)

    self._timer = QtCore.QTimer()
    self._timer.timeout.connect(self.update_graphs)
    self._timer.start(33) # 60 frames per second.

    # Add widget to the user interface
    user_interface = pr2_interface.get_user_interface()
    user_interface.add_widget(self._widget)


  def update_graphs(self):
    # Plot the last 5 seconds of data from the pr2 interface.
    # Get the last 5.5 seconds of data for cleaner edge of the graph.
    recent_data = self._pr2_interface.get_data_range(-5.5)
    current_time = rospy.get_rostime().to_nsec()

    if not self._destroyed:
      self._pulse_graph.clear()
      self._pulse_graph.plot(
          [(value.get_t_recv() - current_time)/1e9 for value in recent_data],
          [value.get_force() for value in recent_data])
      self._pulse_graph.setXRange(-5, 0)
