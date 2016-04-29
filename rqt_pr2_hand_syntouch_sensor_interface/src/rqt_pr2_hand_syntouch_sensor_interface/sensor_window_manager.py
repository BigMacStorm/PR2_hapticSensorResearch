from __future__ import division
import os
import rospy
import rospkg
import threading

from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from .window_manager import WindowManager

BOTTOM_BAR_POSITION = 221
MAX_BAR_HEIGHT = 171
MIN_BAR_HEIGHT = 20
SCALING_FACTOR = MAX_BAR_HEIGHT - MIN_BAR_HEIGHT

class SensorWindowManager(WindowManager):

  # This function will initialize the window and all widgets attached to this window.
  # Args:
  #	   pr2_interface: the single pr2 interface plug in object
  def __init__(self, pr2_interface):
    # Initialize the WindowManager base class. The WindowManager class
    # creates the _widget object that will be used by this window and
    # guarantees successful shutdown of rqt upon program termination.
    super(SensorWindowManager, self).__init__(pr2_interface)

    # Keep track of the lowest and highest value received so far to use for
    # graphing relative sensor intensity.
    self._low_data_values = {'force':None, 'fluid_pressure':None, 'microvibration':None,
                             'temperature':None, 'thermal_flux':None}
    self._high_data_values = {'force':None, 'fluid_pressure':None, 'microvibration':None,
                             'temperature':None, 'thermal_flux':None}

    # Get path to UI file which should be in the "resource" folder of this package
    ui_file = os.path.join(
        rospkg.RosPack().get_path(
            'rqt_pr2_hand_syntouch_sensor_interface'), 'resource', 'sensorvisualizer.ui')

    # Extend the widget with all attributes and children from UI file
    loadUi(ui_file, self._widget)

    # Give QObjects reasonable names
    self._widget.setObjectName('SensorVisualizerWindow')

    self._widget.setWindowTitle('Sensor Visualizer')

    # Add widget to the user interface
    user_interface = pr2_interface.get_user_interface()
    user_interface.add_widget(self._widget)

    # Create a thread that will update the sensor visualization bar graph.
    self._worker = threading.Thread(target=self.draw_bar_graphs)
    self._worker.start()

  def draw_bar_graphs(self):
    # Initialize local variables.
    rate = rospy.Rate(5) # 5hz
    last_data_point = None

    # While rospy is still running and this window has not been destroyed.
    while not rospy.is_shutdown() and not self._destroyed:
      biotac_id = self._widget.comboBox.currentIndex()

      # Get the most recent data from the pr2 interface.
      current_data_point = self._pr2_interface.get_most_recent_data()

      if last_data_point == current_data_point or not last_data_point:
        pass
      else:
        self.update_bar_height(self._widget.ForceBar,
            self.scale('force', current_data_point.get_force(biotac_id)))
        self.update_bar_height(self._widget.FluidPressureBar, 
            self.scale('fluid_pressure', 
                       current_data_point.get_fluid_pressure(biotac_id)))
        self.update_bar_height(self._widget.MicroVibrationBar,
            self.scale('microvibration', 
                       current_data_point.get_microvibration(biotac_id)))
        self.update_bar_height(self._widget.TemperatureBar,
            self.scale('temperature', 
                       current_data_point.get_temperature(biotac_id)))
        self.update_bar_height(self._widget.ThermalFluxBar,
            self.scale('thermal_flux', 
                       current_data_point.get_thermal_flux(biotac_id)))

      # Make the thread go to sleep and set the last data point to the current data point.
      rate.sleep()
      last_data_point = current_data_point

  def scale(self, data_type, value):
    if (self._low_data_values[data_type] > value or
        self._low_data_values[data_type] is None):
      self._low_data_values[data_type] = value
    if (self._high_data_values[data_type] < value or 
        self._high_data_values[data_type] is None): 
      self._high_data_values[data_type] = value

    high = self._high_data_values[data_type]
    low = self._low_data_values[data_type]

    bar_draw_height = MIN_BAR_HEIGHT
    if high > low:
      bar_draw_height += ((value - low) / (high - low))*SCALING_FACTOR
    if(data_type == 'force'):
      print self._high_data_values
      print self._low_data_values
      if high > low:
        print "%d %.2f" % (bar_draw_height, ((value - low) / (high - low)))
      
    return bar_draw_height

  def update_bar_height(self, bar, height):
    current_geometry = bar.geometry()
    current_geometry.setY(int(BOTTOM_BAR_POSITION - height))
    current_geometry.setHeight(int(height))
    bar.setGeometry(current_geometry)

  # This function will reopen this window.
  def reopen(self):
    user_interface = self._pr2_interface.get_user_interface()
    user_interface.add_widget(self._widget)
