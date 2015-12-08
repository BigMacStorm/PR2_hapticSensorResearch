import os
import rospy
import rospkg
import threading

from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

from .window_types import WindowTypes
from .window_manager import WindowManager

class IndexWindowManager(WindowManager):

  def __init__(self, pr2_interface):
    # Initialize the WindowManager base class. The WindowManager class
    # creates the _widget object that will be used by this window and
    # guarantees successful shutdown of rqt upon program termination.
    super(IndexWindowManager, self).__init__(pr2_interface)

    # Get path to UI file which should be in the "resource" folder of this package
    ui_file = os.path.join(
        rospkg.RosPack().get_path(
            'rqt_pr2_hand_syntouch_sensor_interface'), 'resource', 'Index.ui')

    # Extend the widget with all attributes and children from UI file
    loadUi(ui_file, self._widget)
    # Give QObjects reasonable names
    self._widget.setObjectName('IndexWindow')

    self._widget.setWindowTitle(
        'PR2 Robotic Hand and Syntouch Sensor Real-Time Interface')

    # Add widget to the user interface
    user_interface = pr2_interface.get_user_interface()
    user_interface.add_widget(self._widget)

    # Register listeners for all of the buttons on the Index window.
    self._widget.ConnectionInfoButton.clicked.connect(
        self._handle_connection_info_button_clicked)

    self._widget.DataGraphsButton.clicked.connect(
        self._handle_data_graphs_button_clicked)

    self._widget.SensorVisualizerButton.clicked.connect(
        self._handle_sensor_visualizer_button_clicked)

    self._widget.RunProgramsButton.clicked.connect(
        self._handle_run_programs_button_clicked)

    self._widget.RobotVisualizerButton.clicked.connect(
        self._handle_robot_visualizer_button_clicked)

    self._widget.LifetimeStatisticsButton.clicked.connect(
        self._handle_lifetime_statistics_button_clicked)

    self._worker = threading.Thread(target=self.update_labels)
    self._worker.start()

  def _handle_connection_info_button_clicked(self):
    self._pr2_interface.open_window(WindowTypes.ConnectionWindow)

  def _handle_data_graphs_button_clicked(self):
    self._pr2_interface.open_window(WindowTypes.DataGraphsWindow)

  def _handle_sensor_visualizer_button_clicked(self):
    self._pr2_interface.open_window(WindowTypes.SensorVisualizerWindow)

  def _handle_run_programs_button_clicked(self):
    self._pr2_interface.open_window(WindowTypes.RunProgramsWindow)

  def _handle_robot_visualizer_button_clicked(self):
    self._pr2_interface.open_window(WindowTypes.RobotVisualizerWindow)

  def _handle_lifetime_statistics_button_clicked(self):
    self._pr2_interface.open_window(WindowTypes.LifetimeStatsWindow)

  def update_labels(self):
    rate = rospy.Rate(5) # 5hz
    last_data_point = None
    while not rospy.is_shutdown() and not self._destroyed:
      current_data_point = self._pr2_interface.get_most_recent_data()
      if last_data_point == current_data_point or not last_data_point:
        self._widget.label.setText("PR2 Status: Disconnected")
        self._widget.label_2.setText("Syntouch (fingers) Status: Disconnected")
      else:
        self._widget.label.setText("PR2 Status: Connected")
        self._widget.label_2.setText("Syntouch (fingers) Status: Connected")
      rate.sleep()
      last_data_point = current_data_point

  def reopen(self):
    user_interface = self._pr2_interface.get_user_interface()
    user_interface.add_widget(self._widget)
