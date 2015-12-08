import os
import rospy
import rospkg
import threading

from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

from .window_types import WindowTypes
from .window_manager import WindowManager

class IndexWindowManager(WindowManager):

  # This function will initialize the window and all widgets attached to this window.
  # Args:
  #	pr2_interface: the single pr2 interface plug in object
  def __init__(self, pr2_interface):
    # Initialize the WindowManager base class. The WindowManager class
    # creates the _widget object that will be used by this window and
    # guarantees successful shutdown of rqt upon program termination.   
    super(IndexWindowManager, self).__init__(pr2_interface)

    # Get path to UI file which should be in the "resource" folder of this package.
    ui_file = os.path.join(
        rospkg.RosPack().get_path(
            'rqt_pr2_hand_syntouch_sensor_interface'), 'resource', 'Index.ui')

    # Extend the widget with all attributes and children from UI file.
    loadUi(ui_file, self._widget)

    # Give QObjects reasonable names.
    self._widget.setObjectName('IndexWindow')
    self._widget.setWindowTitle(
        'PR2 Robotic Hand and Syntouch Sensor Real-Time Interface')

    # Add widget to the user interface.
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

    # Create a thread that will listen for messages from the PR2.
    self._worker = threading.Thread(target=self.update_labels)
    self._worker.start()


  # This function will check to see if the connection info button is clicked, open
  # the connection window.
  def _handle_connection_info_button_clicked(self):
    self._pr2_interface.open_window(WindowTypes.ConnectionWindow)

  # This function will check to see if the data graphs button is clicked, open the
  # data graphs window.
  def _handle_data_graphs_button_clicked(self):
    self._pr2_interface.open_window(WindowTypes.DataGraphsWindow)

  # This function will check to see if the sensor visualizer button is clicked, 
  # open the sensor visualizer window.
  def _handle_sensor_visualizer_button_clicked(self):
    self._pr2_interface.open_window(WindowTypes.SensorVisualizerWindow)

  # This function will check to see if the run programs button is clicked, open
  # the run programs window.
  def _handle_run_programs_button_clicked(self):
    self._pr2_interface.open_window(WindowTypes.RunProgramsWindow)

  # This function will check to see if the robot visualizer button is clicked,
  # open the robot visualizer window.
  def _handle_robot_visualizer_button_clicked(self):
    self._pr2_interface.open_window(WindowTypes.RobotVisualizerWindow)

  # This function will check to see if the lifetime statistics button is clicked,
  # open the lifetime statistics window.
  def _handle_lifetime_statistics_button_clicked(self):
    self._pr2_interface.open_window(WindowTypes.LifetimeStatsWindow)

  # This function will check to see if the pr2 is connected and if it is the window
  # will update the connection status information.
  def update_labels(self):
    # Initialize local variables.
    rate = rospy.Rate(5) # 5hz
    last_data_point = None

    # While rospy is still running and this window has not been destroyed.
    while not rospy.is_shutdown() and not self._destroyed:

      # Get the most recent data from the pr2 interface.
      current_data_point = self._pr2_interface.get_most_recent_data()

      # If the newest data recieved is equal to the last set of data recieved
      # then the pr2 is not "connected". Else the newest data is not equal to the 
      # last set of data recieved and the pr2 is "connected". This is currently
      # just for testing purposes to use with the mock_pr2.py which is simulating
      # incoming pr2 data.
      if last_data_point == current_data_point or not last_data_point:
        self._disconnected = True
        self._widget.label.setText("PR2 Status: Disconnected")
        self._widget.label_2.setText("Syntouch (fingers) Status: Disconnected")
      else:
        self._widget.label.setText("PR2 Status: Connected")
        self._widget.label_2.setText("Syntouch (fingers) Status: Connected")

      # Make the thread go to sleep and set the last data point to the current data point.
      rate.sleep()
      last_data_point = current_data_point

  # This function will reopen this window.
  def reopen(self):
    user_interface = self._pr2_interface.get_user_interface()
    user_interface.add_widget(self._widget)
