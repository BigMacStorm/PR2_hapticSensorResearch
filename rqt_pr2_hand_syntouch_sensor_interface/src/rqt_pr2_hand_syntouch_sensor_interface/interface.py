import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

class WindowTypes:
  IndexWindow, ConnectionWindow, RunProgramsWindow, DataGraphsWindow, \
  LifetimeStatsWindow, SensorVisualizerWindow, RobotVisualizerWindow, \
  PulseAnalysisWindow, GrabObjectWindow, SwitchHandsWindow = range(10)

class IndexWindowManager:

  def __init__(self, pr2_interface):
    self._pr2_interface = pr2_interface

    # Create QWidget object (this is object that represents the window
    # that the user actually sees).
    self._widget = QWidget()

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
    user_interface = pr2_interface._context
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

  def _handle_connection_info_button_clicked(self):
    self._pr2_interface.open_window(WindowTypes.ConnectionWindow)

  def _handle_data_graphs_button_clicked:
    self._pr2_interface.open_window(WindowTypes.DataGraphsWindow)

  def _handle_sensor_visualizer_button_clicked:
    self._pr2_interface.open_window(WindowTypes.SensorVisualizerWindow)

  def _handle_run_programs_button_clicked:
    self._pr2_interface.open_window(WindowTypes.RunProgramsWindow)

  def _handle_robot_visualizer_button_clicked:
    self._pr2_interface.open_window(WindowTypes.RobotVisualizerWindow)

  def _handle_lifetime_statistics_button_clicked:
    self._pr2_interface.open_window(WindowTypes.LifetimeStatsWindow)

class WindowCreationManager:
  
  def __init__(self, pr2_interface):
    self._pr2_interface = pr2_interface
    self._open_window_types = set()
    self._open_window_managers = set()

  def new_window_manager(self, window_type):
    if window_type in self._open_window_types:
      print 'That window is already open'
      return

    if window_type is WindowTypes.IndexWindow:
      self._open_window_managers.add(IndexWindowManager(self._pr2_interface))
      self._open_window_types.add(window_type)
    
    else:
      raise NotImplementedError
    
  def shutdown_window(self, window_type):
    pass

  def shutdown_all_windows(self):
    for window in self._open_windows():
      self.shutdown_window(window)


class PR2Interface(Plugin):

    def __init__(self, context):
        super(PR2Interface, self).__init__(context)
        self._context = context

        # Parse command line arguments.
        # Only default ones used for now.
        self.parse_arguments(context)

        # Give QObjects reasonable names
        self.setObjectName('PR2Interface')

        # Initialize the window manager and open the Index window for the user.
        self._window_creation_manager = WindowCreationManager(self)
        self.open_window(WindowTypes.IndexWindow)

    def parse_arguments(self, context):
        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

    def get_user_interface(self):
      return self._context

    def open_window(self, window_type):
      self._window_creation_manager.new_window_manager(window_type)
      

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
