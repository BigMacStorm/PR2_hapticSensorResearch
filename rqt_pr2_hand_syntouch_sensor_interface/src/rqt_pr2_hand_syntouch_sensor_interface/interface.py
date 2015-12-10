import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

from .sensor_manager import SensorManager
from .window_creation_manager import WindowCreationManager
from .window_types import WindowTypes

# This is the overaching class and code entroy point for the PR2 interface. There
# will only be one instance of this class for the PR2 Interface.
class PR2Interface(Plugin):

  # Initialize the PR2 interface class/plugin.
  # Args:
  # 	context: The qt_gui.plugin_context.PluginContext object for the
  #            PR2 interface plugin, implementing the rqt api as described in
  #            the API Overview linked below.
  #            http://wiki.ros.org/rqt/Reviews/2012-06-20_API_Review
  def __init__(self, context):
    # Initialize the Plugin base class.
    super(PR2Interface, self).__init__(context)
    self._context = context

    # Parse command line arguments.
    # Only default ones used for now.
    self.parse_arguments(context)

    # Give QObjects reasonable names
    self.setObjectName('PR2Interface')

    # Create a sensor manager object that will begin listening to data
    # coming from the PR2 and the BioTac sensors.
    self._sensor_manager = SensorManager(self)

    # Initialize the window manager and open the Index window for the user.
    self._window_creation_manager = WindowCreationManager(self)
    self.open_window(WindowTypes.IndexWindow)

    # Initialize the LifetimeStatsWindowManager, as it is necessary that it
    # is initialized in order to record statistics.
    # TODO: Provide a way to do this without calling open_window, as it is
    #       mislieading (the window isn't actually opened).
    self.open_window(WindowTypes.LifetimeStatsWindow)

  # This function will process plugin command-line arguments.
  # Args:
  # 	context: The PluginContext object representing the PR2 Interface.
  def parse_arguments(self, context):
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

  # This function will return the user the user interface from the current context
  def get_user_interface(self):
    return self._context

  # This function will signal the window creation manager to open a new window 
  # of a given type.
  # Args:
  # 	window_type: A WindowType indicating the type of window manager to open.
  def open_window(self, window_type):
    self._window_creation_manager.new_window_manager(window_type)

  # This function returns the most recent DataTimeTick object retrieved
  # from the PR2.
  def get_most_recent_data(self):
    return self._sensor_manager.get_data()

  # This function returns a sorted list of DataTimeTick objects representing an
  # sensor data from an interval of time.
  #
  # Example: get_data_range(-5, -3) will return all sensor data retrieved from 5
  #          seconds ago to 3 seconds ago.
  # Args:
  # 	t0: The start time offest of the requested time interval, in seconds.
  # 	t1: The end time offest of the requested time interval, in seconds.
  def get_data_range(self, t0, t1=0):
    return self._sensor_manager.get_data_range(t0, t1)

  # This function returns the count of DataTimeTicks that have been
  # retrieved throughout the lifetime of the pr2 interface.
  def count_data_time_ticks(self):
    return self._sensor_manager.count_data_time_ticks()

  # This function shuts down the window creation manager as well as all of the
  # windows currently open.
  def shutdown_plugin(self):
    self._window_creation_manager.shutdown_all_windows()

  # This function will notify the LifetimeStatsManager object of an action having
  # been completed.
  # Args:
  #    action_type: The ActionType representing the action completed.
  def notify_action_performed(self, action_type):
    open_windows = self._window_creation_manager._open_windows
    LifetimeStatsWindow = (open_windows[WindowTypes.LifetimeStatsWindow])
    LifetimeStatsWindow.notify_action_performed(action_type)

  # This function will save settings that the user has chosen
  # Args:
  # 	plugin_settings: the current settings as they were before being changed
  # 	instance_settings: the settings that the user has currently changed
  def save_settings(self, plugin_settings, instance_settings):
    # TODO save intrinsic configuration, usually using:
    # instance_settings.set_value(k, v)
    pass

  # This function will restore settings that have been changed to the default settings
  # Args:
  # 	plugin_settings: the current settings as they were before being changed
  # 	instance_settings: the settings that the user has currently changed
  def restore_settings(self, plugin_settings, instance_settings):
    # TODO restore intrinsic configuration, usually using:
    # v = instance_settings.value(k)
    pass

  # This function will process standalone plugin command-line arguments
  #def trigger_configuration(self):
    # Comment in to signal that the plugin has a way to configure
    # This will enable a setting button (gear icon) in each dock widget title bar
    # Usually used to open a modal configuration dialog
