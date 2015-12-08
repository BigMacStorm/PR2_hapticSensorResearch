import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

from .sensor_manager import SensorManager
from .window_creation_manager import WindowCreationManager
from .window_types import WindowTypes


class PR2Interface(Plugin):

  # Initialize the PR2 interface class. The PR2Interface class sets the 
  # context for the plugin as well as initializing the other managers
  # needed by the interface and opening up the index window. The managers
  # that this initializes includes the sensor manager and the window creation
  # creation manager.
  # Args:
  # 	context: provides information to the plugin and exposes the methods
  #	         to the underlying RQT framework and the plugin handler
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
    self.open_window(WindowTypes.LifetimeStatsWindow)

  # This function will process standalone plugin command-line arguments
  # Args:
  # 	context: this function is passed the current context
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

  # This function will call the window creation manager to open a new window 
  # of a given type
  # Args:
  # 	window_type: This is the type of window that the window creation manager
  #	             will open
  def open_window(self, window_type):
    self._window_creation_manager.new_window_manager(window_type)

  # This function will return the most recent data gathered from the sensor manager
  def get_most_recent_data(self):
    return self._sensor_manager.get_data()

  # This function will return a sorted list of data time ticks between the offets
  # of t0 and t1
  # Args:
  # 	t0: the start time offest of the requested time period
  # 	t1: the end time offset of the requested time period
  def get_data_range(self, t0, t1=None):
    return self._sensor_manager.get_data_range(t0, t1)

  # This function will return the number of data time ticks that have occurred
  # throughout the lifetime of the pr2 interface.
  def count_data_time_ticks(self):
    return self._sensor_manager.count_data_time_ticks()

  # This function will shutdown the window creation manager as well as all of the
  # windows currently open
  def shutdown_plugin(self):
    self._window_creation_manager.shutdown_all_windows()

  # This function will save settings that the user has chosen
  # Args:
  # 	plugin_settings: the current settings as they were before being changed
  # 	instance_settings: the settings that the user has currently changed
  # NOTE: This function is not actually being implement yet
  def save_settings(self, plugin_settings, instance_settings):
    # TODO save intrinsic configuration, usually using:
    # instance_settings.set_value(k, v)
    pass

  # This function will restore settings that have been changed to the default settings
  # Args:
  # 	plugin_settings: the current settings as they were before being changed
  # 	instance_settings: the settings that the user has currently changed
  # NOTE: This function is not actually being implement yet
  # This function will process standalone plugin command-line arguments
  def restore_settings(self, plugin_settings, instance_settings):
    # TODO restore intrinsic configuration, usually using:
    # v = instance_settings.value(k)
    pass

  # This function will process standalone plugin command-line arguments
  #def trigger_configuration(self):
    # Comment in to signal that the plugin has a way to configure
    # This will enable a setting button (gear icon) in each dock widget title bar
    # Usually used to open a modal configuration dialog
