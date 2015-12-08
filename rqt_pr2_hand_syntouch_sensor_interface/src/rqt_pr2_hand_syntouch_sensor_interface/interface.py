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

    def __init__(self, context):
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

    def get_most_recent_data(self):
      return self._sensor_manager.get_data()

    def shutdown_plugin(self):
      self._window_creation_manager.shutdown_all_windows()

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
