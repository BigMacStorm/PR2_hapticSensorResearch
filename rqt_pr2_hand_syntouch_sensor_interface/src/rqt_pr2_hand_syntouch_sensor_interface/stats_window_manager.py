import os
import rospy
import rospkg
import threading

from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

from .window_manager import WindowManager

class LifetimeStatsWindowManager(WindowManager):

  def __init__(self, pr2_interface):
    # Initialize the WindowManager base class. The WindowManager class
    # creates the _widget object that will be used by this window and
    # guarantees successful shutdown of rqt upon program termination.
    super(LifetimeStatsWindowManager, self).__init__(pr2_interface)

    # Initialize variables necessary for keeping track of time stats.
    self._startup_time = rospy.get_rostime().to_nsec()
    self._disconnected = True

    # Get path to UI file which should be in the "resource" folder of this package
    ui_file = os.path.join(
        rospkg.RosPack().get_path(
            'rqt_pr2_hand_syntouch_sensor_interface'), 'resource', 'lifetimestatistics.ui')

    # Extend the widget with all attributes and children from UI file
    loadUi(ui_file, self._widget)

    # Give QObjects reasonable names
    self._widget.setObjectName('LifetimeStatsWindow')

    self._widget.setWindowTitle('Lifetime Statistics')

    # Temporary workaround: Don't open up the window yet here, we just need
    # to initialize the lifetime statistics tracking. When the user wants this
    # window to be opened, the 'self.reopen' function will be called by the
    # window creation manager.
    # TODO: Remove this work around.

    # user_interface = pr2_interface.get_user_interface()
    # user_interface.add_widget(self._widget)

    self._worker = threading.Thread(target=self.track_lifetime_stats)
    self._worker.start()

  def track_lifetime_stats(self):
    rate = rospy.Rate(5) # 5hz
    last_data_point = None
    while not rospy.is_shutdown() and not self._destroyed:
      current_data_point = self._pr2_interface.get_most_recent_data()
      if last_data_point == current_data_point or not last_data_point:
        self._disconnected = True
      else:
        if self._disconnected:
          self._last_connected_time = rospy.get_rostime().to_nsec()
        self._disconnected = False
      self.update_labels()
      rate.sleep()
      last_data_point = current_data_point

  def update_labels(self):
    self.update_pr2_interface_uptime_label()
    self.update_connection_uptime_label()
    self.update_data_samples_label()
    self.update_average_sample_rate_label()
    self.update_actions_performed_label()
    self.update_most_performed_action_label()
    
  def update_data_samples_label(self):
    total_data_samples = self._pr2_interface.count_data_time_ticks()
    self._widget.DataSamplesLabel.setText('%d' % total_data_samples)

  def update_average_sample_rate_label(self):
    total_data_samples = self._pr2_interface.count_data_time_ticks()

    # Get the pr2 interface uptime in nanoseconds.
    interface_uptime = (
        rospy.get_rostime().to_nsec() - self._startup_time)

    # Convert to seconds.
    interface_uptime = (interface_uptime // 1e9)

    # Need to check if interface uptime is 0 to avoid division by 0.
    if interface_uptime == 0:
      data_samples_per_sec = 0
    else:
      data_samples_per_sec = total_data_samples / interface_uptime

    self._widget.AverageSampleRateLabel.setText('%d hz' % data_samples_per_sec)

  def update_actions_performed_label(self):
    pass

  def update_most_performed_action_label(self):
    pass

  def update_pr2_interface_uptime_label(self):
    t_now = rospy.get_rostime().to_nsec()

    # Get the pr2 interface uptime in nanoseconds.
    interface_uptime = (
        rospy.get_rostime().to_nsec() - self._startup_time)
                              
    # Convert to seconds.
    interface_uptime = (interface_uptime // 1e9)

    # divide up the seconds into hours, minutes, seconds
    seconds = interface_uptime % 60
    minutes = (interface_uptime // 60) % 60
    hours = interface_uptime // 3600
    self._widget.PR2UptimeLabel.setText('%02d:%02d:%02d' 
        % (hours, minutes, seconds)) 

  def update_connection_uptime_label(self):
    t_now = rospy.get_rostime().to_nsec()
    
    # Get the connection uptime in nanoseconds, and also
    # prepare the disconnected string for the formatting below..
    if self._disconnected:
      disconnected_string = '(Disconnected)'
      connection_uptime = 0
    else:
      disconnected_string = ''
      connection_uptime = (
          rospy.get_rostime().to_nsec() - self._last_connected_time)
                              
    # Convert to seconds.
    connection_uptime = (connection_uptime // 1e9)

    # divide up the seconds into hours, minutes, seconds
    seconds = connection_uptime % 60
    minutes = (connection_uptime // 60) % 60
    hours = connection_uptime // 3600
    self._widget.ConnectionUptimeLabel.setText('%02d:%02d:%02d %s' 
        % (hours, minutes, seconds, disconnected_string)) 

  def reopen(self):
    user_interface = self._pr2_interface.get_user_interface()
    user_interface.add_widget(self._widget)
