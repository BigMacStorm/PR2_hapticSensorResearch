import os
import rospy
import rospkg
import threading

from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

from .action_types import ActionTypes
from .action_types import NUM_ACTION_TYPES

from .window_manager import WindowManager

class LifetimeStatsWindowManager(WindowManager):

  # This function will initialize the window and all widgets attached to this window.
  # Args:
  #	pr2_interface: the single pr2 interface plug in object
  def __init__(self, pr2_interface):
    # Initialize the WindowManager base class. The WindowManager class
    # creates the _widget object that will be used by this window and
    # guarantees successful shutdown of rqt upon program termination.
    super(LifetimeStatsWindowManager, self).__init__(pr2_interface)

    # Initialize variables necessary for keeping track of time stats.
    self._startup_time = rospy.get_rostime().to_nsec()
    self._disconnected = True

    # Initialize action stats
    self._action_occurence_counter = [0 in xrange(NUM_ACTION_TYPES)]
    self._total_action_count = 0

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

    # Create a thread that will begin tracking the lifetime statistics
    self._worker = threading.Thread(target=self.track_lifetime_stats)
    self._worker.start()

  # This function will be alled be a thread to keep track of the lifetime statistics
  # of the PR2. This function will also update the lifetime statistics window
  def track_lifetime_stats(self):
    # Initialize local variables.
    rate = rospy.Rate(5) # 5hz
    last_data_point = None

    # While rospy is still running and this window has not been destroyed.
    while not rospy.is_shutdown() and not self._destroyed:
      # Get the most recent data from the pr2 interface.
      current_data_point = self._pr2_interface.get_most_recent_data()

      # If the newest data recieved is equal to the last set of data recieved
      # then the pr2 is not "connected". Else the newest data is not equal to the 
      # last set of data recieved and the pr2 is "connected" and the functions updates
      # the labels of the window and the last connected time. This is currently
      # just for testing purposes to use with the mock_pr2.py which is simulating
      # incoming pr2 data.
      if last_data_point == current_data_point or not last_data_point:
        self._disconnected = True
      else:
        if self._disconnected:
          self._last_connected_time = rospy.get_rostime().to_nsec()
        self._disconnected = False
      self.update_labels()
      rate.sleep()
      last_data_point = current_data_point

  # This function will run through all of the lables on the Lifetime statistics window
  # and update them
  def update_labels(self):
    self.update_pr2_interface_uptime_label()
    self.update_connection_uptime_label()
    self.update_data_samples_label()
    self.update_average_sample_rate_label()
    self.update_actions_performed_label()
    self.update_most_performed_action_label()
 
  # This function is used to update the data samples label specifically because it has to 
  # keep track of the total amount of data samples recieved over the lifetime of the
  # interface of the PR2 
  def update_data_samples_label(self):
    total_data_samples = self._pr2_interface.count_data_time_ticks()
    self._widget.DataSamplesLabel.setText('%d' % total_data_samples)

  # This function is used to update the average data samples rate label specifically
  # because it  has to keep track of the average rate  of data samples recieved over
  # the lifetime of the interface of the PR2 
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

  # This function is called to notify the stats manager that an action has been
  # been performed. If the window performing the action forgets to call this
  # function then the action won't be counted in the stats.
  def notify_action_performed(self, action_type):
    print "asdfasdfasdfasdf"
    print action_type
    print NUM_ACTION_TYPES
    print self._action_occurence_counter
    self._action_occurence_counter[action_type] += 1
    self._total_action_count += 1

  # This function is used to update the actions performed label specifically because
  # it has to add up the total number of actions performed by the PR2
  def update_actions_performed_label(self):
    self._widget.ActionsPerformedLabel.setText('%d' % self._total_action_count)

  # This function is used to update the most performed action label specifically because
  # it has to calculate which action was performed the most.
  def update_most_performed_action_label(self):
    most_performed_action = None
    most_performed_action_count = 0
    for i in range(len(self._action_occurence_counter)):
      if self._action_occurence_counter[i] > most_performed_action_count:
        most_performed_action = i
        most_performed_action_count = self._action_occurence_counter[i]

    if most_performed_action == ActionTypes.LiftObject:
      action_string = 'LiftObject'
    elif most_performed_action == ActionTypes.PlaceObject:
      action_string = 'PlaceObject'
    elif most_performed_action == ActionTypes.RotateObject:
      action_string = 'RotateObject'
    elif most_performed_action == ActionTypes.SwitchHands:
      action_string = 'SwitchHands'
    else:
      action_string = 'None'

    self._widget.MostPerformedActionLabel.setText(action_string)
     
  # This function is used to update the interface uptime label specifically because it
  # has to calculate the total amount of time that the interface has ran over its lifetime.
  def update_pr2_interface_uptime_label(self):
    # get the current time
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

  # This function is used to update the connection uptime label specifically because
  # it has to calculate the total amount of time that the PR2 has been connected to
  # the interface over its lifetime.
  def update_connection_uptime_label(self):
    # get the current time
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

  # This function will reopen this window.
  def reopen(self):
    user_interface = self._pr2_interface.get_user_interface()
    user_interface.add_widget(self._widget)
