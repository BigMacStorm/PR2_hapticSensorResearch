import os
import rospy
import rospkg
import sys
import threading

from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

from .window_manager import WindowManager

# class that handles the connection window and ui
class ConnectionWindowManager(WindowManager):

  # Initialize the WindowManager base class. The WindowManager class
  # creates the _widget object that will be used by this window and
  # guarantees successful shutdown of rqt upon program termination.
  # This function will initialize the window and all widgets attached to this window.
  # Args:
  #	   pr2_interface: the single pr2 interface plug in object
  def __init__(self, pr2_interface):

    super(ConnectionWindowManager, self).__init__(pr2_interface)

    # Get path to UI file which should be in the "resource" folder of this package
    ui_file = os.path.join(
        rospkg.RosPack().get_path(
            'rqt_pr2_hand_syntouch_sensor_interface'), 'resource', 'connectioninfo.ui')

    # Extend the widget with all attributes and children from UI file
    loadUi(ui_file, self._widget)

    # Give QObjects reasonable names
    self._widget.setObjectName('ConnectionInfoWindow')

    self._widget.setWindowTitle(
        'Connection Info')

    # Add widget to the user interface
    user_interface = pr2_interface.get_user_interface()
    user_interface.add_widget(self._widget)

    # Since this window has text that needs to be updated in real time
    # A thread will need to be created that will handle updating the 
    # labels.
    self._worker = threading.Thread(target=self.update_labels)
    self._worker.start()

  # Checks on the connection with the PR2 and updates the displayed info
  # if there isn't any new data coming in it calls set_label_text_disconnected
  # otherwise it calls set_label_text_connected
  def update_labels(self):
    rate = rospy.Rate(5) # 5hz
    last_data_point = None
    while not rospy.is_shutdown() and not self._destroyed:
      current_data_point = self._pr2_interface.get_most_recent_data()
      if last_data_point == current_data_point or not last_data_point:
        self.set_label_text_disconnected()
      else:
        self.set_label_text_connected()

      rate.sleep()
      last_data_point = current_data_point

  # sets all of the displayed information to show that
  # the PR2 is disconnected
  def set_label_text_disconnected(self):
    self._widget.PR2StatusLabel.setText("PR2 Status: Disconnected")
    self._widget.SyntouchStatusLabel.setText(
        "Syntouch (fingers) Status: Disconnected")
    self._widget.CurrentStateLabel.setText('Current state: -1 (Disconnected)')
    self._widget.UploadRateLabel.setText('Upload rate: 0 bytes/s')
    self._widget.DownloadRateLabel.setText('Download rate: 0 bytes/s')
    self._widget.LatencyLabel.setText('Latency: N/a ms')

  # sets all of the displayed information to show that
  # the PR2 is connected
  def set_label_text_connected(self):
    self._widget.PR2StatusLabel.setText("PR2 Status: Connected")
    self._widget.SyntouchStatusLabel.setText(
        "Syntouch (fingers) Status: Connected")
    self._widget.CurrentStateLabel.setText('Current state: 0 (Idle)')

    upload_rate = 0
    self._widget.UploadRateLabel.setText('Upload rate: %d bytes/s' % upload_rate)

    # Retrieve data from the sensor manager to be able to calculate the
    # upload rate, download rate, and latency.

    # Retrieve an array of data time ticks for the last second.
    data_time_ticks = self._pr2_interface.get_data_range(-1)
    
    # Get the size (in bytes).
    download_rate = sys.getsizeof(data_time_ticks)

    self._widget.DownloadRateLabel.setText('Download rate: %s bytes/s' 
        % str(download_rate))
    
    most_recent_tick = data_time_ticks[-1]
    latency = (most_recent_tick.get_t_recv() - most_recent_tick.get_t_recv())
    
    # divide by 1e6 to convert from nanoseconds to milliseconds
    self._widget.LatencyLabel.setText('Latency: %s ns (%s ms)' 
        % (str(latency), str(latency/1e6)))
