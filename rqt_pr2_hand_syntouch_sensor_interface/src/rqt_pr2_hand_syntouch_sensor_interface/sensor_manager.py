import rospy
import json
from std_msgs.msg import String

from .data_time_tick import DataTimeTick
from biotac_sensors.msg import BioTacHand
import rosjson_time

# Class to handle the ROS Node, manage sensor data retrieval from the PR2,
# store retrieved data, and provide methods for accessing data.
class SensorManager:
  
  # Initializes the SensorManager class. The SensorManager class
  # creates a list for sensor data to be placed in and sets up
  # a subscriber to take in and handle new data.
  # Args:
  # 	pr2_interface: The single pr2 interface plug in object.
  def __init__(self, pr2_interface):
    self._pr2_interface = pr2_interface
    self._data = []

    # Register the callback for receiving data.
    rospy.Subscriber("biotac_pub", BioTacHand, self.receive_data, 
                     queue_size = 1000)

  def convert_data(self, raw_data):
    raw_data = json.loads(rosjson_time.ros_message_to_json(raw_data))
    data = {}
    data['t_send'] = raw_data['bt_time']['frame_end_time']
    data['temperature'] = raw_data['bt_data'][0]['tdc_data']
    data['thermal_flux'] = raw_data['bt_data'][0]['tac_data']
    data['fluid_pressure'] = raw_data['bt_data'][0]['pdc_data']
    data['microvibration'] = raw_data['bt_data'][0]['pac_data'][0]
    data['force'] = sum(raw_data['bt_data'][0]['electrode_data'])
    data['x'] = 0
    data['y'] = 0
    data['z'] = 0
    return json.dumps(data)
   
  # A callback to be called when data is recieved from the PR2.
  # Args:
  #	  data: A std_msgs.msgs.String object holding data retrived from the
  #         PR2 robot. This data is a dictionary string (in json format).
  def receive_data(self, data):
    data = self.convert_data(data)
    rospy.loginfo("Received data from node with caller id " + 
                   rospy.get_caller_id())
    self.update_data(data)

  # Store data in memory for fast retrieval by appending the data to the 
  # end of the data list. Since data is always retrieved in-order, this
  # list of DataTimeTicks sorted by data retrieval time.
  # Args:
  #	  data: A std_msgs.msgs.String object holding data retrived from the
  #         PR2 robot. This data is a dictionary string (in json format).
  def update_data(self, data):
    data_time_tick = DataTimeTick(data, rospy.get_rostime().to_nsec())
    self._data.append(data_time_tick)

  # Return the most recent time tick of data. If there is no data yet,
  # return None.
  def get_data(self):
    if self._data:
      return self._data[-1]
    else:
      return None
    
  # This function returns a sorted list of DataTimeTick objects representing
  # sensor data from an interval of time.
  #
  # Example: get_data_range(-5, -3) will return all sensor data retrieved from 5
  #          seconds ago to 3 seconds ago.
  # Args:
  # 	t0: The start time offest of the requested time interval, in seconds.
  # 	t1: The end time offest of the requested time interval, in seconds.
  def get_data_range(self, t0, t1=0):
    # Handle the edge case where there is no data yet.
    if not self._data:
      return []

    # Calculate t0 and t1 in absolute time.
    t0_time = rospy.get_rostime().to_nsec() + t0 * 1e9
    t1_time = rospy.get_rostime().to_nsec() + t1 * 1e9

    # Handle the edge case where there is no data in [t0_time,t1_time]
    if (self._data[0].get_t_recv() > t1_time or
        self._data[-1].get_t_recv() < t0_time):
      return []

    left = 0
    right = len(self._data) - 1

    # Find the first DataTimeTick in the data list after t0 by performing 
    # a binary search.
    while(left < right):
      mid = (left + right) / 2
      if self._data[mid].get_t_recv() < t0_time:
        left = mid + 1
      else:
        right = mid

    t0_index = mid

    # if no t1 is None, return all data retrieved since t0.
    if t1 is None:
      return self._data[t0_index:]
  
    t1_time = rospy.get_rostime().to_nsec() + t1 * 1e9
    left = 0
    right = len(self._data) - 1

    # Find the last DataTimeTick in the data list before t1 by performing 
    # a binary search.
    while(left < right):
      mid = (left + right) / 2
      if self._data[mid].get_t_recv() < t1_time:
        left = mid + 1
      else:
        right = mid
    if self._data[mid].get_t_recv() > t1_time:
      mid -= 1

    t1_index = mid

    return self._data[t0_index:t1_index+1]

  # This function returns the number of DataTimeTicks stored, for statistic
  # purposes.
  def count_data_time_ticks(self):
    return len(self._data)
