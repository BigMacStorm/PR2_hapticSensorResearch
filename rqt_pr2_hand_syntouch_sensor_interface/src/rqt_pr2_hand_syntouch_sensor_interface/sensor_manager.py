import rospy
from std_msgs.msg import String

from .data_time_tick import DataTimeTick

class SensorManager:
  
  def __init__(self, pr2_interface):
    self._pr2_interface = pr2_interface
    self._data = []
    rospy.Subscriber("PR2_data", String, self.receive_data)

  def receive_data(self, data):
    rospy.loginfo("Received data from node with caller id " + 
                   rospy.get_caller_id())
    self.update_data(data)

  # Save data here for possibly writing to file in the future.
  def update_data(self, data):
    data_time_tick = DataTimeTick(data.data, rospy.get_rostime().to_nsec())
    self._data.append(data_time_tick)

  # Return the most recent time tick of data. If there is no data yet,
  # return None.
  def get_data(self):
    if self._data:
      return self._data[-1]
    else:
      return None
    
  # This function will return a range of data time ticks. t0 and 
  # t1 are times (in seconds) relative to the current time that
  # data is requested for (e.g. they should be negative).
  def get_data_range(self, t0, t1=None):
    # TODO test this function for correctness in edge cases.
    # perform a binary search for t0, t1...
    t0_time = rospy.get_rostime().to_nsec() + t0 * 1e9
    left = 0
    right = len(self._data) - 1
    while(left < right):
      mid = (left + right) / 2
      if self._data[mid].get_t_recv() < t0_time:
        left = mid + 1
      else:
        right = mid

    assert 0 <= mid and mid < len(self._data)
    t0_index = mid

    if t1 is None:
      return self._data[t0_index:]
  
    t1_time = rospy.get_rostime().to_nsec() + t1 * 1e9
    left = 0
    right = len(self._data) - 1
    while(left < right):
      mid = (left + right) / 2
      if self._data[mid].get_t_recv() < t1_time:
        left = mid + 1
      else:
        right = mid

    assert 0 <= mid and mid < len(self._data)
    t1_index = mid

    return self._data[t0_index:t1_index]
