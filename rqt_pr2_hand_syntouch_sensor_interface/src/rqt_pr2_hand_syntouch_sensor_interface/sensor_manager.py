import rospy
from std_msgs.msg import String

from .data_time_tick import DataTimeTick

class SensorManager:
  
  def __init__(self, pr2_interface):
    self._pr2_interface = pr2_interface
    self._data = [0]
    rospy.Subscriber("PR2_data", String, self.receive_data)

  def receive_data(self, data):
      rospy.loginfo("Received data from node with caller id " + 
                     rospy.get_caller_id())
      self.update_data(data)

  # Save data here for possibly writing to file in the future.
  def update_data(self, data):
    data_time_tick = DataTimeTick(data.data)
    self._data.append(data_time_tick)

  def get_data(self):
    return self._data[-1]
    
  def get_data_range(self, t0, t1=None):
    return []
