#!/usr/bin/env python

import json
import math
import rospy

from std_msgs.msg import String

class MockPR2:
  
  def __init__(self):
    self._pub = rospy.Publisher('PR2_data', String, queue_size=10)
    rospy.init_node('PR2', anonymous=True)

  def run(self):
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
      data_time_tick = self.create_data_time_tick()
      message = json.dumps(data_time_tick)
      rospy.loginfo(message)
      self._pub.publish(message)
      rate.sleep()

  def create_data_time_tick(self):
    t_now = rospy.get_rostime().to_nsec()
    data_time_tick = {}
    data_time_tick['t_send'] = t_now
    data_time_tick['x'] = 10*math.sin(t_now)
    data_time_tick['y'] = 10*math.cos(t_now)
    data_time_tick['z'] = 10*math.tan(t_now)
    data_time_tick['force'] = 10*math.sin(t_now)
    data_time_tick['fluid_pressure'] = 10*math.cos(t_now)
    data_time_tick['microvibration'] = 10*math.tan(t_now)
    data_time_tick['temperature'] = 10*math.sin(t_now)
    data_time_tick['thermal_flux'] = 10*math.cos(t_now)
    return data_time_tick

if __name__ == '__main__':
    try:
      mock_pr2 = MockPR2()
      mock_pr2.run()
    except rospy.ROSInterruptException:
      pass
