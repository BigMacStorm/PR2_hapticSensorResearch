#!/usr/bin/env python

import json
import math
import rospy

from std_msgs.msg import String

MESSAGE_RATE = 50 # 50 hz, send 50 messages a second.

class MockPR2:

  # This function initializes the Mock PR2 ROS Node, causing it
  # to publish data in a similar manner to the PR2 would. After initialization
  # sin/cos/tan data is published to the ros topic 'PR2_data' at 10 hz.
  # Args:
  #   period: The period of the trigonmetric function data published, 
  #           in seconds.
  def __init__(self, period):
    # Initialize a ros publisher and initialize a new ros node
    self._pub = rospy.Publisher('PR2_data', String, queue_size=10)
    self._counter = 0
    self._period = period
    rospy.init_node('PR2', anonymous=True)

  # This is the function that will run through a loop to generate "PR2" data to be
  # analyzed by the interface
  def run(self):
    rate = rospy.Rate(MESSAGE_RATE)
    # While rospy is still running
    while not rospy.is_shutdown():
      # Create a new data_time_tick
      data_time_tick = self.create_data_time_tick()
      # Generate a json message
      message = json.dumps(data_time_tick)
      # Log the info from the json message
      # rospy.loginfo(message)   (disabled because annoying)

      # Publish the message
      self._pub.publish(message)
      # Put the function to sleep for a moment
      rate.sleep()

  # This function creates a pulse wave based on a pulse wave with 13 parts.
  def get_pulse_tick(self):
    val = (self._counter // (MESSAGE_RATE // 10)) % 13
    if (val == 0 or val == 1 or val == 3 or val == 4 or val == 8 or val == 9 or
        val == 11 or val == 12):
      return 0
    elif (val == 2 or val == 10):
      return 1
    elif (val == 5 or val == 7):
      return -0.5
    else:
      return 3

  # This function generates a data_time_tick with (essentially) random data
  # that is periodic. The periodicity can be set using self._period
  def create_data_time_tick(self):
    # Get the current time
    t_now = rospy.get_rostime().to_nsec()

    # Make a new map called data_time_tick
    data_time_tick = {}

    # add to the map the time it was created
    data_time_tick['t_send'] = t_now

    # convert nanoseconds to seconds
    t_now = t_now / 1e9

    # generate the x, y, and z position information using sin, cos, and tan
    data_time_tick['x'] = 10*math.sin(t_now*2*math.pi/self._period)
    data_time_tick['y'] = 10*math.cos(t_now*2*math.pi/self._period)
    data_time_tick['z'] = 10*math.tan(t_now*2*math.pi/self._period)

    # generate the force, fluid pressure, microvibration, temperature, and thermal flux
    # information using sin, cos, and tan
    data_time_tick['force'] = self.get_pulse_tick()
    data_time_tick['fluid_pressure'] = 10*math.cos(t_now*2*math.pi/self._period)
    data_time_tick['microvibration'] = 10*math.tan(t_now*2*math.pi/self._period)
    data_time_tick['temperature'] = 10*math.sin(t_now*2*math.pi/self._period)
    data_time_tick['thermal_flux'] = 10*math.cos(t_now*2*math.pi/self._period)

    self._counter += 1
    
    # return the map of data
    return data_time_tick

if __name__ == '__main__':
    try:
      print 'Input the desired period of the data (in seconds): '
      period = float(raw_input())
      mock_pr2 = MockPR2(period)
      mock_pr2.run()
    except rospy.ROSInterruptException:
      pass
