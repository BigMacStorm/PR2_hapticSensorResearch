#!/usr/bin/env python

import json
import math
import rospy

from std_msgs.msg import String

class MockPR2:

  # This function initializes the Mock PR2 which will genereate fake data similar to 
  # the actualy data that would be transmitted by the PR2  
  def __init__(self):
    # Initialize a ros publisher and initialize a new ros node
    self._pub = rospy.Publisher('PR2_data', String, queue_size=10)
    rospy.init_node('PR2', anonymous=True)

  # This is the function that will run through a loop to generate "PR2" data to be
  # analyzed by the interface
  def run(self):
    rate = rospy.Rate(10) # 10hz
    # While rospy is still running
    while not rospy.is_shutdown():
      # Create a new data_time_tick
      data_time_tick = self.create_data_time_tick()
      # Generate a json message
      message = json.dumps(data_time_tick)
      # Log the info from the json message
      rospy.loginfo(message)
      # Publish the message
      self._pub.publish(message)
      # Put the function to sleep for a moment
      rate.sleep()

  # This function generates a data_time_tick with random data and returns it
  def create_data_time_tick(self):
    # Get the current time
    t_now = rospy.get_rostime().to_nsec()

    # Make a new map called data_time_tick
    data_time_tick = {}

    # add to the map the time it was created
    data_time_tick['t_send'] = t_now

    # generate the x, y, and z position information using sin, cos, and tan
    data_time_tick['x'] = 10*math.sin(t_now)
    data_time_tick['y'] = 10*math.cos(t_now)
    data_time_tick['z'] = 10*math.tan(t_now)

    # generate the force, fluid pressure, microvibration, temperature, and thermal flux
    # information using sin, cos, and tan
    data_time_tick['force'] = 10*math.sin(t_now)
    data_time_tick['fluid_pressure'] = 10*math.cos(t_now)
    data_time_tick['microvibration'] = 10*math.tan(t_now)
    data_time_tick['temperature'] = 10*math.sin(t_now)
    data_time_tick['thermal_flux'] = 10*math.cos(t_now)
    
    # return the map of data
    return data_time_tick

# if in main, create the mock_pr2 and run it
if __name__ == '__main__':
    try:
      mock_pr2 = MockPR2()
      mock_pr2.run()
    except rospy.ROSInterruptException:
      pass
