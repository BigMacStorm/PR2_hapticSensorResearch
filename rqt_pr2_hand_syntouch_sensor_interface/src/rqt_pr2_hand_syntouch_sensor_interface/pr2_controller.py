import rospy
import actionlib
import roslib
import threading

roslib.load_manifest('rospy')
roslib.load_manifest('actionlib')
roslib.load_manifest('pr2_controllers_msgs')
from actionlib_msgs.msg import *
from pr2_controllers_msgs.msg import *

HAND_OPEN_POSITION = .088
HAND_CLOSE_POSITION = 0

class PR2Controller:

  # This function will initialize the PR2 controller. This class will be responsible
  # for controlling the movement of the PR2
  # NOTE: This is not actually implemented right now since we are not actually controlling
  #       the PR2 at this time
  def __init__(self, pr2_interface):
    self._pr2_interface = pr2_interface
    self._client = actionlib.SimpleActionClient(
        'l_gripper_controller/gripper_action', Pr2GripperCommandAction)
    self._client.wait_for_server()

  # This function will send a signal to the PR2 to make it move to the given coordinates
  # Args:
  #	x: the x coordinate the PR2 will move to
  #	y: the y coordinate the PR2 will move to
  #	z: the z coordinate the PR2 will move to
  # NOTE: This is not actually implemented right now since we are not actually controlling
  #       the PR2 at this time
  def signal_move_pr2_robot(self, x, y, z):
    pass

  # This function will send a signal to the PR2 to make it move its left arm to a given position
  # Args:
  #	shoulder_pan: the pan value of the left shoulder
  #	shoulder_tilt: the tilt value of the left shoulder
  #	upper_arm_roll: the roll value of the left upper arm
  # NOTE: This is not actually implemented right now since we are not actually controlling
  #       the PR2 at this time
  def signal_move_left_arm(self, shoulder_pan, shoulder_tilt, upper_arm_roll):
    pass
 
  # This function will send a signal to the PR2 to make it move its right arm to a given position
  # Args:
  #	shoulder_pan: the pan value of the right shoulder
  #	shoulder_tilt: the tilt value of the right shoulder
  #	upper_arm_roll: the roll value of the right upper arm
  # NOTE: This is not actually implemented right now since we are not actually controlling
  #       the PR2 at this time
  def signal_move_right_arm(self, shoulder_pan, shoulder_tilt, upper_arm_roll):
    pass

  # This function will send a signal to the PR2 to make it move its left hand to a given position
  # Args:
  #	elbow_flex: the elbow flex value of the left elbow
  #	forearm_roll: the roll value of the left forearm
  #	wrist_pitch: the pitch value of the left wrist
  #	wrist_roll: the roll value of the left wrist
  # NOTE: This is not actually implemented right now since we are not actually controlling
  #       the PR2 at this time
  def signal_move_left_hand(self, elbow_flex, forearm_roll, wrist_pitch, wrist_roll):
    pass

   # This function will send a signal to the PR2 to make it move its right hand to a given position
  # Args:
  #	elbow_flex: the elbow flex value of the right elbow
  #	forearm_roll: the roll value of the right forearm
  #	wrist_pitch: the pitch value of the right wrist
  #	wrist_roll: the roll value of the right wrist
  # NOTE: This is not actually implemented right now since we are not actually controlling
  #       the PR2 at this time
  def signal_move_right_hand(self, elbow_flex, forearm_roll, wrist_pitch, wrist_roll):
    pass


  def close_left_hand(self, position=0.0):
    self._client.send_goal(Pr2GripperCommandGoal(
            Pr2GripperCommand(position = position, max_effort = 25)))
    self._client.wait_for_result()

    result = self._client.get_result()
    did = []
    if self._client.get_state() != GoalStatus.SUCCEEDED:
        did.append("failed")
    else:
        if result.stalled: did.append("stalled")
        if result.reached_goal: did.append("reached goal")
    print ' and '.join(did)

  def open_left_hand(self):
    self._client.send_goal(Pr2GripperCommandGoal(
            Pr2GripperCommand(position = 0.088, max_effort = 25)))
    self._client.wait_for_result()

    result = self._client.get_result()
    did = []
    if self._client.get_state() != GoalStatus.SUCCEEDED:
        did.append("failed")
    else:
        if result.stalled: did.append("stalled")
        if result.reached_goal: did.append("reached goal")
    print ' and '.join(did)

  # This function will signal send a signal to the PR2 to make it close its left hand to a given 
  # pressure limit
  #	pressure_limit: The max pressure the left hand should exert on an object with the hand closed
  # NOTE: This is not actually implemented right now since we are not actually controlling
  #       the PR2 at this time
  def signal_close_left_hand(self, position=0.0):
    threading.Thread(target=self.close_left_hand,
                     kwargs={'position':position}).start()
    
  def signal_open_left_hand(self):
    threading.Thread(target=self.open_left_hand).start()

  # This function will signal send a signal to the PR2 to make it close its right hand to a given 
  # pressure limit
  #	pressure_limit: The max pressure the left hand should exert on an object with the hand closed
  # NOTE: This is not actually implemented right now since we are not actually controlling
  #       the PR2 at this time
  def signal_close_right_hand(self, pressure_limit):
    pass
