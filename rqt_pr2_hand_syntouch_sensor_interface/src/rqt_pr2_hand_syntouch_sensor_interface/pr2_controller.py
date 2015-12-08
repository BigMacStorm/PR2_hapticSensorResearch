
class PR2_controller:

  # This function will initialize the PR2 controller. This class will be responsible
  # for controlling the movement of the PR2
  # NOTE: This is not actually implemented right now since we are not actually controlling
  #       the PR2 at this time
  def __init__(self, pr2_interface):
    self._pr2_interface = pr2_interface

  # This function will send a signal to the PR2 to make it move to the given coordinates
  # Args:
  #	x: the x coordinate the PR2 will move to
  #	y: the y coordinate the PR2 will move to
  #	z: the z coordinate the PR2 will move to
  # NOTE: This is not actually implemented right now since we are not actually controlling
  #       the PR2 at this time
  def signal_move_pr2_robot(x, y, z):
    pass

  # This function will send a signal to the PR2 to make it move its left arm to a given position
  # Args:
  #	shoulder_pan: the pan value of the left shoulder
  #	shoulder_tilt: the tilt value of the left shoulder
  #	upper_arm_roll: the roll value of the left upper arm
  # NOTE: This is not actually implemented right now since we are not actually controlling
  #       the PR2 at this time
  def signal_move_left_arm(shoulder_pan, shoulder_tilt, upper_arm_roll):
    pass
 
  # This function will send a signal to the PR2 to make it move its right arm to a given position
  # Args:
  #	shoulder_pan: the pan value of the right shoulder
  #	shoulder_tilt: the tilt value of the right shoulder
  #	upper_arm_roll: the roll value of the right upper arm
  # NOTE: This is not actually implemented right now since we are not actually controlling
  #       the PR2 at this time
  def signal_move_right_arm(shoulder_pan, shoulder_tilt, upper_arm_roll):
    pass

  # This function will send a signal to the PR2 to make it move its left hand to a given position
  # Args:
  #	elbow_flex: the elbow flex value of the left elbow
  #	forearm_roll: the roll value of the left forearm
  #	wrist_pitch: the pitch value of the left wrist
  #	wrist_roll: the roll value of the left wrist
  # NOTE: This is not actually implemented right now since we are not actually controlling
  #       the PR2 at this time
  def signal_move_left_hand(elbow_flex, forearm_roll, wrist_pitch, wrist_roll):
    pass

   # This function will send a signal to the PR2 to make it move its right hand to a given position
  # Args:
  #	elbow_flex: the elbow flex value of the right elbow
  #	forearm_roll: the roll value of the right forearm
  #	wrist_pitch: the pitch value of the right wrist
  #	wrist_roll: the roll value of the right wrist
  # NOTE: This is not actually implemented right now since we are not actually controlling
  #       the PR2 at this time
  def signal_move_right_hand(elbow_flex, forearm_roll, wrist_pitch, wrist_roll):
    pass

  # This function will signal send a signal to the PR2 to make it close its left hand to a given 
  # pressure limit
  #	pressure_limit: The max pressure the left hand should exert on an object with the hand closed
  # NOTE: This is not actually implemented right now since we are not actually controlling
  #       the PR2 at this time
  def signal_close_left_hand(pressure_limit):
    pass

  # This function will signal send a signal to the PR2 to make it close its right hand to a given 
  # pressure limit
  #	pressure_limit: The max pressure the left hand should exert on an object with the hand closed
  # NOTE: This is not actually implemented right now since we are not actually controlling
  #       the PR2 at this time
  def signal_close_right_hand(pressure_limit):
    pass
