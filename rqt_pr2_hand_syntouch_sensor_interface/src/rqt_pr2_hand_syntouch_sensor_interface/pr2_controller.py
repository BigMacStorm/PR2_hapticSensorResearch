
class PR2_controller:

  def __init__(self, pr2_interface):
    self._pr2_interface = pr2_interface

  def signal_move_pr2_robot(x, y, z):
    pass

  def signal_move_left_arm(shoulder_pan, shoulder_tilt, upper_arm_roll):
    pass
  
  def signal_move_right_arm(shoulder_pan, shoulder_tilt, upper_arm_roll):
    pass

  def signal_move_left_hand(elbow_flex, forearm_roll, wrist_pitch, wrist_roll):
    pass

  def signal_move_right_hand(elbow_flex, forearm_roll, wrist_pitch, wrist_roll):
    pass

  def signal_close_left_hand(pressure_limit):
    pass

  def signal_close_right_hand(pressure_limit):
    pass
