import os
import rospy
import rospkg
import threading

from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

from .action_types import ActionTypes
from .window_manager import WindowManager

# Class that handles the place object window and ui
class ObjectHandoffWindowManager(WindowManager):

  # Initialize the WindowManager base class. The WindowManager class
  # creates the _widget object that will be used by this window and
  # guarantees successful shutdown of rqt upon program termination.
  # Args:
  #	pr2_interface: the single pr2 interface plug in object
  def __init__(self, pr2_interface):

    super(ObjectHandoffWindowManager, self).__init__(pr2_interface)

    # Get path to UI file which should be in the "resource" folder of this package
    ui_file = os.path.join(
        rospkg.RosPack().get_path(
            'rqt_pr2_hand_syntouch_sensor_interface'), 'resource', 
            'objecthandoff.ui')

    # Extend the widget with all attributes and children from UI file
    loadUi(ui_file, self._widget)

    # Give QObjects reasonable names
    self._widget.setObjectName('ObjectHandoff')

    self._widget.setWindowTitle('Object Handoff Using PR2 Arm')

    # Add widget to the user interface
    user_interface = pr2_interface.get_user_interface()
    user_interface.add_widget(self._widget)

    self._widget.OutputTextBox.document().setPlainText(
        'Object should already be in the PR2 gripper\'s fingers. Press the \
         button to begin waiting for socially acceptable hand off.')
    
    # Register a listener for the place button.
    self._widget.PlaceButton.clicked.connect(self._handle_hand_off_button_clicked)

  def _socially_aware_handoff(self):
    curr_text = 'Waiting for hand off...'
    self._widget.OutputTextBox.document().setPlainText(curr_text)
    hand_off_complete = False
    pr2_controller = self._pr2_interface.get_pr2_controller()

    rate = rospy.Rate(10) # 10hz
    last_data_point = None
    threshold = 3
    while not hand_off_complete:
      current_data_point = self._pr2_interface.get_most_recent_data()
      if last_data_point and last_data_point != current_data_point:
        if abs(current_data_point.get_fluid_pressure() - 
               last_data_point.get_fluid_pressure()) > threshold:
	  curr_text += '\nHand off detected, opening hand.'
	  self._widget.OutputTextBox.document().setPlainText(curr_text)
          pr2_controller.signal_open_left_hand()
          hand_off_complete = True
          
      rate.sleep()
      last_data_point = current_data_point
      self._pr2_interface.get_most_recent_data()

    self._pr2_interface.notify_action_performed(ActionTypes.HandoffObject)
    

  def _handle_hand_off_button_clicked(self):
    self._worker = threading.Thread(target=self._socially_aware_handoff)
    self._worker.start()

  # calls the function to readd the widgets if the window was closed
  def reopen(self):
    user_interface = self._pr2_interface.get_user_interface()
    user_interface.add_widget(self._widget)
