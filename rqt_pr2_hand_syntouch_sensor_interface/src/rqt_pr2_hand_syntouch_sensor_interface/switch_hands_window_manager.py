import os
import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

from .action_types import ActionTypes
from .window_manager import WindowManager

# Class that handles the switch hands window and ui
class SwitchHandsWindowManager(WindowManager):

  # Initialize the WindowManager base class. The WindowManager class
  # creates the _widget object that will be used by this window and
  # guarantees successful shutdown of rqt upon program termination.
  # Args:
  # 	pr2_interface: the single pr2 interface plug in object
  def __init__(self, pr2_interface):
    super(SwitchHandsWindowManager, self).__init__(pr2_interface)

    # Get path to UI file which should be in the "resource" folder of this package
    ui_file = os.path.join(
        rospkg.RosPack().get_path(
            'rqt_pr2_hand_syntouch_sensor_interface'), 'resource', 
            'swaphands.ui')

    # Extend the widget with all attributes and children from UI file
    loadUi(ui_file, self._widget)

    # Give QObjects reasonable names
    self._widget.setObjectName('SwapHands')

    self._widget.setWindowTitle('Swap object from one hand to another')

    # Add widget to the user interface
    user_interface = pr2_interface.get_user_interface()
    user_interface.add_widget(self._widget)

    # Register a listener for the lift button.
    self._widget.SwitchHandsButton.clicked.connect(
        self._handle_switch_hands_button_clicked)

  # function to handle the switch hands button being clicked.
  def _handle_switch_hands_button_clicked(self):
    # Notify the user that his action has been processed.
    self._widget.OutputTextBox.document().setPlainText(
        'Attempting to switch object from one hand to another')

    # TODO: Add code to call the PR2_Controller to move the hand, control the
    # hand to lift the object, etc.
    self._pr2_interface.notify_action_performed(ActionTypes.SwitchHands)
