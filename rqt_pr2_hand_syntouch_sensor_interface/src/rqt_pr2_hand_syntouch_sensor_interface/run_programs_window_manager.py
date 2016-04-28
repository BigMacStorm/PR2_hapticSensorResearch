import os
import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

from .action_types import ActionTypes
from .window_manager import WindowManager
from .window_types import WindowTypes


# Class that handles the run programs window and ui.
class RunProgramsWindowManager(WindowManager):

  # Initialize the WindowManager base class. The WindowManager class
  # creates the _widget object that will be used by this window and
  # guarantees successful shutdown of rqt upon program termination.
  # Args:
  #	pr2_interface: the single pr2 interface plug in object
  def __init__(self, pr2_interface):

    super(RunProgramsWindowManager, self).__init__(pr2_interface)

    # Get path to UI file which should be in the "resource" folder of this package
    ui_file = os.path.join(
        rospkg.RosPack().get_path(
            'rqt_pr2_hand_syntouch_sensor_interface'), 'resource',
            'programlist.ui')

    # Extend the widget with all attributes and children from UI file
    loadUi(ui_file, self._widget)
    # Give QObjects reasonable names
    self._widget.setObjectName('ProgramList')

    self._widget.setWindowTitle('Runnable Program List')

    # Add widget to the user interface
    user_interface = pr2_interface.get_user_interface()
    user_interface.add_widget(self._widget)

    # Register listeners for all of the buttons on the run programs window.
    self._widget.TakePulseButton.clicked.connect(
        self._handle_take_pulse_button_clicked)

    self._widget.GraspObjectButton.clicked.connect(
        self._handle_grasp_object_button_clicked)

    self._widget.ObjectHandoffButton.clicked.connect(
        self._handle_object_handoff_button_clicked)

    self._widget.RotateObjectButton.clicked.connect(
        self._handle_rotate_object_button_clicked)

    self._widget.SwitchHandsButton.clicked.connect(
        self._handle_switch_hands_button_clicked)

    self._widget.OpenHandButton.clicked.connect(
        self._handle_open_hand_button_clicked)

    self._widget.CloseHandButton.clicked.connect(
        self._handle_close_hand_button_clicked)

  # Opens the pulse analysis window 
  def _handle_take_pulse_button_clicked(self):
    self._pr2_interface.open_window(WindowTypes.PulseAnalysisWindow)

  # Opens the grasp object window
  def _handle_grasp_object_button_clicked(self):
    self._pr2_interface.open_window(WindowTypes.GraspObjectWindow)

  # Opens the place object window
  def _handle_object_handoff_button_clicked(self):
    self._pr2_interface.open_window(WindowTypes.ObjectHandoffWindow)

  # Opens the rotate object window
  def _handle_rotate_object_button_clicked(self):
    self._pr2_interface.open_window(WindowTypes.RotateObjectWindow)

  # Opens the switch hands window
  def _handle_switch_hands_button_clicked(self):
    self._pr2_interface.open_window(WindowTypes.SwitchHandsWindow)

  # Close the PR2 Hand
  def _handle_close_hand_button_clicked(self):
    pr2_controller = self._pr2_interface.get_pr2_controller()
    pr2_controller.signal_close_left_hand()
    self._pr2_interface.notify_action_performed(ActionTypes.CloseHand)

  # Open the PR2 Hand
  def _handle_open_hand_button_clicked(self):
    pr2_controller = self._pr2_interface.get_pr2_controller()
    pr2_controller.signal_open_left_hand()
    self._pr2_interface.notify_action_performed(ActionTypes.OpenHand)

  # calls the function to readd the widgets if the window was closed
  def reopen(self):
    user_interface = self._pr2_interface.get_user_interface()
    user_interface.add_widget(self._widget)
