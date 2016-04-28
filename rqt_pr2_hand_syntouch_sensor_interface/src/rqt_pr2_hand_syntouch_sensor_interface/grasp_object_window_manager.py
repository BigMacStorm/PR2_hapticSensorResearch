import os
import rospy
import rospkg
import threading

from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

from .action_types import ActionTypes
from .window_manager import WindowManager
from .pr2_controller import HAND_OPEN_POSITION

# Class that handles the lift object window and ui
class GraspObjectWindowManager(WindowManager):

  # Initialize the WindowManager base class. The WindowManager class
  # creates the _widget object that will be used by this window and
  # guarantees successful shutdown of rqt upon program termination.
  # Args:
  #	pr2_interface: the single pr2 interface plug in object
  def __init__(self, pr2_interface):
    super(GraspObjectWindowManager, self).__init__(pr2_interface)

    # Get path to UI file which should be in the "resource" folder of this package
    ui_file = os.path.join(
        rospkg.RosPack().get_path(
            'rqt_pr2_hand_syntouch_sensor_interface'), 'resource', 
            'graspobject.ui')

    # Extend the widget with all attributes and children from UI file
    loadUi(ui_file, self._widget)

    # Give QObjects reasonable names
    self._widget.setObjectName('GraspObject')

    self._widget.setWindowTitle('Grasp Object Using PR2 Arm')

    # Add widget to the user interface
    user_interface = pr2_interface.get_user_interface()
    user_interface.add_widget(self._widget)

    # Register a listener for the lift button.
    self._widget.GraspButton.clicked.connect(self._handle_grasp_button_clicked)

  # function to handle the lift button being clicked.
  def _socially_aware_grasp(self):
    curr_text = 'Performing a socially aware grasp.'
    self._widget.OutputTextBox.document().setPlainText(curr_text)
    grasp_complete = False
    pr2_controller = self._pr2_interface.get_pr2_controller()

    last_data_point = None
    current_data_point = self._pr2_interface.get_most_recent_data()
    start_value = current_data_point.get_fluid_pressure()
    threshold = 200
    curr_scalar = 1
    while not grasp_complete:
      current_data_point = self._pr2_interface.get_most_recent_data()
      if last_data_point and last_data_point != current_data_point:
        if abs(current_data_point.get_fluid_pressure() - 
               start_value) > threshold or curr_scalar <= 0:
	  curr_text += '\nGrasp completed.'
	  self._widget.OutputTextBox.document().setPlainText(curr_text)
          grasp_complete = True
        else:
          curr_scalar -= .01
          pr2_controller.signal_close_left_hand(HAND_OPEN_POSITION*curr_scalar)
          print curr_scalar
          
      last_data_point = current_data_point
      self._pr2_interface.get_most_recent_data()

    # TODO: Add code to call the PR2_Controller to move the hand, control the
    # hand to lift the object, etc.
    self._pr2_interface.notify_action_performed(ActionTypes.PlaceObject)
    

  def _handle_grasp_button_clicked(self):
    self._worker = threading.Thread(target=self._socially_aware_grasp)
    self._worker.start()

  # calls the function to readd the widgets if the window was closed
  def reopen(self):
    user_interface = self._pr2_interface.get_user_interface()
    user_interface.add_widget(self._widget)
