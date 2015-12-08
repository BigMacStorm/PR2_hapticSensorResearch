import os
import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

from .action_types import ActionTypes
from .window_manager import WindowManager

# Class that handles the lift object window and ui
class LiftObjectWindowManager(WindowManager):

  # Initialize the WindowManager base class. The WindowManager class
  # creates the _widget object that will be used by this window and
  # guarantees successful shutdown of rqt upon program termination.
  # Args:
  #	pr2_interface: the single pr2 interface plug in object
  def __init__(self, pr2_interface):
    super(LiftObjectWindowManager, self).__init__(pr2_interface)

    # Get path to UI file which should be in the "resource" folder of this package
    ui_file = os.path.join(
        rospkg.RosPack().get_path(
            'rqt_pr2_hand_syntouch_sensor_interface'), 'resource', 
            'liftobject.ui')

    # Extend the widget with all attributes and children from UI file
    loadUi(ui_file, self._widget)

    # Give QObjects reasonable names
    self._widget.setObjectName('LiftObject')

    self._widget.setWindowTitle('Lift Object Using PR2 Arm')

    # Add widget to the user interface
    user_interface = pr2_interface.get_user_interface()
    user_interface.add_widget(self._widget)

    # Register a listener for the lift button.
    self._widget.LiftButton.clicked.connect(self._handle_lift_button_clicked)

  # function to handle the lift button being clicked.
  def _handle_lift_button_clicked(self):
    try:
      # Obtain the coordinates of the object to lift (relative to the PR2).
      # Data comes from textboxes on the interface.
      object_x_coordinate = float(self._widget.XTextBox.toPlainText())
      object_y_coordinate = float(self._widget.YTextBox.toPlainText())
      object_z_coordinate = float(self._widget.ZTextBox.toPlainText())
      hand_to_use = self._widget.ComboBox.currentText()

      self._widget.OutputTextBox.document().setPlainText(
          'Attempting to lift object at coordinate (%s, %s, %s) with %s'
          % (object_x_coordinate, object_y_coordinate, object_z_coordinate,
             hand_to_use))
    except ValueError:
      self._widget.OutputTextBox.document().setPlainText(
          'Parameters must be floats.')

    # TODO: Add code to call the PR2_Controller to move the hand, control the
    # hand to lift the object, etc.
    self._pr2_interface.notify_action_performed(ActionTypes.LiftObject)

  # calls the function to readd the widgets if the window was closed
  def reopen(self):
    user_interface = self._pr2_interface.get_user_interface()
    user_interface.add_widget(self._widget)
