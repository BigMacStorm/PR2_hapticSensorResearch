import os
import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

from .window_manager import WindowManager

# Class that handles the place object window and ui
class PlaceObjectWindowManager(WindowManager):

  # Initialize the WindowManager base class. The WindowManager class
  # creates the _widget object that will be used by this window and
  # guarantees successful shutdown of rqt upon program termination.
  # Args:
  #	pr2_interface: the single pr2 interface plug in object
  def __init__(self, pr2_interface):

    super(PlaceObjectWindowManager, self).__init__(pr2_interface)

    # Get path to UI file which should be in the "resource" folder of this package
    ui_file = os.path.join(
        rospkg.RosPack().get_path(
            'rqt_pr2_hand_syntouch_sensor_interface'), 'resource', 
            'placeobject.ui')

    # Extend the widget with all attributes and children from UI file
    loadUi(ui_file, self._widget)

    # Give QObjects reasonable names
    self._widget.setObjectName('PlaceObject')

    self._widget.setWindowTitle('Place Object Using PR2 Arm')

    # Add widget to the user interface
    user_interface = pr2_interface.get_user_interface()
    user_interface.add_widget(self._widget)

  # calls the function to readd the widgets if the window was closed
  def reopen(self):
    user_interface = self._pr2_interface.get_user_interface()
    user_interface.add_widget(self._widget)
