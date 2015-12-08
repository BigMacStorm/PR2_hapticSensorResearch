import os
import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

from .window_manager import WindowManager

from .mpl_dyanamic import MyMplCanvas, MyDynamicMplCanvas

# class that manages the pulse analysis window and ui
class PulseAnalysisWindowManager(WindowManager):

  # Initialize the WindowManager base class. The WindowManager class
  # creates the _widget object that will be used by this window and
  # guarantees successful shutdown of rqt upon program termination.
  # Args:
  #	pr2_interface: the single pr2 interface plug in object
  def __init__(self, pr2_interface):
    super(PulseAnalysisWindowManager, self).__init__(pr2_interface)

    # Get path to UI file which should be in the "resource" folder of this package
    ui_file = os.path.join(
        rospkg.RosPack().get_path(
            'rqt_pr2_hand_syntouch_sensor_interface'), 'resource', 
            'pulsedetection.ui')

    # Extend the widget with all attributes and children from UI file
    loadUi(ui_file, self._widget)

    # Give QObjects reasonable names
    self._widget.setObjectName('PulseAnalysis')

    self._widget.setWindowTitle('Pulse Analyzer')

    dc1 = MyDynamicMplCanvas()
    dc1.set_pr2_interface(self._pr2_interface)
    dc1.set_type('p')
    self._widget.pulse_layout.addWidget(dc1)

    # Add widget to the user interface
    user_interface = pr2_interface.get_user_interface()
    user_interface.add_widget(self._widget)

  # calls the function to readd the widgets if the window was closed
  def reopen(self):
    user_interface = self._pr2_interface.get_user_interface()
    user_interface.add_widget(self._widget)
