import os
import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

class RobotWindowManager:

  def __init__(self, pr2_interface):
    self._pr2_interface = pr2_interface

    # Create QWidget object (this is object that represents the window
    # that the user actually sees).
    self._widget = QWidget()

    # Get path to UI file which should be in the "resource" folder of this package
    ui_file = os.path.join(
        rospkg.RosPack().get_path(
            'rqt_pr2_hand_syntouch_sensor_interface'), 'resource', 'pr2robotvisualizer.ui')

    # Extend the widget with all attributes and children from UI file
    loadUi(ui_file, self._widget)

    # Give QObjects reasonable names
    self._widget.setObjectName('RobotVisualizerWindow')

    self._widget.setWindowTitle(
        'Robot Visualizer')

    # Add widget to the user interface
    user_interface = pr2_interface.get_user_interface()
    user_interface.add_widget(self._widget)
