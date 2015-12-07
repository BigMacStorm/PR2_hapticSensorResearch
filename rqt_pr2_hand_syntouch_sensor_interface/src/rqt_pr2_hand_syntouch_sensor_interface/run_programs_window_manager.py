import os
import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

from .window_types import WindowTypes

class RunProgramsWindowManager:

  def __init__(self, pr2_interface):
    self._pr2_interface = pr2_interface

    # Create QWidget object (this is object that represents the window
    # that the user actually sees).
    self._widget = QWidget()

    # Get path to UI file which should be in the "resource" folder of this package
    ui_file = os.path.join(
        rospkg.RosPack().get_path(
            'rqt_pr2_hand_syntouch_sensor_interface'), 'resource', 'programlist.ui')

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

    self._widget.LiftObjectButton.clicked.connect(
        self._handle_lift_object_button_clicked)

    self._widget.PlaceObjectButton.clicked.connect(
        self._handle_place_object_button_clicked)

    self._widget.RotateObjectButton.clicked.connect(
        self._handle_rotate_object_button_clicked)

    self._widget.SwitchHandsButton.clicked.connect(
        self._handle_switch_hands_button_clicked)

  def _handle_take_pulse_button_clicked(self):
    self._pr2_interface.open_window(WindowTypes.PulseAnalysisWindow)

  def _handle_lift_object_button_clicked(self):
    self._pr2_interface.open_window(WindowTypes.LiftObjectWindow)

  def _handle_place_object_button_clicked(self):
    self._pr2_interface.open_window(WindowTypes.PlaceObjectWindow)

  def _handle_rotate_object_button_clicked(self):
    self._pr2_interface.open_window(WindowTypes.RotateObjectWindow)

  def _handle_switch_hands_button_clicked(self):
    self._pr2_interface.open_window(WindowTypes.SwitchHandsWindow)

  def reopen(self):
    user_interface = self._pr2_interface.get_user_interface()
    user_interface.add_widget(self._widget)
