from .connection_window_manager import ConnectionWindowManager
from .graph_window_manager import DataGraphsWindowManager
from .index_window_manager import IndexWindowManager
from .robot_window_manager import RobotWindowManager
from .run_programs_window_manager import RunProgramsWindowManager
from .sensor_window_manager import SensorWindowManager
from .stats_window_manager import LifetimeStatsWindowManager
from .pulse_analysis_window_manager import PulseAnalysisWindowManager
from .lift_object_window_manager import LiftObjectWindowManager
from .place_object_window_manager import PlaceObjectWindowManager
#from .rotate_object_window_manager import RotateObjectWindowManager
from .switch_hands_window_manager import SwitchHandsWindowManager


from .window_types import WindowTypes

class WindowCreationManager:
  
  def __init__(self, pr2_interface):
    self._pr2_interface = pr2_interface
    self._open_windows = dict()

  def new_window_manager(self, window_type):
    if window_type in self._open_windows:
      print 'That window has already been opened but may have been closed'
      print 'silently, trying to reopen...'
      self._open_windows[window_type].reopen()
      return

    if window_type is WindowTypes.IndexWindow:
      self._open_windows[window_type] = (
          IndexWindowManager(self._pr2_interface))
    elif window_type is WindowTypes.ConnectionWindow:
      self._open_windows[window_type] = (
          ConnectionWindowManager(self._pr2_interface))
    elif window_type is WindowTypes.SensorVisualizerWindow:
      self._open_windows[window_type] = (
          SensorWindowManager(self._pr2_interface))
    elif window_type is WindowTypes.RobotVisualizerWindow:
      self._open_windows[window_type] = (
          RobotWindowManager(self._pr2_interface))
    elif window_type is WindowTypes.LifetimeStatsWindow:
      self._open_windows[window_type] = (
          LifetimeStatsWindowManager(self._pr2_interface))
    elif window_type is WindowTypes.DataGraphsWindow:
      self._open_windows[window_type] = (
          DataGraphsWindowManager(self._pr2_interface))
    elif window_type is WindowTypes.RunProgramsWindow:
      self._open_windows[window_type] = (
          RunProgramsWindowManager(self._pr2_interface))
    # Below are windows that can be opened from the "Run Programs Window", which
    # has its own windows.
    elif window_type is WindowTypes.PulseAnalysisWindow:
      self._open_windows[window_type] = (
          PulseAnalysisWindowManager(self._pr2_interface))
    elif window_type is WindowTypes.LiftObjectWindow:
      self._open_windows[window_type] = (
          LiftObjectWindowManager(self._pr2_interface))
    elif window_type is WindowTypes.PlaceObjectWindow:
      self._open_windows[window_type] = (
          PlaceObjectWindowManager(self._pr2_interface))
    elif window_type is WindowTypes.RotateObjectWindow:
      raise NotImplementedError
#      self._open_window_managers.add(
#          RotateObjectWindowManager(self._pr2_interface))
#      self._open_window_types.add(window_type)
    elif window_type is WindowTypes.SwitchHandsWindow:
      self._open_windows[window_type] = (
          SwitchHandsWindowManager(self._pr2_interface))
    else:
      raise NotImplementedError
    
  def shutdown_window(self, window_type):
    self._open_windows[window_type].shutdown()

  def shutdown_all_windows(self):
    for window in self._open_windows:
      self.shutdown_window(window)
