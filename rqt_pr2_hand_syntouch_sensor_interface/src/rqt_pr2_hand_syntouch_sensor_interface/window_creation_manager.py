from .index_window_manager import IndexWindowManager
from .connection_window_manager import ConnectionWindowManager
from .sensor_window_manager import SensorWindowManager
from .robot_window_manager import RobotWindowManager
from .lifeTimeStats_window_manager import LifetimeStatsWindowManager
from .dataGraph_window_manager import DataGraphsWindowManager
from .window_types import WindowTypes

class WindowCreationManager:
  
  def __init__(self, pr2_interface):
    self._pr2_interface = pr2_interface
    self._open_window_types = set()
    self._open_window_managers = set()

  def new_window_manager(self, window_type):
    if window_type in self._open_window_types:
      print 'That window is already open'
      return

    if window_type is WindowTypes.IndexWindow:
      self._open_window_managers.add(IndexWindowManager(self._pr2_interface))
      self._open_window_types.add(window_type)
    elif window_type is WindowTypes.ConnectionWindow:
      self._open_window_managers.add(ConnectionWindowManager(self._pr2_interface))
      self._open_window_types.add(window_type)
    elif window_type is WindowTypes.SensorVisualizerWindow:
      self._open_window_managers.add(SensorWindowManager(self._pr2_interface))
      self._open_window_types.add(window_type)
    elif window_type is WindowTypes.RobotVisualizerWindow:
      self._open_window_managers.add(RobotWindowManager(self._pr2_interface))
      self._open_window_types.add(window_type)
    elif window_type is WindowTypes.LifetimeStatsWindow:
      self._open_window_managers.add(LifetimeStatsWindowManager(self._pr2_interface))
      self._open_window_types.add(window_type)
    elif window_type is WindowTypes.DataGraphsWindow:
      self._open_window_managers.add(DataGraphsWindowManager(self._pr2_interface))
      self._open_window_types.add(window_type)
    else:
      raise NotImplementedError
    
  def shutdown_window(self, window_type):
    pass

  def shutdown_all_windows(self):
    for window in self._open_windows():
      self.shutdown_window(window)


