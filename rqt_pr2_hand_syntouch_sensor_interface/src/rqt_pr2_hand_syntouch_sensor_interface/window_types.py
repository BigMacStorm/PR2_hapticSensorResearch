NUM_WINDOW_TYPES = 11

# The following is an enumeration of WindowTypes, each describing
# a single (and custom) window that can be opened within the 
# pr2_interface. 
class WindowTypes:
  IndexWindow, ConnectionWindow, RunProgramsWindow, DataGraphsWindow, \
  LifetimeStatsWindow, SensorVisualizerWindow, PulseAnalysisWindow, \
  GraspObjectWindow, ObjectHandoffWindow, RotateObjectWindow, \
  SwitchHandsWindow = range(NUM_WINDOW_TYPES)
