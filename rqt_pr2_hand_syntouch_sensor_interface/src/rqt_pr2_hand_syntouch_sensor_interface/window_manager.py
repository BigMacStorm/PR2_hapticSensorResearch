import abc

from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

# The abstract base class that all of the other window managers inherit from,
# providing the necessary methods that all WindowManagers need.
class WindowManager:
  __metaclass__ = abc.ABCMeta

  # Initialize the WindowManager base class. The WindowManager class
  # creates the _widget object that will be used by all windows and
  # further initializes variables that will be used by each derived
  # WindowManager object.
  # Args:
  # 	pr2_interface: the single pr2 interface plug in object
  @abc.abstractmethod
  def __init__(self, pr2_interface):
    self._widget = QWidget()
    self._pr2_interface = pr2_interface
    self._destroyed = False
    self._widget.destroyed.connect(self.shutdown)

  # This function signals to an individual window manager that it should start shutting down its threads and begin the process of closing itself
  def shutdown(self):
    self._destroyed = True

  # Re-adds the widget to the user interface window.
  # (different from adding first time because constructor not called.
  def reopen(self):
    user_interface = self._pr2_interface.get_user_interface()
    user_interface.add_widget(self._widget)
