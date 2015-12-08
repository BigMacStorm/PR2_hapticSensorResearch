import abc

from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

# The abstract base class that all of the other window managers inherit from 
class WindowManager:
  __metaclass__ = abc.ABCMeta

  @abc.abstractmethod
  def __init__(self, pr2_interface):
  # Initialize the WindowManager base class. The WindowManager class
  # creates the _widget object that will be used by all windoww and
  # guarantees successful shutdown of rqt upon program termination.
  # Args:
  # 	pr2_interface: the single pr2 interface plug in object
    self._widget = QWidget()
    self._pr2_interface = pr2_interface
    self._destroyed = False
    self._widget.destroyed.connect(self.shutdown)

  def shutdown(self):
  # Signals to an individual window manager that it should start shutting down its threads and begin the process of closing itself
    self._destroyed = True

