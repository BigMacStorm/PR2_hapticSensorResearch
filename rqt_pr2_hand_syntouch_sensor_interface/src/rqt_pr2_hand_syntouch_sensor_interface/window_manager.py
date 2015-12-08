import abc

from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

class WindowManager:
  __metaclass__ = abc.ABCMeta

  @abc.abstractmethod
  def __init__(self, pr2_interface):
    self._widget = QWidget()
    self._pr2_interface = pr2_interface
    self._destroyed = False
    self._widget.destroyed.connect(self.shutdown)

  def shutdown(self):
    self._destroyed = True

