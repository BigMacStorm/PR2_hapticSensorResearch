import rospy
import json

class DataTimeTick:
  
  def __init__(self, data):
    # DataTimeTick is intended to be used as a struct, so the
    # data should be made "public" by not being preprended by
    # an underscore.
    self.data = json.loads(data)
    self.data['t_recv'] = rospy.get_rostime().to_nsec()

  def get_json_string(self):
    return json.dumps(self._data)

  def get_x(self):
    return self._data['x']

  def get_y(self):
    return self._data['y']

  def get_z(self):
    return self._data['z']

  def get_force(self):
    return self._data['force']

  def get_temperature(self):
    return self._data['temperature']

  def get_thermal_flux(self):
    return self._data['thermal_flux']

  def get_microvibration(self):
    return self._data['microvibration']

  def get_fluid_pressure(self):
    return self._data['fluid_pressure']
