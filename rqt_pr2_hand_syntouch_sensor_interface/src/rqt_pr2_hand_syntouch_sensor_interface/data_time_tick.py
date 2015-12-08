import rospy
import json

class DataTimeTick:
  
  def __init__(self, data, t_recv):
    # DataTimeTick is intended to be used as a struct, so the
    # data should be made "public" by not being preprended by
    # an underscore.
    self.data = json.loads(data)
    self.data['t_recv'] = t_recv 

  def get_json_string(self):
    return json.dumps(self.data)

  def get_x(self):
    return self.data['x']

  def get_y(self):
    return self.data['y']

  def get_z(self):
    return self.data['z']

  def get_force(self):
    return self.data['force']

  def get_temperature(self):
    return self.data['temperature']

  def get_thermal_flux(self):
    return self.data['thermal_flux']

  def get_microvibration(self):
    return self.data['microvibration']

  def get_fluid_pressure(self):
    return self.data['fluid_pressure']

  def get_t_recv(self):
    return self.data['t_recv']

  def get_t_send(self):
    return self.data['t_send']
