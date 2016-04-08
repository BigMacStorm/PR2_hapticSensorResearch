import rospy
import json

class DataTimeTick:
  # This function will initialize the data time tick. The class is created
  # with a map to store its data
  def __init__(self, data, t_recv):
    # DataTimeTick is intended to be used as a struct, so the
    # data should be made "public" by not being preprended by
    # an underscore.
    self.data = json.loads(data)
    self.data['t_recv'] = t_recv 

  # This function will return a json string of the data tiem tick
  def get_json_string(self):
    return json.dumps(self.data)

  # This function will return the x-coordinate of the left hand
  def get_x(self):
    return self.data['x']

  # This function will return the y-coordinate of the left hand
  def get_y(self):
    return self.data['y']

  # This function will return the z-coordinate of the left hand
  def get_z(self):
    return self.data['z']

  # This function will return the force of the left hand
  def get_force(self, biotac_id=0):
    return self.data['force'][biotac_id]

  # This function will return the temperature currently being 
  # sensed by the Syntouch Sensors on the left hand
  def get_temperature(self, biotac_id=0):
    return self.data['temperature'][biotac_id]

  # This function will return the thermal flux currently being 
  # sensed by the Syntouch Sensors on the left hand
  def get_thermal_flux(self, biotac_id=0):
    return self.data['thermal_flux'][biotac_id]

  # This function will return the microvibration currently being 
  # sensed by the Syntouch Sensors on the left hand
  def get_microvibration(self, biotac_id=0):
    return self.data['microvibration'][biotac_id]

  # This function will return the fluid pressure currently being 
  # applied to the Syntouch Sensors on the left hand
  def get_fluid_pressure(self, biotac_id=0):
    return self.data['fluid_pressure'][biotac_id]

  # This function will return the x-coordinate of the left hand
  def get_t_recv(self):
    return self.data['t_recv']

  # This function will return the time this data tick was sent
  # by the PR2
  def get_t_send(self):
    return self.data['t_send']
