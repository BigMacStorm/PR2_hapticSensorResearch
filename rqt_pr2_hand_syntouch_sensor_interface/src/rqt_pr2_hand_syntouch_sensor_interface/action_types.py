NUM_ACTION_TYPES = 4

# The following is an enumeration of ActionTypes, each describing
# a single action that the PR2 interface can cause the PR2 robot
# to perform.
class ActionTypes:
  LiftObject, PlaceObject, RotateObject, SwitchHands = range(NUM_ACTION_TYPES)
