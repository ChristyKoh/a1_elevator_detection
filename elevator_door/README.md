# elevator_door

## Package Structure
`exe/`
- `state_cmd_lcm.cpp` - server for sending HighCmd and publishing HighState

`src/`
- `utils.py` - functions for vertical line extraction and point projection
- `elevator_door_tracker.py` - ElevatorDoorTracker class
- `elevator_detection.py` - elevator door detection node
  - publishes avg PointCloud depth to `/elevator/avg_depth`
  - publishes door state to `/elevator/door_state`
- `a1_control_states.py` - ControlState class for state machine implementation
- `elevator_control.py` - state machine control node

`msg/`
- `ElevatorDoorState.msg` - custom msg for state including door boundaries

`launch/control.launch`
(launch file in progress, currently doesn't work)
