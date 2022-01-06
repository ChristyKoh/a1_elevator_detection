# elevator_floor
```
rosrun elevator_floor elevator_floor_main.py
```

## Package Structure
`src/`
- `elevator_floor_main.py` - elevator estimation node
- `altitude_utils.py` - util functions for filtering and altitude calcs
- `plotting.py` - plotting functions for accel, vel, disp

`srv`
- `FloorSetting.srv` - simple service to set int64 current floor