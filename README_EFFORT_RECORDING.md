# Joint Effort Recording

Simple script to record joint effort/torque during controlled joint movement in Isaac Sim.

## Usage

```bash
./rrlab.sh -p standalone/tutorials/controllers/record_joint_effort.py \
  --joint_name Drehzapfen_joint \
  --target_position 1.57 \
  --duration 5.0 \
  --output_file effort.csv
```

## Parameters

- `--joint_name`: Joint to move (default: `Drehzapfen_joint`)
- `--target_position`: Target position in radians (default: `1.57`)
- `--duration`: Recording duration in seconds (default: `5.0`)
- `--output_file`: Output CSV filename (default: `joint_effort_recording.csv`)
- `--num_envs`: Number of environments (default: `1`)

## Output CSV

Columns: `time`, `position`, `velocity`, `effort`

Example:
```
time,position,velocity,effort
0.00,0.0000,0.0000,0.1234
0.01,0.0012,0.1200,0.1456
0.02,0.0050,0.3800,0.2100
...
```

## Joint Names

Main arm joints:
- `Drehzapfen_joint` - Rotation/slew
- `Ausleger_I_joint` - Boom segment 1
- `Ausleger_II_joint` - Boom segment 2
- `Messerkopf_Schwenk_joint` - End effector swing
- `Messerkopf_joint` - End effector blade

Wheel/steering joints:
- `Wheel_Front_Left_Steering_joint`
- `Wheel_Front_Right_Steering_joint`
- `Wheel_Rear_Left_joint`, `Wheel_Rear_Right_joint`
- `Wheel_Front_Left_joint`, `Wheel_Front_Right_joint`
