# Kevin Integration

This folder contains the integration of the trained AAM, inverse dynamics, system dynamics, and future hydraulic reduced-order model into the existing RRLAB Isaac Lab Mulag environment.

## Goal

Use the existing RRLAB Mulag environment and integrate:

1. Arm Adaptation Module
2. Learned inverse dynamics model
3. Learned system dynamics model
4. Reduced-order hydraulic actuator model
5. Final learned control pipeline

## Development rule

Do not modify `rrlab_assets` unless changing the robot asset itself.
Do not modify the original RRLAB task until the integration works separately.

## Learned models + controller scaffold

The trained checkpoints live in `kevin_integration/models/`:
- `aam.pth` (AAM): history -> `z_arm_hat` (24-d)
- `id.pth` (ID): history + `z_arm_hat` -> `valve_cmd` (4-d)
- `sd.pth` (SD): history + `z_arm_hat` -> `delta_state` (18-d)
- `fd.pth` + `fd_scaler.pth` (FD): history -> `torque` (4-d)

The code scaffold to *load and run* these models is in:
- `kevin_integration/controllers/learned_models.py`
- `kevin_integration/controllers/pipeline.py`
- `kevin_integration/control_policy/policy.py`

History window lengths are read from checkpoint metadata when present. If a checkpoint does not expose that
metadata, `PipelineConfig.fallback_window` is used.

### Usage (smoke test)

This repo does not include the feature assembly logic (ordering must match training).
The script below just runs a dummy step with zeros to confirm the pipeline loads:

```bash
./rrlab.sh -p kevin_integration/scripts/run_control_policy.py --debug --device cpu
```

### Usage (IsaacLab Mulag runner)

This runner reuses the working standalone controller structure from `standalone/tutorials/controllers/`:
it launches IsaacLab, spawns `MULAG_CFG`, finds the arm joints, builds feature vectors from live joint state,
and applies the learned command to the simulator. It uses 5 joints for model state features and 4 joints for
the valve-derived command output.

```bash
./rrlab.sh -p kevin_integration/scripts/run_mulag_learned_controller.py --num_envs 1
```

Safer first test:

```bash
./rrlab.sh -p kevin_integration/scripts/run_mulag_learned_controller.py \
  --num_envs 1 \
  --duration 10 \
  --control_mode position_delta \
  --position_scale 0.005
```

FD torque mode:

```bash
./rrlab.sh -p kevin_integration/scripts/run_mulag_learned_controller.py \
  --num_envs 1 \
  --duration 10 \
  --control_mode effort \
  --max_abs_effort 1000
```

In effort mode, FD outputs 4 actuator torques. The runner expands this to a 5-joint Isaac effort vector with:
`[tau_act1, tau_act2, tau_act3, tau_act4, 0.0]`, then applies it on the 5 `state_joints`.

Optional SD prediction:

```bash
./rrlab.sh -p kevin_integration/scripts/run_mulag_learned_controller.py \
  --num_envs 1 \
  --duration 10 \
  --predict_sd
```

### Integration contract (important)

`ControlPolicy.compute_action(observation)` expects a dict with:
- `aam_x_t`: shape `(18,)` = `[q(5), qdot(5), dP(4), valve_cmd(4)]`
- `id_x_t`: shape `(19,)` = `[q(5), qdot(5), qddot(5), dP(4)]`
Optional:
- `sd_x_t`: shape `(22,)` = `[q(5), qdot(5), dP(4), Fnet(4), valve_cmd(4)]`
- `fd_x_t`: shape `(18,)` = `[q(5), qdot(5), dP(4), valve_cmd(4)]`

These vectors must be assembled in the **same feature order as used during training**.

The current IsaacLab runner estimates `qddot` by finite differencing `qdot`. It initializes `dP` and `Fnet`
to zeros because those hydraulic signals are not currently exposed by `MULAG_CFG`; replace them with real,
estimated, or simulated hydraulic signals before judging physical performance.

Default state joints:
- `Drehzapfen_joint`
- `Ausleger_I_joint`
- `Ausleger_II_joint`
- `Messerkopf_Schwenk_joint`
- `Messerkopf_joint`

Default command joints:
- `Drehzapfen_joint`
- `Ausleger_I_joint`
- `Ausleger_II_joint`
- `Messerkopf_Schwenk_joint`
