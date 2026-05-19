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