import argparse
import torch
from isaaclab.app import AppLauncher

"""
This script randomly generates joint position for "Drehzapfen_joint" to test Unimog Mulag functionality

.. code-block:: bash

    # Usage
    ./rrlab.sh -p standalone/tutorials/controllers/random_move_mulag.py --num_envs 1

"""

# add argparse arguments
parser = argparse.ArgumentParser(description="Testing random joint movement on Unimog Mulag")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to spawn.")

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app


import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.utils import configclass

from rrlab_assets import MULAG_CFG
from isaaclab.managers import SceneEntityCfg


@configclass
class PlaneScene(InteractiveSceneCfg):
    """Configuration for a cart-pole scene."""

    # ground plane
    ground = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.GroundPlaneCfg(),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -1.05)),
    )

    robot = MULAG_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")    


    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    )


def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    """Runs the simulation loop with keyboard-controlled wheel velocity."""

    import carb
    import omni.appwindow

    robot = scene["robot"]
    print(f"DEBUG: Actual robot joint names: {robot.joint_names}")

    sim_dt = sim.get_physics_dt()
    count = 0

    # ---------------------------------------------------------------------
    # Vehicle parameters
    # ---------------------------------------------------------------------
    WHEEL_RADIUS = 0.5  # [m] change this to your actual wheel radius
    MAX_SPEED_KMH = 2.0
    MAX_SPEED_MPS = MAX_SPEED_KMH / 3.6
    MAX_WHEEL_OMEGA = MAX_SPEED_MPS / WHEEL_RADIUS

    MAX_STEER_ANGLE = 0.35  # [rad], about 20 deg
    STEER_STEP = 0.02       # [rad] per keyboard event

    # ---------------------------------------------------------------------
    # Joint names
    # ---------------------------------------------------------------------
    wheel_joint_names = [
        "Wheel_Rear_Left_joint",
        "Wheel_Rear_Right_joint",
        "Wheel_Front_Left_joint",
        "Wheel_Front_Right_joint",
    ]

    steering_joint_names = [
        "Wheel_Front_Left_Steering_joint",
        "Wheel_Front_Right_Steering_joint",
    ]

    def find_joint_ids(names):
        ids = []
        for name in names:
            joint_ids, joint_names = robot.find_joints(name)
            if len(joint_ids) == 0:
                raise RuntimeError(f"Could not find joint: {name}")
            ids.append(joint_ids[0])
        return ids

    wheel_joint_ids = find_joint_ids(wheel_joint_names)
    steering_joint_ids = find_joint_ids(steering_joint_names)

    print("DEBUG: wheel joint ids:")
    for name, idx in zip(wheel_joint_names, wheel_joint_ids):
        print(f"  {name}: {idx}")

    print("DEBUG: steering joint ids:")
    for name, idx in zip(steering_joint_names, steering_joint_ids):
        print(f"  {name}: {idx}")

    # ---------------------------------------------------------------------
    # Command state
    # ---------------------------------------------------------------------
    command = {
        "wheel_omega": 0.0,
        "steer_angle": 0.0,
    }

    target_joint_velocities = robot.data.default_joint_vel.clone()
    target_joint_positions = robot.data.default_joint_pos.clone()

    # ---------------------------------------------------------------------
    # Keyboard callback
    # ---------------------------------------------------------------------
    app_window = omni.appwindow.get_default_app_window()
    input_interface = carb.input.acquire_input_interface()
    keyboard = app_window.get_keyboard()

    def keyboard_event_callback(event):
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if event.input == carb.input.KeyboardInput.UP:
                command["wheel_omega"] = MAX_WHEEL_OMEGA
                print(f"[KEY] Forward: wheel omega = {command['wheel_omega']:.4f} rad/s")

            elif event.input == carb.input.KeyboardInput.DOWN:
                command["wheel_omega"] = -MAX_WHEEL_OMEGA
                print(f"[KEY] Backward: wheel omega = {command['wheel_omega']:.4f} rad/s")

            elif event.input == carb.input.KeyboardInput.LEFT:
                command["steer_angle"] += STEER_STEP
                command["steer_angle"] = min(command["steer_angle"], MAX_STEER_ANGLE)
                print(f"[KEY] Steer left: steer angle = {command['steer_angle']:.4f} rad")

            elif event.input == carb.input.KeyboardInput.RIGHT:
                command["steer_angle"] -= STEER_STEP
                command["steer_angle"] = max(command["steer_angle"], -MAX_STEER_ANGLE)
                print(f"[KEY] Steer right: steer angle = {command['steer_angle']:.4f} rad")

            elif event.input == carb.input.KeyboardInput.SPACE:
                command["wheel_omega"] = 0.0
                command["steer_angle"] = 0.0
                print("[KEY] Stop and center steering")

        # Returning True means the event was handled.
        return True

    keyboard_sub = input_interface.subscribe_to_keyboard_events(
        keyboard,
        keyboard_event_callback,
    )

    print("\nKeyboard control enabled:")
    print("  UP    : forward")
    print("  DOWN  : backward")
    print("  LEFT  : steer left")
    print("  RIGHT : steer right")
    print("  SPACE : stop and center steering\n")

    # ---------------------------------------------------------------------
    # Simulation loop
    # ---------------------------------------------------------------------
    while simulation_app.is_running():

        # Reset target buffers from current/default command base.
        target_joint_velocities[:] = 0.0
        target_joint_positions[:] = robot.data.default_joint_pos

        # Wheel velocity command.
        for joint_id in wheel_joint_ids:
            target_joint_velocities[:, joint_id] = command["wheel_omega"]

        # Steering position command.
        for joint_id in steering_joint_ids:
            target_joint_positions[:, joint_id] = command["steer_angle"]

        robot.set_joint_velocity_target(
            target_joint_velocities[:, wheel_joint_ids],
            joint_ids=wheel_joint_ids,
        )

        robot.set_joint_position_target(
            target_joint_positions[:, steering_joint_ids],
            joint_ids=steering_joint_ids,
        )

        # if count % 100 == 0:
        #     print("\nDEBUG command:")
        #     print(f"  wheel omega target: {command['wheel_omega']:.4f} rad/s")
        #     print(f"  steer angle target: {command['steer_angle']:.4f} rad")

        #     print("DEBUG wheel states:")
        #     for name, idx in zip(wheel_joint_names, wheel_joint_ids):
        #         print(
        #             f"  {name:30s} | "
        #             f"pos={robot.data.joint_pos[0, idx].item(): .4f} | "
        #             f"vel={robot.data.joint_vel[0, idx].item(): .4f}"
        #         )

        #     print("DEBUG steering states:")
        #     for name, idx in zip(steering_joint_names, steering_joint_ids):
        #         print(
        #             f"  {name:30s} | "
        #             f"pos={robot.data.joint_pos[0, idx].item(): .4f} | "
        #             f"vel={robot.data.joint_vel[0, idx].item(): .4f}"
        #         )

        scene.write_data_to_sim()
        sim.step()
        count += 1
        scene.update(sim_dt)

    # Clean up keyboard subscription.
    input_interface.unsubscribe_to_keyboard_events(keyboard, keyboard_sub)




def main():
    """Main function."""
    # Load kit helper
    sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    # Set main camera
    sim.set_camera_view([11, 8, 1.3], [0.0, 0.0, 0.0])
    # Design scene
    scene_cfg = PlaneScene(num_envs=args_cli.num_envs, env_spacing=20.0)
    scene = InteractiveScene(scene_cfg)

    # Play the simulator
    sim.reset()

    # Now we are ready!
    print("[INFO]: Setup complete...")
    run_simulator(sim, scene)


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
