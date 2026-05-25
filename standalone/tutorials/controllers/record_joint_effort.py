import argparse
import csv
from pathlib import Path
from isaaclab.app import AppLauncher

"""
Record joint effort/torque during controlled joint movement.

Usage:
    ./rrlab.sh -p standalone/tutorials/controllers/record_joint_effort.py \
      --joint_name Drehzapfen_joint --target_position 1.57 --duration 5.0
"""

# add argparse arguments
parser = argparse.ArgumentParser(description="Record joint effort during controlled movement on Unimog Mulag")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to spawn.")
parser.add_argument("--joint_name", type=str, default="Drehzapfen_joint", help="Name of the joint to move.")
parser.add_argument("--target_position", type=float, default=1.57, help="Target joint position (radians).")
parser.add_argument("--duration", type=float, default=5.0, help="Duration of movement (seconds).")
parser.add_argument("--output_file", type=str, default="joint_effort_recording.csv", help="Output CSV file name.")

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


@configclass
class PlaneScene(InteractiveSceneCfg):
    """Configuration for a plane scene with Unimog Mulag robot."""

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


def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene, args: argparse.Namespace):
    """Run simulation and record joint effort."""
    robot = scene["robot"]

    sim_dt = sim.get_physics_dt()
    count = 0
    
    # Find the target joint
    try:
        joint_ids, _ = robot.find_joints(args.joint_name)
        if len(joint_ids) == 0:
            raise ValueError(f"Joint '{args.joint_name}' not found!")
        target_joint_id = joint_ids[0]
        print(f"[INFO] Found joint '{args.joint_name}' at index {target_joint_id}")
    except Exception as e:
        print(f"[ERROR] {e}")
        return

    # Initialize recording
    duration_steps = int(args.duration / sim_dt)
    recording_data = {
        "time": [],
        "position": [],
        "velocity": [],
        "effort": [],
    }
    
    # Initial target position
    target_joint_positions = robot.data.default_joint_pos.clone()

    print(f"[INFO] Recording for {args.duration}s ({duration_steps} steps)")
    print(f"[INFO] Target: {args.target_position} rad\n")

    # Simulation loop
    while simulation_app.is_running() and count < duration_steps:
        
        # Smooth ramp-up to target position over first 30% of duration
        progress = min(1.0, count / (duration_steps * 0.3))
        target_pos = args.target_position * progress
        target_joint_positions[:, target_joint_id] = target_pos

        # Set and record
        robot.set_joint_position_target(target_joint_positions)
        
        current_time = count * sim_dt
        recording_data["time"].append(current_time)
        recording_data["position"].append(robot.data.joint_pos[0, target_joint_id].item())
        recording_data["velocity"].append(robot.data.joint_vel[0, target_joint_id].item())
        recording_data["effort"].append(robot.data.applied_torque[0, target_joint_id].item())

        if count % 100 == 0:
            print(f"[{count:5d}/{duration_steps}] t={current_time:.2f}s | "
                  f"pos={recording_data['position'][-1]:.4f} | "
                  f"effort={recording_data['effort'][-1]:.4f} N·m")

        scene.write_data_to_sim()
        sim.step()
        count += 1
        scene.update(sim_dt)

    # Save to CSV
    csv_path = Path(args.output_file)
    with open(csv_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=["time", "position", "velocity", "effort"])
        writer.writeheader()
        for i in range(len(recording_data["time"])):
            writer.writerow({
                "time": recording_data["time"][i],
                "position": recording_data["position"][i],
                "velocity": recording_data["velocity"][i],
                "effort": recording_data["effort"][i],
            })
    
    print(f"\n[INFO] Saved {len(recording_data['time'])} points to: {csv_path.absolute()}")


def main():
    """Main function."""
    sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=args_cli.device)
    physx = sim_cfg.physx
    
    # Configure GPU memory for physics (reduced values)
    settings = {
        "gpu_max_rigid_contact_count": 2**16,
        "gpu_found_lost_pairs_capacity": 2**16,
        "gpu_total_aggregate_pairs_capacity": 2**16,
        "gpu_collision_stack_size": 2**20,
        "gpu_heap_capacity": 2**22,
        "gpu_temp_buffer_capacity": 2**20,
    }
    for name, value in settings.items():
        if hasattr(physx, name):
            setattr(physx, name, value)
    
    sim = sim_utils.SimulationContext(sim_cfg)
    sim.set_camera_view([11, 8, 1.3], [0.0, 0.0, 0.0])
    
    scene_cfg = PlaneScene(num_envs=args_cli.num_envs, env_spacing=20.0)
    scene = InteractiveScene(scene_cfg)
    sim.reset()
    
    print("[INFO] Setup complete")
    run_simulator(sim, scene, args_cli)


if __name__ == "__main__":
    main()
    simulation_app.close()
