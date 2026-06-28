# Phase A v2 — verify RMPflow on the MERGED /husky articulation.
# Fixes vs v1:
#   - arm_idx contains only the 7 panda_joint{1..7} (RMPflow doesn't drive fingers).
#   - Init is deferred a few physics steps so PhysX has settled the articulation
#     before we read panda_link0's world pose (used as RMPflow base).
#   - Heavier debug prints (poses, RMPflow active joints, first action).
#
# Procedure (husky_franka.usd open):
#   1) STOP timeline.
#   2) Script Editor:  exec(open('/home/user/Desktop/260527 KMS/reach_test_husky.py').read())
#   3) PLAY.

import numpy as np
import omni.usd, omni.physx
from pxr import Gf, UsdGeom

from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.rotations import euler_angles_to_quat
import isaacsim.robot_motion.motion_generation as mg

HUSKY = "/husky"
BASE  = HUSKY + "/base_link"
LINK0 = HUSKY + "/panda/panda_link0"
EE    = HUSKY + "/panda/panda_rightfinger"
CUBE  = "/PickCube"
CUBE_SIZE = 0.05

INIT_WAIT_STEPS = 10   # let PhysX settle before reading transforms / initializing RMPflow

stage = omni.usd.get_context().get_stage()
xc = UsdGeom.XformCache()

# 1) Spawn cube in front of Husky (world frame), on the ground.
xc.Clear()
hpos = xc.GetLocalToWorldTransform(stage.GetPrimAtPath(HUSKY)).ExtractTranslation()
cube_pos_world = np.array([float(hpos[0]),
                           float(hpos[1]) + 0.70,
                           CUBE_SIZE/2.0])
if stage.GetPrimAtPath(CUBE):
    stage.RemovePrim(CUBE)
cube = DynamicCuboid(prim_path=CUBE, position=cube_pos_world,
                     scale=np.array([CUBE_SIZE]*3), size=1.0,
                     color=np.array([0.1, 0.3, 0.9]))
print(f"[A] cube world pos: {cube_pos_world.tolist()}")

# 2) Merged articulation + RMPflow (Franka).
robot = SingleArticulation(prim_path=HUSKY, name="husky_franka_robot")
rmp_cfg  = mg.interface_config_loader.load_supported_motion_policy_config("Franka", "RMPflow")
rmp_flow = mg.lula.motion_policies.RmpFlow(**rmp_cfg)
art_rmp  = mg.ArticulationMotionPolicy(robot, rmp_flow, default_physics_dt=1.0/60.0)

ARM_NAMES = [f"panda_joint{i}" for i in range(1, 8)]   # 7 joints only (no fingers)

state = {"init": False, "settle": 0, "arm_idx": None, "step": 0, "logged_action": False}

def _wpos(path):
    xc.Clear()
    return xc.GetLocalToWorldTransform(stage.GetPrimAtPath(path)).ExtractTranslation()

def link0_world_pose():
    xc.Clear()
    M = xc.GetLocalToWorldTransform(stage.GetPrimAtPath(LINK0))
    t = M.ExtractTranslation(); q = M.ExtractRotationQuat()
    return (np.array([t[0], t[1], t[2]]),
            np.array([q.GetReal(), *q.GetImaginary()]))   # (w, x, y, z)

def world_to_link0(world_xyz):
    xc.Clear()
    M = xc.GetLocalToWorldTransform(stage.GetPrimAtPath(LINK0))
    p = M.GetInverse().Transform(Gf.Vec3d(float(world_xyz[0]),
                                          float(world_xyz[1]),
                                          float(world_xyz[2])))
    return np.array([p[0], p[1], p[2]])

def _post_init():
    robot.initialize()
    state["arm_idx"] = np.array([robot.get_dof_index(n) for n in ARM_NAMES])
    print(f"[A] dof_names ({len(robot.dof_names)}): {robot.dof_names}")
    print(f"[A] ARM_NAMES -> arm_idx: {dict(zip(ARM_NAMES, state['arm_idx'].tolist()))}")
    for p in [HUSKY, BASE, LINK0, EE]:
        t = _wpos(p); print(f"[A] world {p:40s} -> ({t[0]:+.3f}, {t[1]:+.3f}, {t[2]:+.3f})")
    bp, bq = link0_world_pose()
    rmp_flow.set_robot_base_pose(robot_position=bp, robot_orientation=bq)
    try:
        aj = rmp_flow.get_active_joints()
        print(f"[A] RMPflow active joints ({len(aj)}): {aj}")
    except Exception as e:
        print("[A] (could not query active joints):", e)
    state["init"] = True

def on_physics_step(dt):
    # Defer init: wait for PhysX to settle the articulation.
    if not state["init"]:
        state["settle"] += 1
        if state["settle"] < INIT_WAIT_STEPS:
            return
        try:
            _post_init()
        except Exception as e:
            print(f"[A] init pending (step {state['settle']}):", e); return

    # Update RMPflow base each step (so it stays correct when Husky moves later).
    bp, bq = link0_world_pose()
    rmp_flow.set_robot_base_pose(robot_position=bp, robot_orientation=bq)

    # Target: 10 cm above the cube in world -> panda_link0 local frame.
    cube_w = cube.get_world_pose()[0]
    target_world = np.array([cube_w[0], cube_w[1], cube_w[2] + 0.10])
    target_local = world_to_link0(target_world)
    target_quat  = euler_angles_to_quat(np.array([0.0, np.pi, 0.0]))

    rmp_flow.set_end_effector_target(target_local, target_quat)
    rmp_flow.update_world()
    action = art_rmp.get_next_articulation_action()
    jp = action.joint_positions
    if jp is None:
        return
    jp = np.asarray(jp, dtype=float)

    if not state["logged_action"]:
        print(f"[A] first RMPflow action len={len(jp)}, values={jp.tolist()}")
        print(f"[A] base_world={bp.tolist()}  target_world={target_world.tolist()}  target_link0={target_local.tolist()}")
        state["logged_action"] = True

    robot.apply_action(ArticulationAction(joint_positions=jp, joint_indices=state["arm_idx"]))

    state["step"] += 1
    if state["step"] % 60 == 0:
        ee_w = _wpos(EE)
        err = float(np.linalg.norm(np.array([ee_w[0], ee_w[1], ee_w[2]]) - target_world))
        print(f"[A] step {state['step']}  EE_world=({ee_w[0]:+.3f},{ee_w[1]:+.3f},{ee_w[2]:+.3f})  target_world=({target_world[0]:+.3f},{target_world[1]:+.3f},{target_world[2]:+.3f})  err={err:.3f} m")

# Safe re-run
if "_reach_sub_id" in globals():
    try: omni.physx.get_physx_interface().unsubscribe_physics_step_events(_reach_sub_id)
    except Exception: pass
_reach_sub_id = omni.physx.get_physx_interface().subscribe_physics_step_events(on_physics_step)
print("[A] subscribed. Press PLAY now (init deferred ~10 steps).")
