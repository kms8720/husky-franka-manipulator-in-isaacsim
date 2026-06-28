# Phase B (RMPflow version) — slow-motion demo, for the "before" comparison.
# Same scene setup as pick_place_husky.py but uses RMPflow as the cspace controller.
# EXPECTED RESULT: EE crawls toward the cube, never reaches it within the state-machine's
# phase time, so cube is NOT grasped (cube_z stays constant).
#
# Procedure (husky_franka.usd open, floor lifted to z=0):
#   1) STOP. 2) Script Editor: exec(open('/home/user/Desktop/260527 KMS/pick_place_husky_rmp.py').read())
#   3) PLAY.

import numpy as np
import omni.usd, omni.physx
from pxr import Gf, UsdGeom

from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.robot.manipulators.grippers.parallel_gripper import ParallelGripper
from isaacsim.robot.manipulators.controllers.pick_place_controller import PickPlaceController
import isaacsim.robot_motion.motion_generation as mg

HUSKY      = "/husky"
LINK0      = HUSKY + "/panda/panda_link0"
EE_PRIM    = HUSKY + "/panda/panda_rightfinger"
TOP_PLATE  = HUSKY + "/base_link/top_plate_link"
FLOOR_PRIM = "/SimpleRoom/Towel_Room01_floor_bottom_218/Towel_Room01_floor_bottom"
CUBE       = "/PickCube"
STAND      = "/PickStand"
CUBE_SIZE  = 0.05
STAND_HEIGHT = 0.35
STAND_XY     = 0.25
STAND_FORWARD = 0.70
INIT_WAIT_STEPS = 10
PLACE_OFFSET_TOPLATE_LOCAL = Gf.Vec3d(0.0, 0.18, 0.05)   # original (broken) place — keep for honesty

# === scene setup ============================================================
stage = omni.usd.get_context().get_stage()
xc = UsdGeom.XformCache()

xc.Clear()
floor_z = float(xc.GetLocalToWorldTransform(stage.GetPrimAtPath(FLOOR_PRIM)).ExtractTranslation()[2])
hpos    = xc.GetLocalToWorldTransform(stage.GetPrimAtPath(HUSKY)).ExtractTranslation()

stand_pos = np.array([float(hpos[0]),
                      float(hpos[1]) + STAND_FORWARD,
                      floor_z + STAND_HEIGHT/2])
if stage.GetPrimAtPath(STAND): stage.RemovePrim(STAND)
stand = FixedCuboid(prim_path=STAND, position=stand_pos,
                    scale=np.array([STAND_XY, STAND_XY, STAND_HEIGHT]),
                    size=1.0, color=np.array([0.7, 0.7, 0.72]))

cube_z = floor_z + STAND_HEIGHT + CUBE_SIZE/2 + 0.005
cube_pos_world = np.array([float(hpos[0]),
                           float(hpos[1]) + STAND_FORWARD,
                           cube_z])
if stage.GetPrimAtPath(CUBE): stage.RemovePrim(CUBE)
cube = DynamicCuboid(prim_path=CUBE, position=cube_pos_world,
                     scale=np.array([CUBE_SIZE]*3), size=1.0,
                     color=np.array([0.1, 0.3, 0.9]))
print(f"[RMP] floor z={floor_z:+.4f}  cube spawn={cube_pos_world.tolist()}")

# === robot + RMPflow (the SLOW cspace controller) ===========================
robot    = SingleArticulation(prim_path=HUSKY, name="husky_franka_robot")
rmp_cfg  = mg.interface_config_loader.load_supported_motion_policy_config("Franka", "RMPflow")
rmp_flow = mg.lula.motion_policies.RmpFlow(**rmp_cfg)
art_rmp  = mg.ArticulationMotionPolicy(robot, rmp_flow, default_physics_dt=1.0/60.0)

gripper = ParallelGripper(
    end_effector_prim_path=EE_PRIM,
    joint_prim_names=["panda_finger_joint1", "panda_finger_joint2"],
    joint_opened_positions=np.array([0.04, 0.04]),
    joint_closed_positions=np.array([0.0, 0.0]),
    action_deltas=np.array([0.01, 0.01]),
)

class _RMPFlowCSpace:
    """Wraps RMPflow as a cspace controller for PickPlaceController.
    Note: RMPflow expects panda_link0-LOCAL target coordinates."""
    def forward(self, target_end_effector_position, target_end_effector_orientation):
        rmp_flow.set_end_effector_target(
            np.asarray(target_end_effector_position, dtype=np.float64),
            np.asarray(target_end_effector_orientation, dtype=np.float64),
        )
        rmp_flow.update_world()
        return art_rmp.get_next_articulation_action()
    def reset(self):
        try: rmp_flow.reset()
        except Exception: pass

ctrl = PickPlaceController(
    name="pp_husky_rmp",
    cspace_controller=_RMPFlowCSpace(),
    gripper=gripper,
    end_effector_initial_height=0.20,    # original (will be too low; demonstrates the issue)
    events_dt=[0.01, 0.008, 1, 0.05, 0.02, 0.02, 0.008, 1, 0.01, 0.05],
)

ARM_NAMES = [f"panda_joint{i}" for i in range(1, 8)]
state = {"init": False, "settle": 0, "arm_idx": None, "step": 0,
         "done_logged": False, "active": True}

def _M(prim_path):
    xc.Clear()
    return xc.GetLocalToWorldTransform(stage.GetPrimAtPath(prim_path))

def link0_world_pose():
    M = _M(LINK0); t = M.ExtractTranslation(); q = M.ExtractRotationQuat()
    return (np.array([t[0], t[1], t[2]]),
            np.array([q.GetReal(), *q.GetImaginary()]))

def world_to_link0(world_xyz):
    M = _M(LINK0)
    p = M.GetInverse().Transform(Gf.Vec3d(float(world_xyz[0]),
                                          float(world_xyz[1]),
                                          float(world_xyz[2])))
    return np.array([p[0], p[1], p[2]])

def top_plate_local_to_link0(local_xyz):
    p_w = _M(TOP_PLATE).Transform(local_xyz)
    return world_to_link0([p_w[0], p_w[1], p_w[2]])

def _post_init():
    robot.initialize()
    state["arm_idx"] = np.array([robot.get_dof_index(n) for n in ARM_NAMES])
    bp, bq = link0_world_pose()
    rmp_flow.set_robot_base_pose(robot_position=bp, robot_orientation=bq)
    gripper.initialize(
        physics_sim_view=None,
        articulation_apply_action_func=robot.apply_action,
        get_joint_positions_func=robot.get_joint_positions,
        set_joint_positions_func=robot.set_joint_positions,
        dof_names=robot.dof_names,
    )
    print(f"[RMP] init OK. arm_idx={state['arm_idx'].tolist()}")
    state["init"] = True

def on_physics_step(dt):
    if not state["active"]: return
    try:
        if not state["init"]:
            state["settle"] += 1
            if state["settle"] < INIT_WAIT_STEPS: return
            try: _post_init()
            except Exception as e:
                print(f"[RMP] init pending (step {state['settle']}):", e); return

        bp, bq = link0_world_pose()
        rmp_flow.set_robot_base_pose(robot_position=bp, robot_orientation=bq)

        cube_w     = cube.get_world_pose()[0]
        pick_link0 = world_to_link0(cube_w)              # RMPflow uses local
        place_link0= top_plate_local_to_link0(PLACE_OFFSET_TOPLATE_LOCAL)

        cur = robot.get_joint_positions()
        if cur is None: return

        action = ctrl.forward(
            picking_position=pick_link0,
            placing_position=place_link0,
            current_joint_positions=cur,
            end_effector_offset=np.array([0, 0.005, 0]),
        )

        # Bump step early so progress is visible even when EE is barely moving.
        state["step"] += 1
        if state["step"] <= 5 or state["step"] % 30 == 0:
            ev = ctrl.get_current_event()
            ee_t = _M(EE_PRIM).ExtractTranslation()
            print(f"[RMP] step {state['step']}  phase={ev}  EE_z={float(ee_t[2]):+.3f}  cube_z={float(cube_w[2]):+.3f}")

        jp = action.joint_positions
        if jp is None: return
        jp_list = list(jp)
        if all(v is None for v in jp_list): return

        n = len(jp_list)
        if n == 7:
            arr = np.array(jp_list, dtype=float)
            robot.apply_action(ArticulationAction(joint_positions=arr,
                                                  joint_indices=state["arm_idx"]))
        else:
            robot.apply_action(action)

        if ctrl.is_done() and not state["done_logged"]:
            print("[RMP] PICK & PLACE DONE (cube_z barely changed — RMPflow was too slow to grasp).")
            state["done_logged"] = True
    except Exception as e:
        print(f"[RMP] step error (continuing): {e}")

# Clean previous subscription, then re-subscribe.
for n in ("_pp_sub", "_reach_sub_id"):
    if n in globals():
        try: globals()[n] = None
        except Exception: pass
        try: del globals()[n]
        except Exception: pass

_pp_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(on_physics_step)
print("[RMP] subscribed. Press PLAY.  EXPECTED: EE crawls, cube_z stays ~+0.42, no grasp.")
