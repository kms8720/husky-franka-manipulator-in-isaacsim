# Phase B (v2) — full pick & place on the merged /husky articulation.
# Replaces RMPflow with direct IK (LulaKinematicsSolver) so EE motion is fast (PD-driven).
# Picks a ground-truth cube on a stand and places it on Husky's top plate.
#
# Procedure (husky_franka.usd open, floor at z=0):
#   1) STOP. 2) Script Editor: exec(open('/home/user/Desktop/260527 KMS/pick_place_husky.py').read())
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
PLACE_OFFSET_TOPLATE_LOCAL = Gf.Vec3d(0.30, 0.0, 0.10)   # top_plate local: +x=husky forward, +z=up
TOOL_CENTER_OFFSET_Z = 0.103   # panda_hand origin -> tool_center along panda_hand +z
# Franka home pose (reasonable warm start for IK)
HOME_Q = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])

# === scene setup ============================================================
stage = omni.usd.get_context().get_stage()
xc = UsdGeom.XformCache()

xc.Clear()
floor_z = float(xc.GetLocalToWorldTransform(stage.GetPrimAtPath(FLOOR_PRIM)).ExtractTranslation()[2])
hpos    = xc.GetLocalToWorldTransform(stage.GetPrimAtPath(HUSKY)).ExtractTranslation()

# Stand (static).
stand_pos = np.array([float(hpos[0]),
                      float(hpos[1]) + STAND_FORWARD,
                      floor_z + STAND_HEIGHT/2])
if stage.GetPrimAtPath(STAND): stage.RemovePrim(STAND)
stand = FixedCuboid(prim_path=STAND, position=stand_pos,
                    scale=np.array([STAND_XY, STAND_XY, STAND_HEIGHT]),
                    size=1.0, color=np.array([0.7, 0.7, 0.72]))

# Cube on top of the stand.
cube_z = floor_z + STAND_HEIGHT + CUBE_SIZE/2 + 0.005
cube_pos_world = np.array([float(hpos[0]),
                           float(hpos[1]) + STAND_FORWARD,
                           cube_z])
if stage.GetPrimAtPath(CUBE): stage.RemovePrim(CUBE)
cube = DynamicCuboid(prim_path=CUBE, position=cube_pos_world,
                     scale=np.array([CUBE_SIZE]*3), size=1.0,
                     color=np.array([0.1, 0.3, 0.9]))
print(f"[B] floor z={floor_z:+.4f}  stand top={floor_z+STAND_HEIGHT:+.4f}  cube spawn={cube_pos_world.tolist()}")

# === robot + IK solver + gripper ===========================================
robot = SingleArticulation(prim_path=HUSKY, name="husky_franka_robot")

ks_cfg = mg.interface_config_loader.load_supported_lula_kinematics_solver_config("Franka")
kinematics_solver = mg.lula.LulaKinematicsSolver(**ks_cfg)

gripper = ParallelGripper(
    end_effector_prim_path=EE_PRIM,
    joint_prim_names=["panda_finger_joint1", "panda_finger_joint2"],
    joint_opened_positions=np.array([0.04, 0.04]),
    joint_closed_positions=np.array([0.022, 0.022]),   # half cube width - small squeeze
    action_deltas=None,                                # one-shot to closed/opened (cleaner)
)

# Cspace controller wrapper around the IK solver.
# Returns one-shot joint targets that the PD drive will track quickly.
class _IKCSpace:
    def __init__(self):
        self._last_sol = None
        self._calls = 0
        self._fails = 0
    def forward(self, target_end_effector_position, target_end_effector_orientation):
        warm = HOME_Q.astype(np.float64)
        if state["arm_idx"] is not None:
            cur = robot.get_joint_positions()
            if cur is not None:
                warm = np.asarray(cur[state["arm_idx"]], dtype=np.float64)
        tp = np.asarray(target_end_effector_position, dtype=np.float64)
        to = np.asarray(target_end_effector_orientation, dtype=np.float64)
        sol, ok = kinematics_solver.compute_inverse_kinematics(
            frame_name="panda_hand",
            target_position=tp,
            target_orientation=to,
            warm_start=warm,
            position_tolerance=0.005,
            orientation_tolerance=0.20,   # ~11° — let IK trade off orientation for reachability
        )
        self._calls += 1
        if not ok:
            self._fails += 1
            if self._calls <= 3 or self._fails % 60 == 1:
                print(f"[IK] FAIL  call#{self._calls} fails={self._fails}  target_pos={tp.tolist()}")
            jp = self._last_sol if self._last_sol is not None else [None]*7
            return ArticulationAction(joint_positions=jp)
        if self._calls <= 3:
            print(f"[IK] OK  call#{self._calls}  sol={np.round(sol,3).tolist()}")
        self._last_sol = sol
        return ArticulationAction(joint_positions=sol)
    def reset(self):
        self._last_sol = None
        self._calls = 0
        self._fails = 0

# Note: events_dt slightly shorter — IK + PD is faster than RMPflow.
ctrl = PickPlaceController(
    name="pp_husky",
    cspace_controller=_IKCSpace(),
    gripper=gripper,
    end_effector_initial_height=cube_z + 0.15,              # WORLD-frame hover height
    events_dt=[0.01, 0.008, 1, 0.05, 0.02, 0.02, 0.008, 1, 0.01, 0.05],
)

ARM_NAMES = [f"panda_joint{i}" for i in range(1, 8)]
state = {"init": False, "settle": 0, "arm_idx": None, "step": 0,
         "done_logged": False, "active": True}

# === pose helpers ==========================================================
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

# === init (deferred until articulation is live) ============================
def _post_init():
    robot.initialize()
    state["arm_idx"] = np.array([robot.get_dof_index(n) for n in ARM_NAMES])
    bp, bq = link0_world_pose()
    kinematics_solver.set_robot_base_pose(robot_position=bp, robot_orientation=bq)
    gripper.initialize(
        physics_sim_view=None,
        articulation_apply_action_func=robot.apply_action,
        get_joint_positions_func=robot.get_joint_positions,
        set_joint_positions_func=robot.set_joint_positions,
        dof_names=robot.dof_names,
    )
    print(f"[B] init OK. arm_idx={state['arm_idx'].tolist()}")
    state["init"] = True

# === main step =============================================================
def on_physics_step(dt):
    if not state["active"]: return
    try:
        if not state["init"]:
            state["settle"] += 1
            if state["settle"] < INIT_WAIT_STEPS: return
            try: _post_init()
            except Exception as e:
                print(f"[B] init pending (step {state['settle']}):", e); return

        # Keep IK base pose in sync (handles future Husky motion).
        bp, bq = link0_world_pose()
        kinematics_solver.set_robot_base_pose(robot_position=bp, robot_orientation=bq)

        cube_w   = np.asarray(cube.get_world_pose()[0], dtype=np.float64)
        # Lula IK uses WORLD coordinates — pass picking/placing in world frame.
        place_w  = _M(TOP_PLATE).Transform(PLACE_OFFSET_TOPLATE_LOCAL)
        place_world = np.array([float(place_w[0]), float(place_w[1]), float(place_w[2])])

        cur = robot.get_joint_positions()
        if cur is None: return

        action = ctrl.forward(
            picking_position=cube_w,
            placing_position=place_world,
            current_joint_positions=cur,
            # Offset panda_hand up by tool_center distance so the grasp point lands on the target.
            end_effector_offset=np.array([0, 0, TOOL_CENTER_OFFSET_Z]),
        )
        # Bump step early so PHASES progress and we get logs even when IK fails.
        state["step"] += 1
        if state["step"] <= 5 or state["step"] % 30 == 0:
            ev = ctrl.get_current_event()
            ee_t = _M(EE_PRIM).ExtractTranslation()
            print(f"[B] step {state['step']}  phase={ev}  EE_z={float(ee_t[2]):+.3f}  cube_z={float(cube_w[2]):+.3f}")

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
            print("[B] PICK & PLACE DONE.")
            state["done_logged"] = True
    except Exception as e:
        print(f"[B] step error (continuing): {e}")

# === clean prior subscription (carb.Subscription RAII) =====================
for n in ("_pp_sub", "_reach_sub_id"):
    if n in globals():
        try: globals()[n] = None
        except Exception: pass
        try: del globals()[n]
        except Exception: pass

_pp_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(on_physics_step)
print("[B] subscribed. Press PLAY.")
