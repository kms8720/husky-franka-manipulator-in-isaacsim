# Phase C — mobile Husky + Franka pick/place test.
# Drive Husky toward a fixed stand/cube with the virtual joystick.  When the
# cube enters the arm's reachable band, the script starts the same IK-based
# pick/place sequence and places the cube on Husky's top plate.
#
# Procedure:
#   1) Open husky_franka.usd and press STOP.
#   2) Script Editor:
#        exec(open('/home/user/Desktop/260527 KMS/pick_place_husky_mobile.py').read())
#   3) Press PLAY, then slowly drive Husky forward with the joystick.

import numpy as np
import omni.usd, omni.physx
from pxr import Gf, UsdGeom

from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.manipulators.grippers.parallel_gripper import ParallelGripper
from isaacsim.robot.manipulators.controllers.pick_place_controller import PickPlaceController
import isaacsim.robot_motion.motion_generation as mg

HUSKY      = "/husky"
LINK0      = HUSKY + "/panda/panda_link0"
EE_PRIM    = HUSKY + "/panda/panda_rightfinger"
TOP_PLATE  = HUSKY + "/base_link/top_plate_link"
FLOOR_PRIM = "/SimpleRoom/Towel_Room01_floor_bottom_218/Towel_Room01_floor_bottom"
CUBE       = "/MobilePickCube"
STAND      = "/MobilePickStand"

CUBE_SIZE = 0.05
STAND_HEIGHT = 0.35
STAND_XY = 0.25
STAND_FORWARD_INITIAL = 1.15

INIT_WAIT_STEPS = 10
REACH_MIN_XY = 0.45
REACH_MAX_XY = 0.72
PLACE_OFFSET_TOPLATE_LOCAL = Gf.Vec3d(0.30, 0.0, 0.10)
TOOL_CENTER_OFFSET_Z = 0.103
HOME_Q = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])

stage = omni.usd.get_context().get_stage()
xc = UsdGeom.XformCache()

xc.Clear()
floor_z = float(xc.GetLocalToWorldTransform(stage.GetPrimAtPath(FLOOR_PRIM)).ExtractTranslation()[2])
hpos = xc.GetLocalToWorldTransform(stage.GetPrimAtPath(HUSKY)).ExtractTranslation()

stand_pos = np.array([
    float(hpos[0]),
    float(hpos[1]) + STAND_FORWARD_INITIAL,
    floor_z + STAND_HEIGHT / 2.0,
])
for prim_path in (CUBE, STAND):
    if stage.GetPrimAtPath(prim_path):
        stage.RemovePrim(prim_path)

stand = FixedCuboid(
    prim_path=STAND,
    position=stand_pos,
    scale=np.array([STAND_XY, STAND_XY, STAND_HEIGHT]),
    size=1.0,
    color=np.array([0.7, 0.7, 0.72]),
)

cube_z = floor_z + STAND_HEIGHT + CUBE_SIZE / 2.0 + 0.005
cube_pos_world = np.array([stand_pos[0], stand_pos[1], cube_z])
cube = DynamicCuboid(
    prim_path=CUBE,
    position=cube_pos_world,
    scale=np.array([CUBE_SIZE] * 3),
    size=1.0,
    color=np.array([0.1, 0.3, 0.9]),
)
print(
    "[M] spawned fixed stand/cube. Drive Husky forward until reach_xy is "
    f"{REACH_MIN_XY:.2f}~{REACH_MAX_XY:.2f} m. cube={cube_pos_world.tolist()}"
)

robot = SingleArticulation(prim_path=HUSKY, name="husky_franka_robot_mobile")

ks_cfg = mg.interface_config_loader.load_supported_lula_kinematics_solver_config("Franka")
kinematics_solver = mg.lula.LulaKinematicsSolver(**ks_cfg)

gripper = ParallelGripper(
    end_effector_prim_path=EE_PRIM,
    joint_prim_names=["panda_finger_joint1", "panda_finger_joint2"],
    joint_opened_positions=np.array([0.04, 0.04]),
    joint_closed_positions=np.array([0.022, 0.022]),
    action_deltas=None,
)

ARM_NAMES = [f"panda_joint{i}" for i in range(1, 8)]
mobile_state = {
    "init": False,
    "settle": 0,
    "arm_idx": None,
    "step": 0,
    "wait_step": 0,
    "started": False,
    "done_logged": False,
    "active": True,
    "warned_unready": False,
}


def _M(prim_path):
    xc.Clear()
    return xc.GetLocalToWorldTransform(stage.GetPrimAtPath(prim_path))


def _wpos(prim_path):
    t = _M(prim_path).ExtractTranslation()
    return np.array([float(t[0]), float(t[1]), float(t[2])])


def link0_world_pose():
    M = _M(LINK0)
    t = M.ExtractTranslation()
    q = M.ExtractRotationQuat()
    return (
        np.array([float(t[0]), float(t[1]), float(t[2])]),
        np.array([q.GetReal(), *q.GetImaginary()]),
    )


class _MobileIKCSpace:
    def __init__(self):
        self._last_sol = None
        self._calls = 0
        self._fails = 0

    def forward(self, target_end_effector_position, target_end_effector_orientation):
        warm = HOME_Q.astype(np.float64)
        if mobile_state["arm_idx"] is not None:
            cur = robot.get_joint_positions()
            if cur is not None:
                warm = np.asarray(cur[mobile_state["arm_idx"]], dtype=np.float64)

        tp = np.asarray(target_end_effector_position, dtype=np.float64)
        to = np.asarray(target_end_effector_orientation, dtype=np.float64)
        sol, ok = kinematics_solver.compute_inverse_kinematics(
            frame_name="panda_hand",
            target_position=tp,
            target_orientation=to,
            warm_start=warm,
            position_tolerance=0.005,
            orientation_tolerance=0.20,
        )
        self._calls += 1
        if not ok:
            self._fails += 1
            if self._calls <= 3 or self._fails % 60 == 1:
                print(f"[M-IK] FAIL call#{self._calls} fails={self._fails} target={tp.tolist()}")
            jp = self._last_sol if self._last_sol is not None else [None] * 7
            return ArticulationAction(joint_positions=jp)

        if self._calls <= 3:
            print(f"[M-IK] OK call#{self._calls} sol={np.round(sol, 3).tolist()}")
        self._last_sol = sol
        return ArticulationAction(joint_positions=sol)

    def reset(self):
        self._last_sol = None
        self._calls = 0
        self._fails = 0


ctrl = PickPlaceController(
    name="pp_husky_mobile",
    cspace_controller=_MobileIKCSpace(),
    gripper=gripper,
    end_effector_initial_height=cube_z + 0.15,
    events_dt=[0.01, 0.008, 1, 0.05, 0.02, 0.02, 0.008, 1, 0.01, 0.05],
)


def _post_init():
    robot.initialize()
    mobile_state["arm_idx"] = np.array([robot.get_dof_index(n) for n in ARM_NAMES])
    bp, bq = link0_world_pose()
    kinematics_solver.set_robot_base_pose(robot_position=bp, robot_orientation=bq)
    gripper.initialize(
        physics_sim_view=None,
        articulation_apply_action_func=robot.apply_action,
        get_joint_positions_func=robot.get_joint_positions,
        set_joint_positions_func=robot.set_joint_positions,
        dof_names=robot.dof_names,
    )
    print(f"[M] init OK. arm_idx={mobile_state['arm_idx'].tolist()}")
    print("[M] waiting for cube to enter reach band. Drive slowly; stop near the cube for a cleaner grasp.")
    mobile_state["init"] = True
    mobile_state["warned_unready"] = False


def _mark_physics_unready():
    if not mobile_state["warned_unready"]:
        print("[M] physics view is not ready. If you pressed STOP, re-exec the script before PLAY for a clean restart.")
        mobile_state["warned_unready"] = True
    mobile_state["init"] = False
    mobile_state["settle"] = 0
    mobile_state["arm_idx"] = None


def _reach_xy(cube_w, link0_w):
    return float(np.linalg.norm(np.asarray(cube_w[:2]) - np.asarray(link0_w[:2])))


def on_mobile_physics_step(dt):
    if not mobile_state["active"]:
        return
    try:
        if not mobile_state["init"]:
            mobile_state["settle"] += 1
            if mobile_state["settle"] < INIT_WAIT_STEPS:
                return
            try:
                _post_init()
            except Exception as e:
                print(f"[M] init pending (step {mobile_state['settle']}): {e}")
                return

        bp, bq = link0_world_pose()
        kinematics_solver.set_robot_base_pose(robot_position=bp, robot_orientation=bq)

        cube_w = np.asarray(cube.get_world_pose()[0], dtype=np.float64)
        reach_xy = _reach_xy(cube_w, bp)

        if not mobile_state["started"]:
            mobile_state["wait_step"] += 1
            if mobile_state["wait_step"] <= 5 or mobile_state["wait_step"] % 30 == 0:
                print(
                    f"[M] WAIT reach_xy={reach_xy:.3f} m "
                    f"(target {REACH_MIN_XY:.2f}~{REACH_MAX_XY:.2f})"
                )
            if REACH_MIN_XY <= reach_xy <= REACH_MAX_XY:
                mobile_state["started"] = True
                print(f"[M] START pick/place. reach_xy={reach_xy:.3f} m")
            else:
                return

        place_w = _M(TOP_PLATE).Transform(PLACE_OFFSET_TOPLATE_LOCAL)
        place_world = np.array([float(place_w[0]), float(place_w[1]), float(place_w[2])])

        cur = robot.get_joint_positions()
        if cur is None:
            _mark_physics_unready()
            return

        action = ctrl.forward(
            picking_position=cube_w,
            placing_position=place_world,
            current_joint_positions=cur,
            end_effector_offset=np.array([0.0, 0.0, TOOL_CENTER_OFFSET_Z]),
        )

        mobile_state["step"] += 1
        if mobile_state["step"] <= 5 or mobile_state["step"] % 30 == 0:
            ev = ctrl.get_current_event()
            ee_z = float(_wpos(EE_PRIM)[2])
            print(
                f"[M] step {mobile_state['step']} phase={ev} "
                f"reach_xy={reach_xy:.3f} EE_z={ee_z:+.3f} cube_z={float(cube_w[2]):+.3f}"
            )

        jp = action.joint_positions
        if jp is None:
            return
        jp_list = list(jp)
        if all(v is None for v in jp_list):
            return

        if len(jp_list) == 7:
            robot.apply_action(
                ArticulationAction(
                    joint_positions=np.array(jp_list, dtype=float),
                    joint_indices=mobile_state["arm_idx"],
                )
            )
        else:
            robot.apply_action(action)

        if ctrl.is_done() and not mobile_state["done_logged"]:
            print("[M] MOBILE PICK & PLACE DONE.")
            mobile_state["done_logged"] = True
    except Exception as e:
        print(f"[M] step error (continuing): {e}")


for n in ("_mobile_pp_sub", "_pp_sub", "_reach_sub_id"):
    if n in globals():
        try:
            globals()[n] = None
        except Exception:
            pass
        try:
            del globals()[n]
        except Exception:
            pass

_mobile_pp_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(on_mobile_physics_step)
print("[M] subscribed. Press PLAY, then drive Husky toward the stand.")
