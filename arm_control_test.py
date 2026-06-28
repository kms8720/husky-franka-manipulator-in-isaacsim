# Sanity-test that the mounted Franka arm is controllable as part of the /husky articulation.
# Paste into Isaac Sim Script Editor.  >>> Timeline must be PLAYING (press Play first) <<<
# Only the 7 arm joints get position targets; wheels stay free for cmd_vel.

import numpy as np
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction
import omni.usd
from pxr import UsdPhysics

# locate the articulation root (handles /husky or /World/husky)
stage = omni.usd.get_context().get_stage()
root = stage.GetPrimAtPath("/husky")
if not (root and root.IsValid()):
    root = next(p for p in stage.Traverse()
                if p.GetName() == "husky" and p.HasAPI(UsdPhysics.ArticulationRootAPI))
root_path = root.GetPath().pathString

robot = SingleArticulation(prim_path=root_path, name="husky_franka")
robot.initialize()                       # needs the sim PLAYING
print("root:", root_path, "| num_dof:", robot.num_dof)
print("dof_names:", robot.dof_names)     # expect 4 wheels + panda_joint1..7 + 2 fingers

arm = [f"panda_joint{i}" for i in range(1, 8)]
idx = np.array([robot.get_dof_index(n) for n in arm])
target = np.array([1.2, -0.4, 0.0, -1.8, 0.0, 1.5, 0.8])   # distinctive pose -> visible motion
robot.apply_action(ArticulationAction(joint_positions=target, joint_indices=idx))
print("arm target sent:", dict(zip(arm, target.tolist())))
print("-> arm should swing to this pose while the base stays put.")
