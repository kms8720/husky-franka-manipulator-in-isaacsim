# Mount a Franka Panda on the Husky top_plate (single articulation).
# Paste into Isaac Sim 5.1  ->  Window > Script Editor, with husky_test scene open.
# IMPORTANT: press STOP (timeline) before running, then PLAY after. Re-runnable.

import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf

FRANKA_USD = ("/home/user/Downloads/isaac-sim-assets-robots_and_sensors-5.1.0/"
              "Assets/Isaac/5.1/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd")

# --- mount placement, expressed in the top_plate_link local frame ---
MOUNT_OFFSET = Gf.Vec3d(0.0, 0.18, 0.01)   # +y = robot forward (clears the lidar), +z = up onto plate
MOUNT_YAW_DEG = 0.0                         # rotate arm about Z later if you want it facing forward

stage = omni.usd.get_context().get_stage()

# 1) locate the husky articulation root robustly (handles /husky or /World/husky)
husky = stage.GetPrimAtPath("/husky")
if not husky or not husky.IsValid():
    husky = next((p for p in stage.Traverse()
                  if p.GetName() == "husky" and p.HasAPI(UsdPhysics.ArticulationRootAPI)), None)
assert husky, "Could not find the husky articulation root prim."
hp = husky.GetPath()
base_path  = hp.AppendChild("base_link")
tp_path    = base_path.AppendChild("top_plate_link")
panda_path = hp.AppendChild("panda")
link0_path = panda_path.AppendChild("panda_link0")
joint_path = panda_path.AppendChild("panda_mount_joint")
print(f"[mount] husky root = {hp}")

# 2) clean previous run
if stage.GetPrimAtPath(panda_path):
    stage.RemovePrim(panda_path)

# 3) reference the Franka under /husky (so its links are descendants of the artic root)
panda = stage.DefinePrim(panda_path, "Xform")
panda.GetReferences().AddReference(FRANKA_USD)
assert stage.GetPrimAtPath(link0_path), "panda_link0 not found after referencing Franka."

# 4) place panda_link0 on the top plate (compute target in /husky-local space)
xc = UsdGeom.XformCache(Usd.TimeCode.Default())
M_tp    = xc.GetLocalToWorldTransform(stage.GetPrimAtPath(tp_path))
M_husky = xc.GetLocalToWorldTransform(husky)
off = Gf.Matrix4d(1.0)
off.SetRotateOnly(Gf.Rotation(Gf.Vec3d(0, 0, 1), MOUNT_YAW_DEG))
off.SetTranslateOnly(MOUNT_OFFSET)
desired_world = off * M_tp
desired_local = desired_world * M_husky.GetInverse()
xf = UsdGeom.Xformable(panda)
xf.ClearXformOpOrder()
xf.AddTransformOp().Set(desired_local)

# 5) drop the Franka's own world-fix joint + articulation root (keep a single articulation)
# NOTE: rootJoint comes from the Franka *reference*, so stage.RemovePrim() can't delete it
#       (it only clears local opinions). Deactivate it instead -- this is exactly what the
#       GUI "Delete" does for a referenced prim.
rootjoint = stage.GetPrimAtPath(panda_path.AppendChild("rootJoint"))
if rootjoint and rootjoint.IsValid():
    rootjoint.SetActive(False)
panda.RemoveAPI(UsdPhysics.ArticulationRootAPI)
try:
    from pxr import PhysxSchema
    panda.RemoveAPI(PhysxSchema.PhysxArticulationAPI)
except Exception as e:
    print("[mount] PhysxArticulationAPI remove skipped:", e)

# 6) fixed joint base_link -> panda_link0, local frames from the current relative pose (no snap)
xc2 = UsdGeom.XformCache(Usd.TimeCode.Default())
M_link0 = xc2.GetLocalToWorldTransform(stage.GetPrimAtPath(link0_path))
M_base  = xc2.GetLocalToWorldTransform(stage.GetPrimAtPath(base_path))
rel = M_link0 * M_base.GetInverse()
q = rel.ExtractRotationQuat()
joint = UsdPhysics.FixedJoint.Define(stage, joint_path)
joint.CreateBody0Rel().SetTargets([base_path])
joint.CreateBody1Rel().SetTargets([link0_path])
joint.CreateLocalPos0Attr().Set(Gf.Vec3f(rel.ExtractTranslation()))
joint.CreateLocalRot0Attr().Set(Gf.Quatf(q.GetReal(), Gf.Vec3f(*q.GetImaginary())))
joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
joint.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))

print(f"[mount] done. Franka at {panda_path}, fixed to {base_path} via {joint_path.name}.")
print("[mount] Now press PLAY. Arm should stay on the plate and move with the Husky.")
