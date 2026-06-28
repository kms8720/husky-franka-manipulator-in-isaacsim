# Quick STOP-time pose diagnostic. No physics needed.
# Run with husky_franka.usd open, timeline STOPPED:
#   exec(open('/home/user/Desktop/260527 KMS/diagnose_poses.py').read())

import omni.usd
from pxr import UsdGeom, UsdPhysics, Sdf
stage = omni.usd.get_context().get_stage()
xc = UsdGeom.XformCache()
def show(p):
    pr = stage.GetPrimAtPath(p)
    if not pr or not pr.IsValid():
        print(f"  {p}: MISSING"); return
    t = xc.GetLocalToWorldTransform(pr).ExtractTranslation()
    print(f"  {p}: ({t[0]:+.3f}, {t[1]:+.3f}, {t[2]:+.3f})")

print("=== world translations (USD authored, no physics) ===")
for p in ["/husky", "/husky/base_link", "/husky/base_link/top_plate_link",
          "/husky/panda", "/husky/panda/panda_link0",
          "/husky/panda/panda_hand", "/husky/panda/panda_rightfinger"]:
    show(p)

print("=== articulation roots ===")
for pr in stage.Traverse():
    if pr.HasAPI(UsdPhysics.ArticulationRootAPI):
        print(f"  {pr.GetPath()} [{pr.GetTypeName()}]")
print("=== active rootJoint? ===")
rj = stage.GetPrimAtPath("/husky/panda/rootJoint")
print(f"  /husky/panda/rootJoint: exists={bool(rj) and rj.IsValid()}, active={rj.IsActive() if rj and rj.IsValid() else None}")
print("=== panda_mount_joint ===")
mj = stage.GetPrimAtPath("/husky/panda/panda_mount_joint")
if mj and mj.IsValid():
    j = UsdPhysics.Joint(mj)
    print(f"  body0={list(j.GetBody0Rel().GetTargets())}")
    print(f"  body1={list(j.GetBody1Rel().GetTargets())}")
    print(f"  localPos0={j.GetLocalPos0Attr().Get()}  localPos1={j.GetLocalPos1Attr().Get()}")
else:
    print("  MISSING — Franka mount joint not found.")
