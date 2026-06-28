# Lift /husky and /SimpleRoom so the SimpleRoom floor sits at z=0.
# Run at STOP, then File > Save (Ctrl+S) to persist into husky_franka.usd.
# After PLAY, PhysX will drop the husky onto the floor; re-Save the settled pose if you want it permanent.
# Re-runnable: skips if the floor is already at z≈0.

import omni.usd
from pxr import UsdGeom, Gf

stage = omni.usd.get_context().get_stage()
xc = UsdGeom.XformCache()

FLOOR_PATH = "/SimpleRoom/Floor/SM_Template_Map_Floor"
TARGETS    = ["/husky", "/SimpleRoom"]
REMOVE     = ["/PickCube"]   # transient; reach script will respawn

floor_prim = stage.GetPrimAtPath(FLOOR_PATH)
if not floor_prim or not floor_prim.IsValid():
    raise RuntimeError(f"floor prim not found: {FLOOR_PATH}")
floor_z = float(xc.GetLocalToWorldTransform(floor_prim).ExtractTranslation()[2])
print(f"[lift] current floor z = {floor_z:+.4f}")

if abs(floor_z) < 0.005:
    print("[lift] floor already at z≈0 — nothing to do.")
else:
    LIFT = -floor_z
    print(f"[lift] applying +{LIFT:.4f} m to: {TARGETS}")
    for p in REMOVE:
        if stage.GetPrimAtPath(p):
            stage.RemovePrim(p); print(f"  removed {p}")

    def add_z_translate(path, dz):
        pr = stage.GetPrimAtPath(path)
        if not pr or not pr.IsValid():
            print(f"  skip: {path} not found"); return
        xf = UsdGeom.Xformable(pr)
        t_op = None
        for op in xf.GetOrderedXformOps():
            if op.GetOpName() == "xformOp:translate":
                t_op = op; break
        if t_op is None:
            t_op = xf.AddTranslateOp()
        cur = t_op.Get() if t_op.Get() is not None else Gf.Vec3d(0,0,0)
        new = Gf.Vec3d(cur[0], cur[1], cur[2] + dz)
        t_op.Set(new)
        print(f"  {path}:  {tuple(round(c,3) for c in cur)}  ->  {tuple(round(c,3) for c in new)}")

    for p in TARGETS:
        add_z_translate(p, LIFT)

# Verify
xc.Clear()
for p in [FLOOR_PATH, "/husky", "/husky/front_left_wheel", "/husky/panda/panda_link0"]:
    pr = stage.GetPrimAtPath(p)
    if pr and pr.IsValid():
        z = float(xc.GetLocalToWorldTransform(pr).ExtractTranslation()[2])
        print(f"  new world z  {p:50s} = {z:+.3f}")

print("\n>>> Ctrl+S to save.  Then PLAY: husky should fall to the floor.")
print(">>> If it stays hovering, tell me — we'll set its initial z explicitly.")
