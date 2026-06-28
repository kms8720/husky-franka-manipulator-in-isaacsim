# Find where SimpleRoom's floor sits, list every top-level prim, and pick
# the offset needed to lift the floor to z=0. Read-only — does NOT modify the stage.

import math
import omni.usd
from pxr import UsdGeom

stage = omni.usd.get_context().get_stage()
xc = UsdGeom.XformCache()

dp = stage.GetDefaultPrim()
print(f"defaultPrim: {dp.GetPath() if dp else None}")
print(f"upAxis: {UsdGeom.GetStageUpAxis(stage)}  metersPerUnit: {UsdGeom.GetStageMetersPerUnit(stage)}")
print(f"rootLayer: {stage.GetRootLayer().identifier}")

print("\n--- TOP-LEVEL prims (children of /) ---")
top = []
for p in stage.GetPseudoRoot().GetChildren():
    try:
        t = xc.GetLocalToWorldTransform(p).ExtractTranslation()
        top.append((p.GetPath().pathString, p.GetTypeName(), float(t[2])))
    except Exception:
        top.append((p.GetPath().pathString, p.GetTypeName(), None))
for path, typ, z in top:
    print(f"  {path:40s} [{typ}]   z={z}")

print("\n--- prims whose name suggests FLOOR/GROUND/BOTTOM (anywhere) ---")
hits = []
for p in stage.Traverse():
    n = p.GetPath().pathString.lower()
    if any(k in n for k in ["look", "shader", "material"]): continue
    if any(k in n for k in ["floor_bottom", "ground", "/floor", "_floor", "edge", "rim_bottom"]):
        try:
            t = xc.GetLocalToWorldTransform(p).ExtractTranslation()
            hits.append((p.GetPath().pathString, float(t[2])))
        except Exception: pass
hits.sort(key=lambda x: x[1])
for path, z in hits[:15]:
    print(f"  {path:65s} z={z:+.3f}")

print("\n--- approximate ground = min z among all Mesh prims ---")
min_z = math.inf; min_path = None
for p in stage.Traverse():
    if p.GetTypeName() == "Mesh":
        try:
            z = float(xc.GetLocalToWorldTransform(p).ExtractTranslation()[2])
            if z < min_z:
                min_z, min_path = z, p.GetPath().pathString
        except Exception: pass
print(f"  min mesh z = {min_z:+.3f}  at  {min_path}")

print("\n--- husky bottom estimate (wheel center) ---")
for p in ["/husky/front_left_wheel", "/husky/rear_left_wheel"]:
    pr = stage.GetPrimAtPath(p)
    if pr and pr.IsValid():
        t = xc.GetLocalToWorldTransform(pr).ExtractTranslation()
        print(f"  {p:32s} world z = {float(t[2]):+.3f}")

print("\n>>> If floor is at z = Zf, lifting EVERY top-level prim by (-Zf) puts floor at 0.")
print(">>> Send me back the 'TOP-LEVEL prims' list and the 'min mesh z' value.")
