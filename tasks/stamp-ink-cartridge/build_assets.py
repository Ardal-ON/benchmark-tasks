"""Author a self-inking stamp with an open cartridge slot and spring-return cap."""

from pathlib import Path

from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdShade, Sdf


def material(stage, name, color):
    path = f"/Stamp/Materials/{name}"
    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path + "/Surface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.62)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def piece(stage, path, center, size, material, collision=True):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.AddTranslateOp().Set(Gf.Vec3d(*center))
    cube.AddScaleOp().Set(Gf.Vec3f(*size))
    prim = cube.GetPrim()
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)
    if collision:
        UsdPhysics.CollisionAPI.Apply(prim)
        prim.AddAppliedSchema("PhysxCollisionAPI")
        prim.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float).Set(0.0007)


def link(stage, path, mass):
    prim = UsdGeom.Xform.Define(stage, path).GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
    return prim.GetPath()


def build(path):
    if path.exists():
        path.unlink()
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, "/Stamp").GetPrim())
    frame = link(stage, "/Stamp/Frame", 0.12)
    cap = link(stage, "/Stamp/SpringCap", 0.052)
    navy = material(stage, "ABS", (0.025, 0.11, 0.22))
    grey = material(stage, "Frame", (0.09, 0.11, 0.12))
    white = material(stage, "WindowInsert", (0.83, 0.86, 0.85))
    for sign, name in ((-1, "Left"), (1, "Right")):
        piece(stage, f"{frame}/{name}Foot", (sign * .035, 0, .004), (.010, .056, .008), grey)
        piece(stage, f"{frame}/{name}Pillar", (sign * .035, 0, .029), (.009, .052, .048), navy)
        piece(stage, f"{frame}/{name}Guide", (sign * .024, 0, .040), (.012, .040, .004), grey)
        piece(stage, f"{frame}/{name}GuideCeiling", (sign * .024, 0, .0565), (.012, .040, .004), grey,
              collision=False)
        piece(stage, f"{frame}/{name}SideGuide", (sign * .029, 0, .047), (.004, .026, .009), grey,
              collision=False)
        piece(stage, f"{cap}/{name}Carrier", (sign * .043, 0, .066), (.005, .041, .028), navy)
    piece(stage, f"{frame}/FixedFrontRail", (0, -.026, .014), (.065, .004, .014), navy)
    piece(stage, f"{frame}/FixedRearRail", (0, .026, .014), (.065, .004, .014), navy)
    piece(stage, f"{cap}/Grip", (0, 0, .083), (.092, .046, .014), navy)
    piece(stage, f"{cap}/Label", (0, 0, .0903), (.050, .018, .0006), white)
    # The raised inlet gates prevent feeding a fresh pad. The outlet stays open to the catch tray.
    for side, end in ((-1, "Left"), (1, "Right")):
        piece(stage, f"{cap}/Inlet{end}Gate", (side * .020, -.0257, .047), (.008, .004, .008), navy)
    anchor = UsdPhysics.FixedJoint.Define(stage, "/Stamp/Anchor")
    anchor.CreateBody1Rel().SetTargets([frame])
    UsdPhysics.ArticulationRootAPI.Apply(anchor.GetPrim())
    slide = UsdPhysics.PrismaticJoint.Define(stage, "/Stamp/cap_slide")
    slide.CreateAxisAttr("Z")
    slide.CreateBody0Rel().SetTargets([frame])
    slide.CreateBody1Rel().SetTargets([cap])
    slide.CreateLowerLimitAttr(-.016)
    slide.CreateUpperLimitAttr(0.0)
    slide.CreateCollisionEnabledAttr(False)
    spring = UsdPhysics.DriveAPI.Apply(slide.GetPrim(), "linear")
    spring.CreateTypeAttr("force")
    spring.CreateStiffnessAttr(550.0)
    spring.CreateDampingAttr(8.0)
    spring.CreateMaxForceAttr(30.0)
    stage.GetRootLayer().Save()


def build_cartridge(path):
    if path.exists():
        path.unlink()
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/Stamp").GetPath()
    stage.SetDefaultPrim(stage.GetPrimAtPath(root))
    guide = link(stage, f"{root}/GuideAnchor", .10)
    carrier = link(stage, f"{root}/Cartridge", .010)
    blue = material(stage, "FreshPad", (.16, .40, .57))
    piece(stage, f"{carrier}/Pad", (0, 0, 0), (.042, .049, .008), blue)
    piece(stage, f"{carrier}/ThumbTab", (-.033, 0, 0), (.029, .018, .006), blue)
    piece(stage, f"{carrier}/ThumbFlange", (-.043, 0, .004), (.009, .033, .034), blue)
    anchor = UsdPhysics.FixedJoint.Define(stage, f"{root}/Anchor")
    anchor.CreateBody1Rel().SetTargets([guide])
    UsdPhysics.ArticulationRootAPI.Apply(anchor.GetPrim())
    slide = UsdPhysics.PrismaticJoint.Define(stage, f"{root}/pad_slide")
    slide.CreateAxisAttr("X")
    slide.CreateBody0Rel().SetTargets([guide])
    slide.CreateBody1Rel().SetTargets([carrier])
    slide.CreateLowerLimitAttr(0.0)
    slide.CreateUpperLimitAttr(.075)
    stage.GetRootLayer().Save()


if __name__ == "__main__":
    build(Path(__file__).resolve().with_name("stamp.usda"))
    build_cartridge(Path(__file__).resolve().with_name("fresh-pad.usda"))
