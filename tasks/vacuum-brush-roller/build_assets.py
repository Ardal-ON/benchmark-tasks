"""Build a detached floor head with a removable roller and a physical end-cap lock."""

from math import cos, pi, sin
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

OUT = Path(__file__).resolve().parent
STROKE = .100


def material(stage, name, color):
    path = f"{stage.GetDefaultPrim().GetPath()}/Materials/{name}"
    mat = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path + "/Surface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(.55)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return mat


def decorate(prim, mat, collision):
    if mat is not None:
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)
    if collision:
        UsdPhysics.CollisionAPI.Apply(prim)
        prim.AddAppliedSchema("PhysxCollisionAPI")
        prim.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float).Set(.00045)


def box(stage, path, pos, size, material, collision=True, rotation=0.0):
    geom = UsdGeom.Cube.Define(stage, path)
    geom.CreateSizeAttr(1)
    geom.AddTranslateOp().Set(Gf.Vec3d(*pos))
    geom.AddRotateXOp().Set(rotation)
    geom.AddScaleOp().Set(Gf.Vec3f(*size))
    decorate(geom.GetPrim(), material, collision)


def cylinder(stage, path, pos, radius, length, material, collision=True, axis="X"):
    geom = UsdGeom.Cylinder.Define(stage, path)
    geom.CreateAxisAttr(axis)
    geom.CreateRadiusAttr(radius)
    geom.CreateHeightAttr(length)
    geom.AddTranslateOp().Set(Gf.Vec3d(*pos))
    decorate(geom.GetPrim(), material, collision)


def link(stage, path, pos, mass):
    geom = UsdGeom.Xform.Define(stage, path)
    geom.AddTranslateOp().Set(Gf.Vec3d(*pos))
    UsdPhysics.RigidBodyAPI.Apply(geom.GetPrim())
    UsdPhysics.MassAPI.Apply(geom.GetPrim()).CreateMassAttr(mass)
    geom.GetPrim().AddAppliedSchema("PhysxContactReportAPI")
    geom.GetPrim().CreateAttribute("physxContactReport:threshold", Sdf.ValueTypeNames.Float).Set(0.0)
    return geom.GetPath()


def cap_shell(stage, mat, sign):
    """A circular cap face split at the slot, with convex collision on both banks."""
    radius = .028
    angles = [a * pi / 180 for a in range(-85, 86, 5)]
    section = [(sign * radius * cos(a), radius * sin(a)) for a in angles]
    n = len(section) + 2
    points = []
    for x, half_width in ((-.010, .0031), (-.006, .0020)):
        bank = [*section, (sign * half_width, section[-1][1]), (sign * half_width, section[0][1])]
        points.extend(Gf.Vec3f(x, y, z) for y, z in bank)
    mesh = UsdGeom.Mesh.Define(stage, f"/CleanerHead/Cap/SlotBank{'A' if sign < 0 else 'B'}")
    mesh.CreatePointsAttr(points)
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDoubleSidedAttr(True)
    faces = [list(range(n)), list(reversed(range(n, n * 2)))]
    faces.extend([i, (i + 1) % n, (i + 1) % n + n, i + n] for i in range(n))
    mesh.CreateFaceVertexCountsAttr([len(f) for f in faces])
    mesh.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
    decorate(mesh.GetPrim(), mat, True)
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("convexHull")


def build():
    path = OUT / "head.usda"
    if path.exists():
        path.unlink()
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, "/CleanerHead").GetPrim())
    head = link(stage, "/CleanerHead/Head", (0, 0, 0), .66)
    shell = UsdGeom.Xform.Define(stage, "/CleanerHead/StaticShell").GetPath()
    roller = link(stage, "/CleanerHead/Roller", (-STROKE, 0, 0), .262)
    cap = link(stage, "/CleanerHead/Cap", (-STROKE, 0, 0), .028)
    dark = material(stage, "GraphiteABS", (.08, .10, .11))
    navy = material(stage, "Roller", (.10, .18, .28))
    rubber = material(stage, "Bristles", (.63, .12, .035))
    mech = material(stage, "LockingABS", (.28, .32, .36))
    mark = material(stage, "Index", (.88, .88, .77))
    box(stage, f"{shell}/TopShell", (.109, .017, -.032), (.228, .106, .012), dark, collision=False)
    box(stage, f"{shell}/LeadingLip", (.129, -.039, -.016), (.188, .012, .044), dark, collision=False)
    box(stage, f"{shell}/ExtractionSide", (.129, .044, .0), (.188, .015, .080), dark, collision=False)
    box(stage, f"{shell}/DriveEnd", (.219, 0, 0), (.048, .070, .073), dark, collision=False)
    cylinder(stage, f"{shell}/WandPort", (.11, .069, -.010), .022, .052, dark, collision=False, axis="Y")
    for sign, suffix in ((-1, "A"), (1, "B")):
        box(stage, f"{shell}/Strike{suffix}", (.0015, 0, sign * .036), (.006, .015, .011), mech)
        box(stage, f"{cap}/Lug{suffix}", (.010, sign * .032, 0), (.006, .012, .012), mech)
    box(stage, f"{shell}/LockIndex", (-.004, -.020, .041), (.001, .009, .006), mark, collision=False)
    cylinder(stage, f"{roller}/Tube", (.096, 0, 0), .023, .188, navy)
    for index in range(28):
        phase = index * 2 * pi / 28
        for sign, suffix in ((-1, "A"), (1, "B")):
            box(stage, f"{roller}/Bristles{suffix}{index:02d}",
                (.013 + .0066 * index, sign * .0235 * cos(phase), sign * .0235 * sin(phase)),
                (.0078, .005, .012), rubber, collision=False, rotation=phase * 180 / pi)
    cylinder(stage, f"{cap}/Back", (0, 0, 0), .028, .012, mech)
    cap_shell(stage, mech, -1)
    cap_shell(stage, mech, 1)
    box(stage, f"{cap}/Pointer", (-.0103, 0, .025), (.0003, .0035, .006), mark, collision=False)

    fixed = UsdPhysics.FixedJoint.Define(stage, "/CleanerHead/Anchor")
    fixed.CreateBody1Rel().SetTargets([head])
    anchor = fixed.GetPrim()
    UsdPhysics.ArticulationRootAPI.Apply(anchor)
    anchor.AddAppliedSchema("PhysxArticulationAPI")
    anchor.CreateAttribute("physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool).Set(True)
    slide = UsdPhysics.PrismaticJoint.Define(stage, "/CleanerHead/roller_slide")
    slide.CreateAxisAttr("X")
    slide.CreateBody0Rel().SetTargets([head])
    slide.CreateBody1Rel().SetTargets([roller])
    slide.CreateLocalPos0Attr(Gf.Vec3f(-STROKE, 0, 0))
    slide.CreateLowerLimitAttr(-.003)
    slide.CreateUpperLimitAttr(STROKE)
    slide.CreateCollisionEnabledAttr(True)
    turn = UsdPhysics.RevoluteJoint.Define(stage, "/CleanerHead/cap_turn")
    turn.CreateAxisAttr("X")
    turn.CreateBody0Rel().SetTargets([roller])
    turn.CreateBody1Rel().SetTargets([cap])
    turn.CreateLowerLimitAttr(0)
    turn.CreateUpperLimitAttr(90)
    turn.CreateCollisionEnabledAttr(False)
    shell_file = OUT / "housing.usda"
    if shell_file.exists():
        shell_file.unlink()
    fixed_stage = Usd.Stage.CreateNew(str(shell_file))
    UsdGeom.SetStageUpAxis(fixed_stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(fixed_stage, 1.0)
    fixed_stage.SetDefaultPrim(UsdGeom.Xform.Define(fixed_stage, "/CleanerHead").GetPrim())
    for prim_path in (shell, Sdf.Path("/CleanerHead/Materials")):
        Sdf.CopySpec(stage.GetRootLayer(), prim_path, fixed_stage.GetRootLayer(), prim_path)
    fixed_stage.GetRootLayer().Save()
    stage.RemovePrim(shell)
    stage.GetRootLayer().Save()

    coin_path = OUT / "coin.usda"
    if coin_path.exists():
        coin_path.unlink()
    coin = Usd.Stage.CreateNew(str(coin_path))
    UsdGeom.SetStageUpAxis(coin, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(coin, 1.0)
    coin.SetDefaultPrim(UsdGeom.Xform.Define(coin, "/Coin").GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(coin.GetDefaultPrim())
    UsdPhysics.MassAPI.Apply(coin.GetDefaultPrim()).CreateMassAttr(.00567)
    silver = material(coin, "CoinAlloy", (.54, .54, .51))
    cylinder(coin, "/Coin/Blank", (0, 0, 0), .01213, .00175, silver, axis="Y")
    coin.GetRootLayer().Save()


if __name__ == "__main__":
    build()
