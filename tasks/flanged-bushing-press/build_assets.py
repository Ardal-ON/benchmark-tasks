"""Build the press overlay and geometry for a staged insertion prototype."""

from math import cos, pi, sin
from pathlib import Path

from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

ASSETS = Path(__file__).resolve().parent / "assets"
TRAVEL = 0.015
PITCH_RADIUS = 0.020
HOUSING_TOP = 0.075
FLANGE_HEIGHT = 0.0015
FIT_FORCE = 35.0  # Prototype setting. Replace with the measured value before evaluation.
FIT_STIFFNESS = 280_000.0


def stage_at(name, root):
    path = ASSETS / name
    if path.exists():
        path.unlink()
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, root).GetPrim())
    return stage


def material(stage, path, color):
    mat = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path + "/Surface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.48)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return mat


def surface(prim, mat):
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)
    UsdPhysics.CollisionAPI.Apply(prim)
    api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
    api.CreateContactOffsetAttr(0.0004)
    api.CreateRestOffsetAttr(0.0)


def cylinder(stage, path, mat, radius, length, translation, axis="Z"):
    geom = UsdGeom.Cylinder.Define(stage, path)
    geom.CreateAxisAttr(axis)
    geom.CreateRadiusAttr(radius)
    geom.CreateHeightAttr(length)
    geom.AddTranslateOp().Set(Gf.Vec3d(*translation))
    surface(geom.GetPrim(), mat)


def annulus(stage, path, mat, profiles, square=False, count=24):
    """Convex sectors keep both physical bores open. Profiles hold z, inner and outer radii."""
    for sector in range(count):
        angles = tuple(2 * pi * i / count for i in (sector, sector + 1))
        points = []
        for z, inner, outer in profiles:
            for theta, radius in zip((*angles, *reversed(angles)), (inner, inner, outer, outer)):
                x, y = cos(theta), sin(theta)
                if square and radius == outer:
                    radius /= max(abs(x), abs(y))
                points.append(Gf.Vec3f(radius * x, radius * y, z))
        mesh = UsdGeom.Mesh.Define(stage, f"{path}/sector_{sector:02d}")
        mesh.CreatePointsAttr(points)
        mesh.CreateSubdivisionSchemeAttr("none")
        # The lowest section faces down. Each successive profile adds four wall faces.
        faces = [[0, 1, 2, 3], list(reversed(range(len(points) - 4, len(points))))]
        for base in range(0, len(points) - 4, 4):
            for edge in range(4):
                a = base + edge
                b = base + (edge + 1) % 4
                faces.append([b, a, a + 4, b + 4])
        mesh.CreateFaceVertexCountsAttr([4] * len(faces))
        mesh.CreateFaceVertexIndicesAttr([i for face in faces for i in face])
        mesh.CreateDoubleSidedAttr(True)
        surface(mesh.GetPrim(), mat)
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("convexHull")


def anchor(stage, root, body):
    joint = UsdPhysics.FixedJoint.Define(stage, root + "/WorldAnchor")
    joint.CreateBody1Rel().SetTargets([Sdf.Path(body)])
    UsdPhysics.ArticulationRootAPI.Apply(joint.GetPrim())
    api = PhysxSchema.PhysxArticulationAPI.Apply(joint.GetPrim())
    api.CreateSolverPositionIterationCountAttr(24)
    api.CreateSolverVelocityIterationCountAttr(8)
    return api


def passive_drive(prim, axis, damping=0.0, stiffness=0.0, max_force=1000.0):
    drive = UsdPhysics.DriveAPI.Apply(prim, axis)
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(stiffness)
    drive.CreateDampingAttr(damping)
    drive.CreateMaxForceAttr(max_force)
    drive.CreateTargetPositionAttr(0.0)
    drive.CreateTargetVelocityAttr(0.0)


def build_press():
    stage = stage_at("press.usda", "/Press")
    imported = stage.DefinePrim("/Press/Imported", "Xform")
    imported.GetReferences().AddReference("./source_press.usdc")
    links = "/Press/Imported/Links"
    joints = "/Press/Imported/Joints"
    stage.GetPrimAtPath(links + "/frame").RemoveAPI(UsdPhysics.ArticulationRootAPI)
    anchor(stage, "/Press", links + "/frame")
    lever = UsdPhysics.RevoluteJoint.Get(stage, joints + "/lever_pinion")
    ram = UsdPhysics.PrismaticJoint.Get(stage, joints + "/ram_slide")
    lever.CreateLowerLimitAttr(0.0)
    lever.CreateUpperLimitAttr(100.0)
    passive_drive(lever.GetPrim(), "angular", damping=0.002, stiffness=0.015, max_force=1.0)
    passive_drive(ram.GetPrim(), "linear", damping=3.0)

    steel = material(stage, "/Press/ExtensionSteel", (0.44, 0.47, 0.50))
    grip = material(stage, "/Press/ExtensionGrip", (0.11, 0.12, 0.12))
    # Model a bench-press handle with a 160 mm grip radius and the original hub.
    cylinder(stage, links + "/lever/Extension", steel, 0.006, 0.112, (0.063, -0.102, 0), axis="Y")
    cylinder(stage, links + "/lever/Grip", grip, 0.013, 0.036, (0.063, -0.160, 0), axis="Y")
    # This nose adapter brings the press load onto the sleeve flange.
    cylinder(stage, links + "/ram/FlatPlaten", steel, 0.0173, 0.006, (0, 0, -0.003))
    coupling = PhysxSchema.PhysxPhysicsRackAndPinionJoint.Define(stage, "/Press/Transmission")
    coupling.CreateBody0Rel().SetTargets([Sdf.Path(links + "/lever")])
    coupling.CreateBody1Rel().SetTargets([Sdf.Path(links + "/ram")])
    coupling.CreateHingeRel().SetTargets([lever.GetPath()])
    coupling.CreatePrismaticRel().SetTargets([ram.GetPath()])
    coupling.CreateRatioAttr(180 / (pi * PITCH_RADIUS))
    coupling.CreateExcludeFromArticulationAttr(True)
    stage.GetRootLayer().Save()


def rigid_link(stage, path, mass, translation=(0, 0, 0)):
    link = UsdGeom.Xform.Define(stage, path)
    link.AddTranslateOp().Set(Gf.Vec3d(*translation))
    UsdPhysics.RigidBodyAPI.Apply(link.GetPrim())
    UsdPhysics.MassAPI.Apply(link.GetPrim()).CreateMassAttr(mass)
    return link.GetPrim()


def build_fit():
    stage = stage_at("fit.usda", "/Fit")
    housing = rigid_link(stage, "/Fit/Housing", 0.54)
    sleeve = rigid_link(stage, "/Fit/Bushing", 0.006, (0, 0, TRAVEL))
    art = anchor(stage, "/Fit", str(housing.GetPath()))
    art.CreateEnabledSelfCollisionsAttr(True)
    steel = material(stage, "/Fit/Steel", (0.34, 0.42, 0.49))
    polymer = material(stage, "/Fit/Polymer", (0.76, 0.62, 0.32))
    annulus(stage, "/Fit/Housing/Wall", steel,
            [(-0.039, 0.01425, 0.026), (-0.002, 0.01425, 0.026), (0, 0.0155, 0.026)], square=True)
    annulus(stage, "/Fit/Bushing/Sleeve", polymer, [(-TRAVEL, 0.01255, 0.014), (0, 0.01255, 0.014)])
    annulus(stage, "/Fit/Bushing/Flange", polymer, [(0, 0.01255, 0.0175), (FLANGE_HEIGHT, 0.01255, 0.0175)])
    fit = UsdPhysics.PrismaticJoint.Define(stage, "/Fit/fit_slide")
    fit.CreateAxisAttr("Z")
    fit.CreateBody0Rel().SetTargets([housing.GetPath()])
    fit.CreateBody1Rel().SetTargets([sleeve.GetPath()])
    fit.CreateLocalPos0Attr(Gf.Vec3f(0, 0, TRAVEL))
    fit.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    fit.CreateLowerLimitAttr(-TRAVEL - 0.00025)
    fit.CreateUpperLimitAttr(0.0)
    fit.CreateCollisionEnabledAttr(True)
    passive_drive(fit.GetPrim(), "linear", damping=20.0, stiffness=FIT_STIFFNESS, max_force=FIT_FORCE)
    stage.GetRootLayer().Save()


if __name__ == "__main__":
    build_press()
    build_fit()
