"""Check the passive roller and cap on an isolated stand. This is a bench diagnostic."""

from pathlib import Path
import json

from isaaclab.app import AppLauncher


def check():
    import torch
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import Articulation, ArticulationCfg
    from isaaclab.sim import SimulationCfg, SimulationContext, UsdFileCfg

    hz = 240
    sim = SimulationContext(SimulationCfg(dt=1 / hz, device="cpu"))
    assets = Path(__file__).resolve().parent
    housing = UsdFileCfg(usd_path=str(assets / "housing.usda"))
    housing.func("/World/BenchHousing", housing)
    cfg = ArticulationCfg(
        prim_path="/World/BenchHead",
        spawn=UsdFileCfg(usd_path=str(assets / "head.usda")),
        actuators={
            "slide": ImplicitActuatorCfg(joint_names_expr=["roller_slide"], stiffness=0.0, damping=6.0),
            "lock": ImplicitActuatorCfg(joint_names_expr=["cap_turn"], stiffness=0.0, damping=.05, friction=.08),
        },
    )
    assembly = Articulation(cfg)
    sim.reset()
    slide, turn = (assembly.joint_names.index(s) for s in ("roller_slide", "cap_turn"))
    inputs = torch.zeros((1, 2), device=sim.device)

    def force_stroke(axial, moment, duration):
        inputs[0, slide] = axial
        inputs[0, turn] = moment
        for _ in range(round(hz * duration)):
            assembly.set_joint_effort_target(inputs)
            assembly.write_data_to_sim()
            sim.step(render=False)
            assembly.update(1 / hz)
        positions = assembly.data.joint_pos
        positions = getattr(positions, "torch", positions)
        return {"advance_m": float(positions[0, slide]), "cap_rad": float(positions[0, turn])}

    rows = {
        "unlocked": force_stroke(0, 0, 0.3),
        "pressed_home": force_stroke(10, 0, 1.0),
        "locked": force_stroke(3, 0.22, 1.0),
        "pull_while_locked": force_stroke(-8, 0, 0.5),
        "unlocked_again": force_stroke(3, -0.22, 0.9),
        "pull_while_unlocked": force_stroke(-8, 0, 0.5),
    }
    print("BENCH_RESULT", json.dumps(rows), flush=True)


if __name__ == "__main__":
    app = AppLauncher(headless=True, device="cpu").app
    try:
        check()
    finally:
        app.close()
