# Simulation fidelity and limits

The figure is an original dimensioned fabrication, with an aluminum load path
and hollow PETG exterior specified in REAL_TRIAL.md. It has not been built or
load-qualified. Rigid external envelopes and assigned masses replace internal
construction; their inertia and friction have not been measured on a physical
assembly. Static/dynamic friction coefficients are 1.2/1.0, provisional values
that require checking during physical commissioning.

The walnut is two rigid halves with an attached visual kernel. It has no shell
flexure, biological fracture network or independently separable kernel. A
native fixed joint with provisional 200 N / 5 N m limits generates rupture;
these are joint reaction limits, not measured external jaw compression or a
calibrated walnut strength. Published walnut compression results vary strongly
with orientation and moisture; they do not qualify this particular model.
See [Saeidi et al.](https://doi.org/10.1111/j.1745-4549.2011.00601.x) and
[Xu et al.](https://dl.ifip.org/db/conf/ifip12/ccta2011-2/XuYB11.pdf).

Shell-to-shell contact is filtered while the seam exists, approximating an
intact body. After native rupture the broken joint and filter are removed at
the first completed physics-step boundary. Ordinary fragment contact resumes
one physics tick later in the recorded audit. No shell pose, velocity, force
or impulse is overwritten at fracture. The 480 Hz trace records pair contacts,
separation, positions and jaw impulse-derived contact totals around rupture.
Those totals are diagnostic; they do not establish measured normal forces.

The score uses ordered opposing-jaw compression, normal rim opening, continuous
mouth custody and exclusion of robot/foreign-object shell contact. Native seam
rupture alone cannot earn credit. A 3 mm opening is a rigid-shell proxy for a
visible crack, not a measurement of biological damage. Simulated support uses
actual contact plus a stable-pose envelope over the final two seconds; it is
an operational resting-state check, not a reconstructed force balance.

The successful local demonstration uses normal robot joint commands, with
simulator feedback for scripted motion planning. Object pose writes occur only
at reset. Fixture-only bench diagnostics use a motor and pose placement and
are not robot demonstrations. The fixed oblique witness camera records the
same live demonstration trajectory; model inputs remain the standard three
policy cameras. Matching run results and limitations are recorded in README.md.
