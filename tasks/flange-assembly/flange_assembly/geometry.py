"""Geometry checks for horizontal flange assembly, in metres.

Object positions are bounding-box centres; axes derive from XYZW quaternions.
Both assemblies include real pipe; face offsets follow their470/203mm axial envelopes and real branch offset.
"""
import numpy as np
from flange_assembly.specs import MOVING_BBOX,HOLE_XY,TARGET_HOLES

HALF_DEPTH = .1015
HOLE_RADIUS = .0425
AXIS_COS = np.cos(np.deg2rad(7.25))
ANGLES=np.arange(32)*(2*np.pi/32)
SHAFT_XY=.006*np.column_stack((np.cos(ANGLES),np.sin(ANGLES)))
BORE_NORMALS=np.column_stack((np.cos(ANGLES+np.pi/32),np.sin(ANGLES+np.pi/32)))
BORE_BOUND=.007*np.cos(np.pi/32)
FIT_TOLERANCE=.00005


def rotation(q):
    x, y, z, w = np.asarray(q, dtype=float)
    norm = np.linalg.norm([x, y, z, w])
    if not np.isfinite(norm) or norm < 1e-12:
        raise ValueError('invalid quaternion')
    x, y, z, w = np.array([x, y, z, w]) / norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def holes(obj, half_depth=HALF_DEPTH, local_y_offset=0.):
    r = rotation(obj.quat_xyzw)
    face = np.asarray(obj.pos) - half_depth*r[:, 2] - local_y_offset*r[:, 1]
    return [face + r @ np.array([x, y, 0.]) for x, y in
            HOLE_XY]


def _evaluate(fixed, loose, pins, head_gap_limit):
    """Final state only; .4 alignment plus .3 per distinct seated pin.

    Fixed local +z points along +x, loose local +z along -x, pin +z
    towards -x (head side). Holes are matched in three dimensions. Both plate bores must clear the bolt axis; the head is within3mm of the loose back face and the tip projects2mm beyond the fixed plate.
    """
    f, l = rotation(fixed.quat_xyzw), rotation(loose.quat_xyzw)
    if f[0, 2] < AXIS_COS or -l[0, 2] < AXIS_COS:
        return 0.0
    fh, lh = holes(fixed), holes(loose, MOVING_BBOX[2], MOVING_BBOX[1])
    # Hole centres account for both axial mating and bolt-circle alignment.
    if any(min(np.linalg.norm(a-b) for b in lh) >= .003 for a in fh):
        return 0.0
    claimed=set()
    for pin in pins:
        pin_R=rotation(pin.quat_xyzw);axis=pin_R[:,2]
        if -axis[0]<AXIS_COS:continue
        position=np.asarray(pin.pos)
        for j in TARGET_HOLES:
            if j in claimed:continue
            k=int(np.argmin([np.linalg.norm(fh[j]-h) for h in lh]))
            # A12mm bolt must clear both14mm holes across the full14mm plates.
            planes=((fh[j],f),(fh[j]+.014*f[:,2],f),
                    (lh[k],l),(lh[k]+.014*l[:,2],l))
            fits=True
            # Exact32-sided shaft section in each actual32-sided bore.
            # The50um tolerance is for the contact solver, not extra clearance.
            radial_points=position+SHAFT_XY@pin_R[:,:2].T
            for centre,frame in planes:
                normal=frame[:,2];denom=float(np.dot(axis,normal))
                if abs(denom)<.9:fits=False;break
                parameters=((centre-radial_points)@normal)/denom
                section=radial_points+parameters[:,None]*axis
                xy=(section-centre)@frame[:,:2]
                if np.max(xy@BORE_NORMALS.T-BORE_BOUND)>FIT_TOLERANCE:fits=False;break
            if not fits:continue
            head=position+.02125*axis
            tip=position-.02875*axis
            head_gap=float(np.dot(head-(lh[k]+.014*l[:,2]),l[:,2]))
            tip_projection=float(np.dot(tip-(fh[j]+.014*f[:,2]),f[:,2]))
            if -.0005<=head_gap<=head_gap_limit and tip_projection>=.002:claimed.add(j);break
    return min(1.,.4+.3*len(claimed))


def score(fixed,loose,pins):
    """Official final rubric; heads must be within3mm of the back face."""
    return _evaluate(fixed,loose,pins,.003)


def count_engaged_bolts(fixed,loose,pins):
    """Controller safety only: count through-shafts before head seating.

    Full bore fit and2mm tip projection remain required. This never changes
    the official task score, which still requires the3mm head gap.
    """
    value=_evaluate(fixed,loose,pins,.020)
    return max(0,int(round((value-.4)/.3)))
