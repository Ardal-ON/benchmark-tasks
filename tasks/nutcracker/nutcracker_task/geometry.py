"""Conservative visible-envelope measurements, independent of Isaac imports."""
import csv
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]

def rim_opening(a,ra,b,rb):
    normal=ra@np.array([0.,1.,0.])+rb@np.array([0.,1.,0.])
    normal/=max(np.linalg.norm(normal),1.e-12)
    return max(0.,max(float(np.dot((a+ra@np.array([x,-.008,z]))-
        (b+rb@np.array([x,.008,z])),normal))
        for x,z in ((.016,0),(-.016,0),(0,.016),(0,-.016))))

def fixture_stable(poses,rotations,origin,yaw):
    pivot=np.array([0.,.020,0.]);target=origin+yaw@pivot
    return (np.linalg.norm(poses['upper']-origin)<.010 and
        np.linalg.norm(poses['lower']+rotations['lower']@pivot-target)<.010)

def table_bounds_ok(positions):
    # Radius 26 mm encloses each half about its own body origin at any rotation.
    # Table: centre (.36,0), 1.10 x 1.30 m. The radius makes bounds conservative.
    return all(-.19+.026 <= p[0] <= .91-.026 and
        -.65+.026 <= p[1] <= .65-.026 and
        .7475-.005 <= p[2] <= .7475+.220-.026 for p in positions)

def supported(support_ages,pair_age,whole_shell=False):
    direct=[age<=24 for age in support_ages]  # 50 ms contact tolerance at 480 Hz
    return all(direct) or (any(direct) and (whole_shell or (pair_age<=24 and
        min(abs(pair_age-age) for age in support_ages if age<=24)<=1)))

def envelopes():
    rows=[]
    with (ROOT/'fabrication-envelope.csv').open() as f:
        for row in csv.DictReader(f):
            rows.append((row['body'],np.array([float(row['center_'+x+'_mm']) for x in 'xyz'])/1000,
                np.array([float(row['size_'+x+'_mm']) for x in 'xyz'])/2000))
    # Both shell half outer boxes include the visually attached kernel patch.
    rows.extend((name,np.array([0.,sign*.001,0.]),np.array([.018,.010,.018]))
        for name,sign in (('shell_a',1),('shell_b',-1)))
    return rows

def grippers_clear(hands,poses,rotations,boxes):
    # 150 mm sphere encloses YAM gripper body and both complete finger meshes
    # about TCP across the full prismatic range. Measured maximum URDF visual
    # vertex radius 147.284 mm (fingers <=101.379 mm). No mesh sampling at run time.
    clearance=float('inf')
    for hand in hands.values():
        if -float(np.sum(hand['fingers'])) < .070:return False,0.
        for body,center,half in boxes:
            local=rotations[body].T@(hand['tcp']-poses[body])-center
            distance=float(np.linalg.norm(np.maximum(np.abs(local)-half,0.)))-.150
            clearance=min(clearance,distance)
    return clearance>=.080,clearance
