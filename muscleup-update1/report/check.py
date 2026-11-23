'''check.py

   Run the pull-up + typewriter offline (no ROS nodes, no RVIZ) and
   report how well the IK did.

      python3 report/check.py              default shift (0.15 m)
      python3 report/check.py 0.30         try a bigger shift
'''

import os, sys
import numpy as np

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PKG)

from muscleup_update1 import KinematicChain as KC
from muscleup_update1.g1         import *
from muscleup_update1.typewriter import trajectory, segments


# Build the kinematic chains straight from the URDF file (KinematicChain
# normally reads it from /robot_description).
class Quiet():
    def get_logger(self):    return self
    def info(self, *args):   pass
    def error(self, msg):    raise Exception(msg)

def chain(tip):
    html = open(os.path.join(PKG, 'urdf', 'g1_29dof_hands.urdf')).read()
    c = object.__new__(KC.KinematicChain)
    c.node  = Quiet()
    c.chain = KC.parse_HTML(c.node, html, 'pelvis', tip)
    c.dofs  = 0
    for step in c.chain:
        step.dof = None if step.type == KC.JointType.FIXED else c.dofs
        c.dofs  += step.type != KC.JointType.FIXED
    return c


shift = float(sys.argv[1]) if len(sys.argv) > 1 else 0.15
(pd, _, Rd, _) = trajectory(0.0, shift)
g1 = G1({s: chain(PALM[s]) for s in SIDES}, pd, Rd)

# Run two full cycles (rest, pull-up, typewriter reps, lower).
period = sum(T for (_, _, T) in segments(shift))
steps  = round(period / g1.dt)
print("shift %.2fm, hands %.2fm apart, cycle %.1fs" % (shift, 2*WIDTH, period))
print("   t   hand err  pelvis err  max|qdot|  elbow L/R   sigmin L/R")
worst = {'hand': 0, 'pelvis': 0, 'qdot': 0, 'limits': set()}
for k in range(2*steps + 1):
    t = k * g1.dt
    (qdot, e) = g1.step(*trajectory(t, shift))
    hand   = max(np.linalg.norm(e[0:3]), np.linalg.norm(e[5:8]))
    pelvis = np.linalg.norm(e[10:13])
    worst['hand']   = max(worst['hand'],   hand)
    worst['pelvis'] = max(worst['pelvis'], pelvis)
    worst['qdot']   = max(worst['qdot'],   np.abs(qdot[6:]).max())
    worst['limits'] |= {JOINTS[i] for i in np.where((g1.q < QLO) | (g1.q > QHI))[0]}

    # Remember the joints at the end of the first cycle.
    if k == steps:
        q1 = g1.q.copy()

    # Every 1s: the elbow angles and the smallest singular value of
    # each arm's position Jacobian w.r.t. the shoulder + elbow joints.
    # sigmin -> 0 means that arm is straight (singular): it cannot
    # reach any further out.
    if k % 100 == 0:
        sig = [np.linalg.svd(g1.hand(s)[2][0:3, [6 + J[n] for n in ARM[s][0:4]]],
                             compute_uv=False)[-1] for s in SIDES]
        print("%5.1f  %6.1fmm  %7.1fmm  %7.2f   %5.2f %5.2f  %6.3f %6.3f" %
              (t, 1000*hand, 1000*pelvis, np.abs(qdot[6:]).max(),
               g1.q[J['left_elbow_joint']], g1.q[J['right_elbow_joint']], *sig))

print("worst hand error   %.1f mm" % (1000*worst['hand']))
print("worst pelvis error %.1f mm" % (1000*worst['pelvis']))
print("max joint speed    %.2f rad/s" % worst['qdot'])
print("limit violations   %s" % (sorted(worst['limits']) or 'none'))
print("cycle 2 vs cycle 1  %.4f rad (largest joint difference)" % np.abs(g1.q - q1).max())
