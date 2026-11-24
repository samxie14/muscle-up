'''check2.py   (update 2)

   Run a motion offline (no ROS nodes, no RVIZ) and report how the IK did.

      python3 report/check2.py                      the muscle-up
      python3 report/check2.py low  -1.0            pelvis asked to go too low
      python3 report/check2.py side  0.40           too far left (- = right)
      python3 report/check2.py low  -1.0 --equal    update 1's equal weights
      python3 report/check2.py low  -1.0 --gamma 0  no damping (grip and pelvis)
'''

import os, sys
import numpy as np

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PKG)

from muscleup_update1 import KinematicChain as KC
from muscleup_update2.g1       import *
from muscleup_update2.muscleup import trajectory, knots


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


def run(motion='muscleup', target=0.0, priority=True, gamma=None, quiet=False):
    '''Run two cycles; return the worst values.'''
    traj = lambda t: trajectory(t, motion, target)
    (pd, _, Rd, _, qnom) = traj(0.0)
    g1 = G1({s: chain(PALM[s]) for s in SIDES}, pd, Rd, qnom, priority=priority)
    if gamma is not None:                       # override both dampings
        (g1.gamma, g1.gamma2) = (gamma, gamma)
    period = knots(motion, target)[-1][1]
    steps  = round(period / g1.dt)

    if not quiet:
        print("%s, target %.2f, %s, gamma %.3f/%.3f, cycle %.1fs" % (motion, target,
              'grip first' if priority else 'equal weights', g1.gamma, g1.gamma2, period))
        print("   t   hand err  pelvis err  max|qdot|  elbow L/R   sigmin L/R")
    w = {'hand': 0, 'pelvis': 0, 'qdot': 0, 'elbow': -9, 'sigma': 9, 'limits': set()}
    for k in range(2*steps + 1):
        t = k * g1.dt
        (pd, vd, Rd, wd, qnom) = traj(t)
        (qdot, e) = g1.step(pd, vd, Rd, wd, qnom)
        hand   = max(np.linalg.norm(e[0:3]), np.linalg.norm(e[5:8]))
        pelvis = np.linalg.norm(e[10:13])
        elbows = (g1.q[J['left_elbow_joint']], g1.q[J['right_elbow_joint']])
        # Smallest singular value of each arm's position Jacobian w.r.t.
        # the shoulder + elbow joints: -> 0 when that arm is straight.
        sig = [np.linalg.svd(g1.hand(s)[2][0:3, [6 + J[n] for n in ARM[s][0:4]]],
                             compute_uv=False)[-1] for s in SIDES]
        w['hand']   = max(w['hand'],   hand)
        w['pelvis'] = max(w['pelvis'], pelvis)
        w['qdot']   = max(w['qdot'],   np.abs(qdot[6:]).max())
        w['elbow']  = max(w['elbow'],  *elbows)
        w['sigma']  = min(w['sigma'],  *sig)
        w['limits'] |= {JOINTS[i] for i in np.where((g1.q < QLO) | (g1.q > QHI))[0]}
        w['over']    = max(w.get('over', 0), np.maximum(QLO - g1.q, g1.q - QHI).max())
        if k == steps:
            q1 = g1.q.copy()
        if not quiet and k % 100 == 0:
            print("%5.1f  %6.1fmm  %7.1fmm  %7.2f   %5.2f %5.2f  %6.3f %6.3f" %
                  (t, 1000*hand, 1000*pelvis, np.abs(qdot[6:]).max(), *elbows, *sig))
    w['repeat'] = np.abs(g1.q - q1).max()

    if not quiet:
        print("worst hand error    %.1f mm" % (1000*w['hand']))
        print("worst pelvis error  %.1f mm" % (1000*w['pelvis']))
        print("max joint speed     %.2f rad/s" % w['qdot'])
        print("max elbow angle     %.2f rad (straight ~1.5)" % w['elbow'])
        print("min arm sigma       %.3f" % w['sigma'])
        print("limit violations    %s" % (sorted(w['limits']) or 'none'))
        print("worst overshoot     %.3f rad past a limit" % max(w['over'], 0))
        print("cycle 2 vs cycle 1  %.4f rad (largest joint difference)" % w['repeat'])
    return w


if __name__ == "__main__":
    args   = [a for a in sys.argv[1:] if not a.startswith('--')]
    gamma  = float(sys.argv[sys.argv.index('--gamma') + 1]) if '--gamma' in sys.argv else None
    if '--gamma' in sys.argv:
        args.remove(sys.argv[sys.argv.index('--gamma') + 1])
    run(args[0] if args else 'muscleup', float(args[1]) if len(args) > 1 else 0.0,
        priority='--equal' not in sys.argv, gamma=gamma)
