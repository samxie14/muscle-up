'''g1.py

   The G1 on the bar: joint names/limits, the bar and grip, and the
   whole-body inverse kinematics.  No ROS here (only the kinematic
   chains), so report/check.py can run it offline.

   The pelvis floats, so its velocity (v, w) is 6 extra DOFs in front
   of the 29 joints:   qdot = [v_pelvis (3), w_pelvis (3), qdot_joints (29)]

   Primary tasks (16 of 35 DOFs):
      each hand: grip point on the bar (3) + grip axis along the bar (2)
      pelvis:    position (3) + orientation (3)
   Secondary task (null space): leg/waist/wrist posture + joint limits.
'''

import numpy as np

from utils.TransformHelpers     import *


#
#   Joints
#
SIDES  = ('left', 'right')
WAIST  = ['waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint']
ARM    = {s: [f'{s}_shoulder_pitch_joint', f'{s}_shoulder_roll_joint',
              f'{s}_shoulder_yaw_joint',   f'{s}_elbow_joint',
              f'{s}_wrist_roll_joint',     f'{s}_wrist_pitch_joint',
              f'{s}_wrist_yaw_joint'] for s in SIDES}
LEG    = {s: [f'{s}_hip_pitch_joint',  f'{s}_hip_roll_joint',
              f'{s}_hip_yaw_joint',    f'{s}_knee_joint',
              f'{s}_ankle_pitch_joint', f'{s}_ankle_roll_joint'] for s in SIDES}
JOINTS = LEG['left'] + LEG['right'] + WAIST + ARM['left'] + ARM['right']
J      = {name: i for (i, name) in enumerate(JOINTS)}

# Each hand's chain runs pelvis -> waist -> arm -> palm.
CHAIN  = {s: WAIST + ARM[s] for s in SIDES}
PALM   = {s: f'{s}_hand_palm_link' for s in SIDES}

# Joint limits (from the URDF).  The right side mirrors the left.
LIMITS = {'waist_yaw_joint':   (-2.618, 2.618),
          'waist_roll_joint':  (-0.52,  0.52),
          'waist_pitch_joint': (-0.52,  0.52)}
for (s, m) in (('left', 1), ('right', -1)):
    mirror = lambda lo, hi: (lo, hi) if m > 0 else (-hi, -lo)
    LIMITS.update({
        f'{s}_hip_pitch_joint':      (-2.5307, 2.8798),
        f'{s}_hip_roll_joint':       mirror(-0.5236, 2.9671),
        f'{s}_hip_yaw_joint':        (-2.7576, 2.7576),
        f'{s}_knee_joint':           (-0.0873, 2.8798),
        f'{s}_ankle_pitch_joint':    (-0.8727, 0.5236),
        f'{s}_ankle_roll_joint':     (-0.2618, 0.2618),
        f'{s}_shoulder_pitch_joint': (-3.0892, 2.6704),
        f'{s}_shoulder_roll_joint':  mirror(-1.5882, 2.2515),
        f'{s}_shoulder_yaw_joint':   (-2.618, 2.618),
        f'{s}_elbow_joint':          (-1.0472, 2.0944),
        f'{s}_wrist_roll_joint':     (-1.9722, 1.9722),
        f'{s}_wrist_pitch_joint':    (-1.6144, 1.6144),
        f'{s}_wrist_yaw_joint':      (-1.6144, 1.6144)})
QLO = np.array([LIMITS[n][0] for n in JOINTS])
QHI = np.array([LIMITS[n][1] for n in JOINTS])


#
#   The Bar and Grip
#
# The bar runs along the world y axis, the hands hold it wide apart.
BAR_HEIGHT = 1.6
BAR_RADIUS = 0.016
WIDTH      = 0.36
PBAR       = pxyz(0.0, 0.0, BAR_HEIGHT)
PHAND      = {'left':  PBAR + WIDTH * ny(),
              'right': PBAR - WIDTH * ny()}

# The grip point (on the bar axis) and the axis along the bar, in each
# palm frame.  The grip fixes the point and the two rotations
# perpendicular to the bar, so the hand can still roll about the bar.
GRIP     = {'left':  pxyz(0.062, -0.038, 0.0), 'right': pxyz(0.062, 0.038, 0.0)}
GRIPAXIS = {'left':  -nz(),                    'right': nz()}
PERP     = np.vstack((nx(), nz()))

# Dex3 finger angles, chosen so each segment wraps tangent to the bar.
FINGERS = {}
for (s, m) in (('left', 1), ('right', -1)):
    FINGERS.update({f'{s}_hand_index_0_joint':  -1.260 * m,
                    f'{s}_hand_index_1_joint':  -0.893 * m,
                    f'{s}_hand_middle_0_joint': -1.260 * m,
                    f'{s}_hand_middle_1_joint': -0.893 * m,
                    f'{s}_hand_thumb_0_joint':   0.0,
                    f'{s}_hand_thumb_1_joint':   0.444 * m,
                    f'{s}_hand_thumb_2_joint':   0.852 * m})

# Nominal posture for the secondary task: legs together, slightly bent.
QNOM = np.zeros(len(JOINTS))
for s in SIDES:
    QNOM[J[f'{s}_hip_pitch_joint']]   = -0.2
    QNOM[J[f'{s}_knee_joint']]        =  0.3
    QNOM[J[f'{s}_ankle_pitch_joint']] = -0.1


#
#   Whole-body Inverse Kinematics
#
class G1():
    def __init__(self, chains, pelvis, Rpelvis, dt=0.01):
        # chains[side] is a KinematicChain from 'pelvis' to that palm.
        self.chains = chains
        self.idx    = {s: [J[n] for n in CHAIN[s]] for s in SIDES}
        self.dt     = dt

        # Gains: primary convergence, posture, joint limits, damping.
        self.lam    = 20.0
        self.lams   = 5.0
        self.lamlim = 50.0
        self.gamma  = 0.02

        # Weighted pseudo-inverse (HW6 P1): a small weight makes the
        # waist "expensive", so the arms do the pulling.
        self.W = np.ones(6 + len(JOINTS))
        self.W[[6 + J[n] for n in WAIST]] = 0.15

        # The posture task only pulls on the legs, waist, and wrists.
        self.wpost = np.array([0.0 if ('shoulder' in n or 'elbow' in n)
                               else 1.0 for n in JOINTS])

        # Start: arms up and out in a V, then Newton-Raphson (lam = 1/dt)
        # onto the bar with the pelvis held at the given pose.
        self.p = pelvis.copy()
        self.R = Rpelvis.copy()
        self.q = QNOM.copy()
        for (s, m) in (('left', 1), ('right', -1)):
            self.q[J[f'{s}_shoulder_pitch_joint']] = -2.8
            self.q[J[f'{s}_shoulder_roll_joint']]  =  1.3 * m
            self.q[J[f'{s}_elbow_joint']]          =  1.2
        for _ in range(200):
            self.step(pelvis, vzero(), Rpelvis, vzero(), lam=1/dt)

    def hand(self, s):
        '''Grip point position, hand orientation, and the 6x35 world
        Jacobian, including the floating pelvis columns.'''
        (p, R, Jv, Jw) = self.chains[s].fkin(self.q[self.idx[s]])
        r  = R @ GRIP[s]                      # palm -> grip point
        p  = self.p + self.R @ (p + r)
        Jv = self.R @ (Jv - crossmat(r) @ Jw)
        Jw = self.R @ Jw

        Jh = np.zeros((6, 6 + len(JOINTS)))
        Jh[0:3, 0:3] = Reye()                 # pelvis translation
        Jh[0:3, 3:6] = -crossmat(p - self.p)  # pelvis rotation
        Jh[3:6, 3:6] = Reye()
        Jh[:, [6 + i for i in self.idx[s]]] = np.vstack((Jv, Jw))
        return (p, self.R @ R, Jh)

    def tasks(self, pd, Rd):
        '''Stack the 16 primary tasks: Jacobian and error.'''
        (rows, errs) = ([], [])
        for s in SIDES:
            (p, R, Jh) = self.hand(s)
            rows += [Jh[0:3], PERP @ Jh[3:6]]
            errs += [ep(PHAND[s], p), PERP @ cross(R @ GRIPAXIS[s], ny())]
        rows.append(np.eye(6, 6 + len(JOINTS)))
        errs.append(np.concatenate((ep(pd, self.p), eR(Rd, self.R))))
        return (np.vstack(rows), np.concatenate(errs))

    def step(self, pd, vd, Rd, wd, lam=None):
        '''One IK step toward the desired pelvis (pd, vd, Rd, wd).
        Returns qdot (35) and the task error (16).'''
        (Jt, e) = self.tasks(pd, Rd)
        xrdot = np.concatenate((np.zeros(10), vd, wd)) + (lam or self.lam) * e

        # Weighted, damped pseudo-inverse (HW6 P1 and P2).
        JW   = Jt * self.W
        Jinv = (self.W[:, None] * JW.T) @ np.linalg.inv(
            JW @ JW.T + self.gamma**2 * np.eye(len(e)))

        # Secondary: posture, plus a push away from within 0.15 rad of
        # any joint limit, both projected into the null space.
        push = np.maximum(QLO + 0.15 - self.q, 0) - np.maximum(self.q - QHI + 0.15, 0)
        qsec = np.concatenate((np.zeros(6), self.lams * self.wpost * (QNOM - self.q)
                                            + self.lamlim * push))
        qdot = Jinv @ xrdot + (np.eye(len(qsec)) - Jinv @ Jt) @ qsec

        # Integrate the pelvis (v, w) and the joints.
        (v, w) = (qdot[0:3], qdot[3:6])
        self.p = self.p + self.dt * v
        if np.linalg.norm(w) > 1e-9:
            self.R = Rotn(w / np.linalg.norm(w), np.linalg.norm(w) * self.dt) @ self.R
        self.q = self.q + self.dt * qdot[6:]
        return (qdot, e)
