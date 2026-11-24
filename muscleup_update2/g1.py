'''g1.py   (update 2)

   The G1 on the bar: joint names/limits, the bar and grip, and the
   whole-body inverse kinematics.  No ROS here (only the kinematic
   chains), so report/check2.py can run it offline.

   DOFs.  The pelvis floats, so its velocity (v, w) is 6 extra
   (unactuated) DOFs in front of the 29 joints:

      qdot = [v_pelvis (3), w_pelvis (3), qdot_joints (29)]      35

   Tasks, in strict priority order (new in update 2):
      1. grip    each hand: grip point on the bar (3)        10 rows
                            grip axis along the bar (2)
      2. pelvis  position (3) + orientation (3)               6 rows
      3. posture legs/waist/wrists + joint-limit push        (whatever
                 null space is left: 35 - 16 = 19 DOFs)

   The grip can never be traded for the pelvis: if the pelvis target
   is out of reach, the hands stay on the bar and the pelvis stops
   short.  (Update 1 stacked all 16 rows with equal weight, so the
   hands slid off the bar as much as the pelvis fell short.)
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

# Each hand's chain runs pelvis -> waist (3) -> arm (7) -> palm.
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
# The bar runs along the world y axis.  Muscle-up grip: hands 0.44 m
# apart (update 1's typewriter used a wide 0.72 m grip).
BAR_HEIGHT = 1.6
BAR_RADIUS = 0.016
WIDTH      = 0.22
PBAR       = pxyz(0.0, 0.0, BAR_HEIGHT)
PHAND      = {'left':  PBAR + WIDTH * ny(),
              'right': PBAR - WIDTH * ny()}

# The grip point (on the bar axis) and the axis along the bar, in each
# palm frame.  The grip fixes the point (3) and the two rotations
# perpendicular to the bar (2), so the hand can still roll about the
# bar, which it must do to get over the bar in a muscle-up.
GRIP     = {'left':  pxyz(0.062, -0.038, 0.0), 'right': pxyz(0.062, 0.038, 0.0)}
GRIPAXIS = {'left':  -nz(),                    'right': nz()}
PERP     = np.vstack((nx(), nz()))     # keep x,z rows: drop rotation about y

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


def posture(hip, knee):
    '''Nominal posture for the secondary task: legs together at the
    given hip pitch/knee, everything else at zero.'''
    qnom = np.zeros(len(JOINTS))
    for s in SIDES:
        qnom[J[f'{s}_hip_pitch_joint']]   = hip
        qnom[J[f'{s}_knee_joint']]        = knee
        qnom[J[f'{s}_ankle_pitch_joint']] = -0.3 * knee
    return qnom


def pinv(Jt, W, gamma):
    '''Weighted, damped pseudo-inverse (HW6 P1 + P2):
          J# = W (J W)^T [ (J W)(J W)^T + gamma^2 I ]^-1'''
    JW = Jt * W
    return (W[:, None] * JW.T) @ np.linalg.inv(JW @ JW.T + gamma**2 * np.eye(len(Jt)))


#
#   Whole-body Inverse Kinematics
#
class G1():
    def __init__(self, chains, pelvis, Rpelvis, qnom, priority=True, dt=0.01):
        # chains[side] is a KinematicChain from 'pelvis' to that palm.
        self.chains   = chains
        self.idx      = {s: [J[n] for n in CHAIN[s]] for s in SIDES}
        self.priority = priority          # False = update 1 (equal weight)
        self.dt       = dt

        # Gains: task convergence, posture, joint limits, damping.
        self.lam    = 20.0
        self.lams   = 5.0
        self.lamlim = 50.0
        self.gamma  = 0.02      # grip (and the equal-weight stack)
        self.gamma2 = 0.05      # pelvis: more damping, so an unreachable
                                # pelvis target can't whip the arms
        self.emax   = 0.10      # pelvis feedback error cap (m)

        # Joint weights: a small weight makes the waist "expensive", so
        # the arms do the pulling.
        self.W = np.ones(6 + len(JOINTS))
        self.W[[6 + J[n] for n in WAIST]] = 0.15

        # The posture task only pulls on the legs, waist, and wrists.
        self.wpost = np.array([0.0 if ('shoulder' in n or 'elbow' in n)
                               else 1.0 for n in JOINTS])

        # Start: arms up, then Newton-Raphson (lam = 1/dt) onto the bar
        # with the pelvis held at the given pose.
        self.p = pelvis.copy()
        self.R = Rpelvis.copy()
        self.q = qnom.copy()
        for (s, m) in (('left', 1), ('right', -1)):
            self.q[J[f'{s}_shoulder_pitch_joint']] = -2.8
            self.q[J[f'{s}_shoulder_roll_joint']]  =  0.2 * m
            self.q[J[f'{s}_elbow_joint']]          =  1.2
        for _ in range(200):
            self.step(pelvis, vzero(), Rpelvis, vzero(), qnom, lam=1/dt)

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
        '''The grip task (10 rows) and the pelvis task (6 rows):
        Jacobians and errors.'''
        (rows, errs) = ([], [])
        for s in SIDES:
            (p, R, Jh) = self.hand(s)
            rows += [Jh[0:3], PERP @ Jh[3:6]]                 # 3 + 2 rows
            errs += [ep(PHAND[s], p),                         # point on bar
                     PERP @ cross(R @ GRIPAXIS[s], ny())]     # axis || bar
        Jgrip = np.vstack(rows)
        egrip = np.concatenate(errs)
        Jpel  = np.eye(6, 6 + len(JOINTS))                    # [I6  0]
        epel  = np.concatenate((ep(pd, self.p), eR(Rd, self.R)))
        return (Jgrip, egrip, Jpel, epel)

    def cap(self, e2):
        '''Cap the pelvis position error used for feedback at emax, so a
        pelvis target far out of reach asks for at most lam*emax.'''
        n = np.linalg.norm(e2[0:3])
        return e2 if n <= self.emax else np.concatenate((e2[0:3] * self.emax/n, e2[3:6]))

    def step(self, pd, vd, Rd, wd, qnom, lam=None):
        '''One IK step toward the desired pelvis (pd, vd, Rd, wd) and
        posture qnom.  Returns qdot (35) and the errors (10 grip + 6).'''
        lam = lam or self.lam
        (J1, e1, J2, e2) = self.tasks(pd, Rd)
        xr1 = lam * e1                                  # the bar is still
        xr2 = np.concatenate((vd, wd)) + lam * self.cap(e2)   # ffwd + fbk
        Jall = np.vstack((J1, J2))

        if self.priority:
            # 1. Grip.  2. Pelvis, only using motions that leave the grip
            # alone (the null space N1 of the grip), and only asking for
            # what step 1 didn't already do.
            J1inv = pinv(J1, self.W, self.gamma)
            N1    = np.eye(len(self.W)) - J1inv @ J1
            qdot  = J1inv @ xr1
            qdot += N1 @ pinv(J2 @ N1, self.W, self.gamma2) @ (xr2 - J2 @ qdot)
        else:
            # Update 1: all 16 rows in one least-squares problem.
            qdot = pinv(Jall, self.W, self.gamma) @ np.concatenate((xr1, xr2))

        # 3. Posture and a push away from within 0.15 rad of any joint
        # limit, in the null space of all 16 task rows.
        push = np.maximum(QLO + 0.15 - self.q, 0) - np.maximum(self.q - QHI + 0.15, 0)
        qsec = np.concatenate((np.zeros(6), self.lams * self.wpost * (qnom - self.q)
                                            + self.lamlim * push))
        qdot += (np.eye(len(qsec)) - pinv(Jall, self.W, self.gamma) @ Jall) @ qsec

        # Integrate the pelvis (v, w) and the joints.
        (v, w) = (qdot[0:3], qdot[3:6])
        self.p = self.p + self.dt * v
        if np.linalg.norm(w) > 1e-9:
            self.R = Rotn(w / np.linalg.norm(w), np.linalg.norm(w) * self.dt) @ self.R
        self.q = self.q + self.dt * qdot[6:]
        return (qdot, np.concatenate((e1, e2)))
