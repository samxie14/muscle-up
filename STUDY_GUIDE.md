# G1 Muscle-up, Update 2: Study Guide

How every piece works and fits together, so either of us can walk through any
part of it on the spot. The numbers come from running `report/check2.py` and a
few one-off diagnostics on the update 2 code.

**Contents**

1. [The 60-second pitch](#1-the-60-second-pitch)
2. [The whole pipeline, end to end](#2-the-whole-pipeline-end-to-end)
3. [Frames and coordinates](#3-frames-and-coordinates)
4. [Degrees of freedom](#4-degrees-of-freedom), including which ones are really needed
5. [Forward kinematics: `KinematicChain.fkin`](#5-forward-kinematics-kinematicchainfkin)
6. [From the palm to the grip point](#6-from-the-palm-to-the-grip-point)
7. [The floating base: the 6×35 hand Jacobian](#7-the-floating-base-the-635-hand-jacobian)
8. [The tasks: what we specify and how](#8-the-tasks-what-we-specify-and-how)
9. [The 5 hand constraints, in full detail](#9-the-5-hand-constraints-in-full-detail)
10. [The pseudo-inverse: weighted and damped](#10-the-pseudo-inverse-weighted-and-damped)
11. [Task priority: grip, then pelvis, then posture](#11-task-priority-grip-then-pelvis-then-posture)
12. [One IK step, line by line](#12-one-ik-step-line-by-line)
13. [The secondary task: posture and joint limits](#13-the-secondary-task-posture-and-joint-limits)
14. [The starting pose: Newton–Raphson](#14-the-starting-pose-newtonraphson)
15. [The trajectory](#15-the-trajectory)
16. [The ROS node: one 10 ms tick](#16-the-ros-node-one-10-ms-tick)
17. [Singularities: what happens when the pelvis is out of reach](#17-singularities-what-happens-when-the-pelvis-is-out-of-reach)
18. [Measured results](#18-measured-results)
19. ["Walk us through X": spoken answers](#19-walk-us-through-x-spoken-answers)
20. [Quiz questions and answers](#20-quiz-questions-and-answers)
21. [Symbols, gains, and where they live in the code](#21-symbols-gains-and-where-they-live-in-the-code)

---

## 1. The 60-second pitch

The Unitree G1 humanoid (29 actuated joints, Dex3 three-finger hands) does a
bar muscle-up in RViz:

1. Hang from the bar.
2. Pull up.
3. Transition: the chest goes over the bar and the wrists turn over.
4. Press to straight-arm support.
5. Hold, then lower back down.

It repeats every 12 s.

The pelvis isn't attached to anything, so we model it as a **floating base**:
6 extra virtual DOFs, which brings the total to **35**. We broadcast
`world → pelvis` ourselves.

The IK has **three priority levels**:

1. **Grip (10 rows).** Each hand keeps a point on the bar (3) and keeps its
   grip axis along the bar (2). It can still roll about the bar.
2. **Pelvis (6 rows).** The pelvis follows a spline trajectory.
3. **Posture.** Legs, waist, and wrists go to a nominal pose, and a push keeps
   joints away from their limits. This uses the 19 leftover DOFs.

Every pseudo-inverse is weighted (so the waist is expensive) and damped (so
the IK is stable near singularities).

We tested asking for a pelvis pose that's out of reach. With the grip first,
the hands stay on the bar and the body stops at full stretch. With equal
weights (our update 1), the hands slide off the bar. With no damping, the
arms whip through their joint limits at about 1000 rad/s.

---

## 2. The whole pipeline, end to end

```
 urdf/g1_29dof_hands.urdf
        │  (launch file reads it)
        ▼
 robot_state_publisher ── publishes /robot_description (the URDF text)
        │                                     │
        │                                     ▼
        │                     KinematicChain(node,'pelvis','left_hand_palm_link', CHAIN['left'])
        │                     KinematicChain(node,'pelvis','right_hand_palm_link',CHAIN['right'])
        │                       parse_HTML: walk the URDF from the tip back to the pelvis,
        │                       build a list of steps (fixed / revolute), check the 10 joint names
        │                                     │
        │                                     ▼
        │      ┌──────────── every 10 ms: MuscleUpNode.update() ────────────┐
        │      │ 1. trajectory(t) → pd, vd, Rd, wd, qnom   (muscleup.py)    │
        │      │ 2. G1.step(...)                            (g1.py)         │
        │      │      tasks():  hand('left'), hand('right') → fkin()        │
        │      │                → J1 (10×35), e1;  J2 (6×35), e2           │
        │      │      priority solve → qdot (35)                            │
        │      │      integrate: pelvis p, R and joints q                   │
        │      │ 3. publish TF world→pelvis (p, R)                          │
        │      │    publish /joint_states (29 joints + 14 fixed fingers)    │
        │      │    publish /pose, /twist (desired pelvis), the bar marker  │
        │      └────────────────────────────────────────────────────────────┘
        │                                     │
        ▼                                     ▼
 robot_state_publisher takes /joint_states → TF for every link below the pelvis
        │
        ▼
 RViz: TF tree world → pelvis → … → fingers, plus the bar marker
```

**Who owns what:**

| Piece | Owner |
|---|---|
| `world → pelvis` transform | our node: it's the integrated floating base, `self.g1.p`, `self.g1.R` |
| Every transform below the pelvis | `robot_state_publisher`, computed from the URDF and our `/joint_states` |
| The 14 finger joints | published at fixed angles (`FINGERS`); without them, those links would have no TF and wouldn't draw |

---

## 3. Frames and coordinates

| Frame | Where | Axes |
|---|---|---|
| `world` | floor origin, below the bar | x forward (the way the robot faces), y to the robot's left, z up |
| Bar | `PBAR = (0, 0, 1.6)` | runs along world **y** (`ny()`), radius 1.6 cm |
| Hand spots | `PHAND = PBAR ± 0.22 ŷ` | left at +0.22, right at −0.22 (grip 0.44 m wide) |
| `pelvis` | floating; `self.g1.p`, `self.g1.R` | the base of both kinematic chains |
| `*_hand_palm_link` | the tip of each chain | its local z axis runs across the palm (left: −z, right: +z points along +ŷ) |
| Grip point | palm frame + `GRIP = (0.062, ∓0.038, 0)` | where the bar's center line sits in the hand |

Waypoints are given **relative to the bar**. For example, `HANG` z = −0.73
means the pelvis is 0.73 m below the bar, at world z = 0.87.

---

## 4. Degrees of freedom

### 4.1 The full count

| Group | DOFs | Joints | Appears in |
|---|---|---|---|
| Floating pelvis | 6 | v_x, v_y, v_z, ω_x, ω_y, ω_z | the hand Jacobians + the pelvis task |
| Left leg | 6 | hip pitch, hip roll, hip yaw, knee, ankle pitch, ankle roll | **no task Jacobian** |
| Right leg | 6 | same | **no task Jacobian** |
| Waist | 3 | yaw, roll, pitch | both hand Jacobians (shared) |
| Left arm | 7 | shoulder pitch, roll, yaw, elbow, wrist roll, pitch, yaw | left hand |
| Right arm | 7 | same | right hand |
| **IK total** | **35** | | |
| Fingers | 14 | per hand: index 0/1, middle 0/1, thumb 0/1/2 | not in the IK: fixed angles |

The URDF has 29 + 14 = 43 movable joints. The IK solves for 35.

The order in code is `qdot = [v (3), ω (3), legs (12), waist (3), left arm (7), right arm (7)]`.
The joint order is `JOINTS` in `g1.py`, line 42, and `J[name]` gives a joint's index.

### 4.2 What we specify, and what's left over

| | Count |
|---|---|
| Grip rows (5 per hand × 2) | 10 |
| Pelvis rows | 6 |
| **Specified (primary)** | **16** |
| Left over for posture | 35 − 16 = **19** |

Measured, with the pseudo-inverses as built: the null-space projector `N1` of the grip has
trace 25.0 (35 − 10), and the projector `N12` of all 16 rows has trace 19.3
(about 19; damping makes it slightly more than 19).

### 4.3 Which DOFs actually matter? (the "that's a lot to solve for" question)

**Short answer:** the 12 leg DOFs aren't needed in the IK. The other 23 are.

**Legs (12): not needed.** We verified this numerically:

- The 12 leg columns of the 16×35 task Jacobian are exactly zero. A hip or
  knee can't move a hand or the pelvis frame.
- So the weighted, damped pseudo-inverse has zero rows for the legs, and the
  null-space projector is exactly the identity on the leg block (error 0.0).
- So the IK's leg velocity is exactly `lams·(qnom − q)`, the posture term,
  to machine precision.
- That means the legs are **decoupled**. Including them costs some matrix
  size but changes nothing. We could equally set `q_legs` directly from the
  trajectory and solve a 23-DOF problem with the same result.
- We keep them in the vector so the code has one uniform 35-DOF state. The
  answer to "how many DOFs matter" is **23**.

**Pelvis (6): needed.** It's the floating base. The hands are fixed to the
bar, so the whole body moves *because* the pelvis moves. It's also what lets
the priority scheme work: when the pelvis target is unreachable, the pelvis
DOFs are allowed to differ from the target. If we hard-set the pelvis pose,
the grip would have to fail instead.

**Waist (3): earns its keep.** We tested locking it by zeroing its columns in
the grip Jacobian:

| | Waist free | Waist locked |
|---|---|---|
| Muscle-up | no limits violated | both shoulder rolls past their limits (0.012 rad) |
| Too far sideways (0.40 m) | pelvis 69 mm short, 3.0 rad/s, 1 limit (0.009 rad) | pelvis 158 mm short, 4.5 rad/s, elbow + shoulder limits (0.158 rad) |

**Arms (7 each): needed.** Each hand has 5 constraints, so each arm has
7 − 5 = 2 spare DOFs. One is the **elbow swivel** (the elbow circling the
shoulder–wrist line). The other is **roll about the bar**. The posture and
joint-limit tasks use that spare room.

**Redundancy, counted only over the DOFs that matter:** 23 − 16 = 7. That's
3 (waist) + 2 (left arm) + 2 (right arm), once the pelvis task has pinned the
6 pelvis DOFs. The 19 total is those 7 plus the 12 decoupled legs.

---

## 5. Forward kinematics: `KinematicChain.fkin`

`KinematicChain.py` is our HW5 code, unchanged.

**Building the chain (`parse_HTML`).** It starts at the tip frame (the palm)
and repeatedly finds the one URDF joint whose child is the current frame,
until it reaches the pelvis. Every child has exactly one parent, so the path
through the tree is unique. For each joint it stores:

- `Tshift`, the fixed origin transform from the URDF `<origin xyz rpy>`;
- the type (fixed or revolute);
- `nlocal`, the axis in the local frame.

Active joints are numbered 0–9. It then checks that the names match
`CHAIN[s] = WAIST + ARM[s]`.

**`fkin(q)`, for 10 joint angles:**

1. Start with `T = I`, which is the pelvis frame.
2. For each step:
   - fixed: `T = T · Tshift`;
   - revolute: `T = T · Tshift · Rot(nlocal, q_i)`.

   After each active joint, record `p_i` (the joint's position) and
   `n_i = R · nlocal` (its axis), both in the pelvis frame.
3. The tip is `p_tip`, `R_tip`.
4. Each Jacobian column, for revolute joint i:

   ```
   Jv[:, i] = n_i × (p_tip − p_i)      linear velocity of the tip from spinning joint i
   Jw[:, i] = n_i                      angular velocity of the tip
   ```

It returns `(p, R, Jv, Jw)`, **all in the pelvis frame**. Each Jacobian is 3×10.

---

## 6. From the palm to the grip point

The bar doesn't pass through the palm-frame origin. It sits against the palm
at the base of the fingers. In the palm frame that's `GRIP[s]`; expressed in
the pelvis frame it's `r = R · GRIP[s]`.

```
p_grip = p_palm + r
v_grip = v_palm + ω × r = Jv q̇ + (Jw q̇) × r = Jv q̇ − r × (Jw q̇) = (Jv − [r]× Jw) q̇
```

So `Jv_grip = Jv − crossmat(r) @ Jw`. This is `g1.py`, lines 168–170. The
angular Jacobian doesn't change, because the whole hand rotates together.

The finger angles were chosen so each finger segment is tangent to a 1.6 cm
bar centered at the grip point. They're published as constants and never
solved for.

---

## 7. The floating base: the 6×35 hand Jacobian

The chain's numbers are in the pelvis frame. We need world coordinates, and we
need the pelvis motion columns. The world position is

```
p_world = p_pelvis + R_pelvis · p_local
```

Differentiate it, with the pelvis moving at (v, ω) and the joints at q̇:

```
ṗ_world = v + ω × (p_world − p_pelvis) + R_pelvis · Jv q̇
ω_world = ω + R_pelvis · Jw q̇
```

Written as a matrix, using `a × b = −[b]× a`:

```
              pelvis v   pelvis ω               this chain's 10 joints     other 19 joints
J_hand =  [     I      −[p − p_pelvis]×          R_pelvis · Jv_grip            0          ]   3 rows (linear)
          [     0             I                  R_pelvis · Jw                 0          ]   3 rows (angular)
```

This is `G1.hand()`, `g1.py` lines 164–178. Each hand Jacobian has 16 non-zero
columns: 6 pelvis + 3 waist + 7 arm.

**This is also why the grip Jacobian `J1` is never singular.** The pelvis
columns alone can translate and rotate both hands in any direction. At the
hang, the singular values of `J1` (10×35) run from 3.11 down to 0.177: full
rank, far from 0.

---

## 8. The tasks: what we specify and how

`G1.tasks()` (`g1.py` lines 180–193) returns two separate tasks:

| | Rows | Jacobian | Error | Desired |
|---|---|---|---|---|
| **Task 1: grip** | 10 | `J1` = stack of `[Jh_L[0:3]; PERP@Jh_L[3:6]; Jh_R[0:3]; PERP@Jh_R[3:6]]` | `e1` = `[ep(PHAND_L, p_L); PERP@(a_L × ŷ); …right…]` | the hands fixed on the bar, so the desired velocity is 0 |
| **Task 2: pelvis** | 6 | `J2 = [I₆ 0]` (`np.eye(6, 35)`) | `e2 = [ep(pd, p_pelvis); eR(Rd, R_pelvis)]` | the spline `pd(t)`, `Rd(t) = Roty(pitch(t))` |

**Error functions, from the course `TransformHelpers`:**

- `ep(pd, p) = pd − p`, the position error.
- `eR(Rd, R) = ½ Σᵢ (R[:,i] × Rd[:,i])`, the orientation error as a rotation
  vector. For a small error it's the axis times the angle that turns R onto Rd.

**Why the pelvis Jacobian is just `[I 0]`:** the pelvis's own velocity *is*
the first 6 entries of q̇. Nothing else moves the pelvis frame.

**Why feedback makes the error decay.** Each task solves `J q̇ = ẋ_d + λ e`.
The error is `e = x_d − x`, so

```
ė = ẋ_d − ẋ = ẋ_d − J q̇ = −λ e      →      e(t) = e(0) · exp(−λ t)
```

With λ = 20 /s the time constant is 50 ms. It wipes out the drift from
integrating with a finite 10 ms step.

---

## 9. The 5 hand constraints, in full detail

### 9.1 What they are

| # | Constraint | Error component | Jacobian row |
|---|---|---|---|
| 1 | grip point x = 0 (on the bar's line in x) | `ep(PHAND, p)[0]` | `Jh[0]`: v_x |
| 2 | grip point y = ±0.22 (the hand's spot along the bar) | `ep(PHAND, p)[1]` | `Jh[1]`: v_y |
| 3 | grip point z = 1.6 (bar height) | `ep(PHAND, p)[2]` | `Jh[2]`: v_z |
| 4 | the grip axis doesn't tilt about world x | `(a × ŷ)_x` | `Jh[3]`: ω_x |
| 5 | the grip axis doesn't tilt about world z | `(a × ŷ)_z` | `Jh[5]`: ω_z |
| free | **roll about the bar (world y)** | — | `Jh[4]`: ω_y is **dropped** |

### 9.2 The code

```python
GRIPAXIS = {'left': -nz(), 'right': nz()}   # palm-frame axis that lies along the bar
PERP     = np.vstack((nx(), nz()))          # 2x3 = [[1,0,0],
                                            #        [0,0,1]]   keeps x and z rows

for s in SIDES:
    (p, R, Jh) = self.hand(s)                       # grip point, hand R, 6x35 Jacobian
    rows += [Jh[0:3],                               # constraints 1-3: linear rows
             PERP @ Jh[3:6]]                        # constraints 4-5: omega_x, omega_z rows
    errs += [ep(PHAND[s], p),                       # 3 position errors
             PERP @ cross(R @ GRIPAXIS[s], ny())]   # 2 alignment errors
```

### 9.3 Why `a × ŷ` is the right orientation error

- `a = R · GRIPAXIS[s]` is the hand's across-the-palm axis, in world. We want
  `a = ŷ`.
- For two unit vectors, `a × ŷ` points along the axis you'd rotate about to
  turn `a` onto `ŷ`. Its length is the sine of the angle between them. For
  small angles it's exactly the rotation vector, the same kind of quantity as
  `eR`, so it pairs correctly with ω in `J q̇ = λ e`.
- Rotating the hand at ω changes `a` at `ω × a`. Only the parts of ω
  perpendicular to `a ≈ ŷ` change `a`, which are ω_x and ω_z. Spinning about
  ŷ leaves `a` unchanged. That's the roll freedom.
- `a × ŷ` is perpendicular to ŷ, so its y component is always 0. Keeping it
  would add a row that's always zero, so `PERP` drops it.

### 9.4 What `PERP` does, twice

1. `PERP @ cross(...)` keeps the 2 meaningful error components.
2. `PERP @ Jh[3:6]` keeps the ω_x and ω_z rows of the hand's angular
   Jacobian. Because the ω_y row isn't in the task, roll about the bar falls
   in the **null space**. The IK is free to roll the hand, and it does.

### 9.5 Why leave roll free

- A real hand rolls around a bar.
- In a muscle-up the wrists **must** turn over the top of the bar during the
  transition. We measured the left hand rolling from −102° to +84° about the
  bar in one rep, about **187°** in total.
- Fixing all 6 would mean writing a hand-roll trajectory by hand that matches
  whatever the arms naturally do. Leaving it free lets the IK find the roll.

### 9.6 Subtleties

- **Why the sign of `GRIPAXIS` differs.** The two palm frames are mirror
  images, so −z for the left and +z for the right both end up along +ŷ.
- **Two solutions.** `a × ŷ = 0` also holds when `a = −ŷ` (the hand flipped
  around). The Newton–Raphson initial guess puts us on the correct branch.
  The flipped solution is unstable under the feedback anyway.
- **Not a closed-form constraint.** It's a velocity-level task with feedback.
  The hand drifts by at most 1.7 mm in the muscle-up, and feedback pulls it
  back.

---

## 10. The pseudo-inverse: weighted and damped

```python
def pinv(Jt, W, gamma):           # g1.py line 116
    JW = Jt * W                   # scale column j by W[j]
    return (W[:, None] * JW.T) @ np.linalg.inv(JW @ JW.T + gamma**2 * np.eye(len(Jt)))
```

```
J# = W (JW)ᵀ [ (JW)(JW)ᵀ + γ² I ]⁻¹          (W = diag of the weights)
```

**What it minimizes.** `q̇ = J# ẋ` solves

```
minimize   ‖ J q̇ − ẋ ‖²  +  γ² ‖ W⁻¹ q̇ ‖²
```

To see this, substitute `u = W⁻¹ q̇`. The problem becomes the plain damped
least-squares problem `‖(JW) u − ẋ‖² + γ²‖u‖²`, whose solution is
`u = (JW)ᵀ[(JW)(JW)ᵀ + γ²I]⁻¹ ẋ`. Then `q̇ = W u`.

**Weights.** `W[waist] = 0.15` and every other weight is 1. The cost per unit
speed is `(q̇ᵢ/Wᵢ)²`, so the waist costs `1/0.15² ≈ 44×` as much as an arm
joint. The solver bends the arms, not the waist. **W weights the columns,
which are DOFs. It doesn't weight the tasks.**

**Damping, using the SVD `J = U Σ Vᵀ` (ignore W for clarity).** Each gain 1/σ becomes

```
σ / (σ² + γ²)    ≈ 1/σ  when σ ≫ γ     (healthy directions: almost exact)
                 = 1/(2γ) at σ = γ     (the largest gain possible)
                 ≈ σ/γ² → 0  as σ → 0  (singular directions: the solver gives up there)
```

| | γ | Max gain 1/(2γ) |
|---|---|---|
| Grip | 0.02 | 25 |
| Pelvis | 0.05 | 10 |

**The trade-off.** Damping always costs a little accuracy, because the
solution isn't exact. It buys bounded joint speeds near singularities.
Small γ tracks well but is fragile; large γ is robust but sloppy.

---

## 11. Task priority: grip, then pelvis, then posture

### 11.1 Derivation

Every solution of the grip task can be written as

```
q̇ = J1# ẋ1 + N1 z,        N1 = I − J1# J1      (N1 projects onto motions that don't move the grip)
```

for any z. Choose z to do the pelvis task as well as possible:

```
J2 q̇ = ẋ2   →   J2 N1 z = ẋ2 − J2 J1# ẋ1   →   z = (J2 N1)# (ẋ2 − J2 J1# ẋ1)
```

So

```
q̇ = J1# ẋ1 + N1 (J2 N1)# (ẋ2 − J2 J1# ẋ1)  +  (I − J12# J12) q̇_sec
```

Reading the pelvis term from right to left:

1. `ẋ2 − J2 J1# ẋ1` is how much pelvis motion is still needed after the grip
   step already moved the pelvis.
2. `(J2 N1)#` finds the smallest motion that achieves it, using only
   grip-safe directions. `J2 N1` is "what the pelvis does if you move only
   along grip-safe directions".
3. The leading `N1` makes sure that motion stays grip-safe. In exact
   arithmetic `(J2 N1)#` already lies in the null space. With the waist
   weights and damping it leaks slightly, so we project again. Without the
   leading `N1`, the hands drift 6–10 mm when the pelvis is out of reach; with
   it, 0.3 mm.

### 11.2 Equal weights vs priority: the key idea

**Update 1 (equal weights):** one solve, `q̇ = J12# [ẋ1; ẋ2]`. Least squares
minimizes the *sum* of all 16 squared residuals.

**When all 16 rows can be met** (the normal muscle-up), both methods give the
same q̇: every residual is 0, so there's nothing to trade. They differ only
when tasks **conflict**.

**When they conflict:**

- Least squares trades the residuals off against each other, so the pelvis
  falls short *and* the hands come off the bar.
- Priority gives every bit of the shortfall to the pelvis.

**Why the grip goes first:** a hand off the bar is physically impossible; the
robot would fall. A pelvis a few cm off its spline is just a slightly
different muscle-up.

### 11.3 Level 3

The posture term goes through `I − J12# J12`, the null space of all 16 rows.
It can only use the ~19 leftover DOFs, so it never disturbs the grip or the
pelvis.

---

## 12. One IK step, line by line

`G1.step(pd, vd, Rd, wd, qnom)`, `g1.py` lines 201–235:

```python
lam = lam or self.lam                                  # 20, or 1/dt during Newton–Raphson
(J1, e1, J2, e2) = self.tasks(pd, Rd)                  # fkin both arms, build 10 + 6 rows
xr1 = lam * e1                                         # grip: bar still, so feedback only
xr2 = np.concatenate((vd, wd)) + lam * self.cap(e2)    # pelvis: feedforward + capped feedback
Jall = np.vstack((J1, J2))                             # 16x35

if self.priority:
    J1inv = pinv(J1, self.W, self.gamma)               # 35x10, gamma = 0.02
    N1    = np.eye(35) - J1inv @ J1                    # 35x35 grip null-space projector
    qdot  = J1inv @ xr1                                # level 1
    qdot += N1 @ pinv(J2 @ N1, self.W, self.gamma2) @ (xr2 - J2 @ qdot)   # level 2, gamma2 = 0.05
else:
    qdot = pinv(Jall, self.W, self.gamma) @ np.concatenate((xr1, xr2))   # update 1

push = np.maximum(QLO + 0.15 - self.q, 0) - np.maximum(self.q - QHI + 0.15, 0)
qsec = [0]*6 ++ ( lams * wpost * (qnom - q) + lamlim * push )            # 35
qdot += (I - pinv(Jall) @ Jall) @ qsec                 # level 3

p += dt * v                                            # pelvis position
R  = Rotn(w/|w|, |w| dt) @ R                           # exact rotation about w, stays orthonormal
q += dt * qdot[6:]                                     # the 29 joints
```

**Sizes:**

| Quantity | Size |
|---|---|
| `J1` | 10×35 |
| `J2` | 6×35 |
| `J1inv` | 35×10 |
| `N1` | 35×35, rank about 25 |
| `J2 @ N1` | 6×35 |
| `qdot` | 35 |

**`cap(e2)`** (line 195) limits the pelvis position error used for
*feedback* to `emax` = 0.10 m. An impossible target then asks for at most
`λ · 0.10` = 2 m/s of pelvis speed. Without the cap the request grows with
the distance to the target. Below 10 cm, which covers the whole normal
muscle-up, it changes nothing. The reported error stays the true one.

**Why the pelvis rotation update has that form.** ω is constant over the
10 ms step, so rotating by angle `|ω|·dt` about the axis `ω/|ω|` is the exact
integral. Adding `dt·[ω]×R` instead would slowly un-orthonormalize R.

---

## 13. The secondary task: posture and joint limits

**Posture.** `lams · wpost · (qnom − q)` with `lams = 5`. `wpost` is 1 for
the legs, waist, and wrists, and 0 for the shoulders and elbows, which are left
entirely to the tasks. `qnom = posture(hip, knee)` from the trajectory:

- both hips pitch to `hip` and both knees to `knee`;
- ankle pitch = −0.3 · knee;
- everything else to 0.

The legs pike during the dip (hip −0.70, knee 0.40) and straighten in support.

**Joint-limit push.** Any joint within 0.15 rad of a limit gets
`lamlim · (distance into that 0.15 band)`, with `lamlim = 50`, pushing it
back toward the middle.

**Its limits.** Both terms live at level 3, in the null space of the 16 task
rows. If a primary task needs a joint to go past its limit, the push can't
stop it. We saw this for the far shoulder roll when the pelvis slides more
than about 0.25 m sideways: it goes 0.007–0.025 rad over. Turning the push
off makes that 4.5 rad, so it's doing real work. It just can't override
level 1 or 2.

**The tightest spot in the muscle-up:** the elbows fold to −1.038 rad in the
dip, against a −1.047 limit. The push keeps them inside.

---

## 14. The starting pose: Newton–Raphson

In the `G1.__init__` constructor (lines 152–162):

1. Put the pelvis exactly at `HANG`.
2. Set the legs to `qnom`, and the arms straight up: shoulder pitch −2.8,
   shoulder roll ±0.2, elbow 1.2.
3. Run `step(...)` 200 times with **λ = 1/dt** and zero velocity.

With λ = 1/dt, each step moves `dt · q̇ = dt · J#(e/dt) = J# e`. That's
exactly the Newton–Raphson update `Δq = J# e`. Run 200 times, it converges
the hands onto the bar. The pelvis error stays 0, because the pelvis starts
on target.

**Why the guess matters.** IK is local. Newton–Raphson converges to the
solution nearest the guess, and that "branch" (for example which way the
elbows bend) is carried through the whole motion. Update 1 showed that a
1 cm change in the hang height could switch branches.

---

## 15. The trajectory

`muscleup.py`, lines 43–94. Only the pelvis gets a trajectory. A point is
`(x, y, z, pitch, hip, knee)`:

| Waypoint | x | y | z (rel. bar) | pitch | hip | knee | Body |
|---|---|---|---|---|---|---|---|
| `HANG` | −0.03 | 0 | −0.73 | 0.00 | −0.10 | 0.10 | hanging, elbows ~0.37 rad |
| `PULL` | −0.13 | 0 | −0.44 | −0.10 | −0.35 | 0.20 | top of the pull, chin near the bar |
| `DIP` | −0.25 | 0 | −0.15 | +0.40 | −0.70 | 0.40 | chest over the bar, leaning forward, legs piked |
| `SUPPORT` | −0.12 | 0 | +0.12 | +0.12 | −0.15 | 0.05 | straight-arm support on top |

**Knots:** `knots(motion, target)` lists `(waypoint, arrival time)` pairs.

| Motion | Knots (times in s) |
|---|---|
| muscle-up | HANG 0 → HANG 1 → PULL 3 → DIP 4.5 → SUPPORT 6 → SUPPORT 7.5 → DIP 9 → PULL 10.5 → HANG 12 |
| `low` | HANG 0 → HANG 1 → LOW 3 → LOW 6 → HANG 8, where LOW is HANG with z = target |
| `side` | HANG 0 → HANG 1 → PULL 3 → SIDE 5 → SIDE 8 → PULL 10 → HANG 12, where SIDE is PULL with y = target |

**Splines.** Each segment is a cubic Hermite spline, `spline(t, T, p0, pf, v0, vf)`:

```
p(t) = a + b t + c t² + d t³,   with   p(0) = p0, ṗ(0) = v0, p(T) = pf, ṗ(T) = vf
```

**Knot velocities:**

- **Interior knot:** the Catmull–Rom average, `(P[k+1] − P[k−1]) / (T[k+1] − T[k−1])`.
  The body flows through PULL and DIP without stopping.
- **A knot that starts or ends a hold** (the same waypoint twice in a row),
  and the first and last knot: velocity 0, a real stop.
- **Continuity:** position and velocity are continuous. Acceleration jumps at
  the knots (it's a cubic). A quintic would also make acceleration continuous.

**What `trajectory(t)` returns:**

| Output | Value |
|---|---|
| `pd` | `PBAR + (x, y, z)` |
| `vd` | `(ẋ, ẏ, ż)` |
| `Rd` | `Roty(pitch)`; positive pitch leans forward |
| `wd` | `ŷ · pitch_dot`, the rotation rate about the y axis |
| `qnom` | `posture(hip, knee)` |

`t % T_last` loops it. Each motion ends where it starts, so cycle 2 repeats
cycle 1 exactly: we measured a 0.0000 rad joint difference.

---

## 16. The ROS node: one 10 ms tick

`MuscleUpNode` (`muscleup.py`, lines 97–160).

**Startup:**

1. Read the parameters `motion` (`muscleup`, `low`, or `side`), `target`, and
   `priority`.
2. Build the two chains. This reads `/robot_description` from
   `robot_state_publisher`.
3. Build `G1`. This runs Newton–Raphson onto the bar.
4. Create the publishers and the TF broadcaster.
5. Wait until something subscribes to `/joint_states`
   (`robot_state_publisher`).
6. Start a 100 Hz timer. `t` starts at −dt, so the first tick is t = 0.

**Each tick (`update`):**

1. `t += dt`
2. `(pd, vd, Rd, wd, qnom) = trajectory(t, motion, target)`
3. `(qdot, e) = g1.step(...)`
4. Publish:

| Output | Content |
|---|---|
| TF `world → pelvis` | `Transform_from_Rp(g1.R, g1.p)`: the floating base |
| `/joint_states` | 29 joints `g1.q` plus 14 finger constants, with velocities |
| `/pose`, `/twist` | the **desired** pelvis pose and velocity, for comparison and plotting |
| `/visualization_marker_array` | the bar, as a cylinder rotated by Rotx(π/2) so it lies along y |

**Offline (`report/check2.py`).** It builds the same chains straight from the
URDF file by calling `parse_HTML` (no ROS), runs two cycles, and prints the
worst values. Those are the hand error, pelvis error, joint speed, elbow
angle, arm σ_min, any limit violations, and cycle-to-cycle drift.

---

## 17. Singularities: what happens when the pelvis is out of reach

### 17.1 The singularity, physically

An arm is singular when it's **straight**. For the G1 the elbow joint is
0 rad when bent 90° and about 1.5 rad when straight; it can go further, to
2.09, but that's bending backwards. At straight, no joint motion moves the
hand further from the shoulder. The 3×4 shoulder + elbow position Jacobian
loses rank, and its σ_min goes to 0.

### 17.2 The singularity, in our equations

The hands are locked to the bar (level 1). The question is whether a
grip-safe motion can still move the pelvis, so we look at the singular values
of `J2 N1` (6×35):

| Pose | σ of `J2 N1` (largest → smallest) |
|---|---|
| Hang | 1.00, 0.94, 0.89, 0.57, 0.11, 0.052 |
| Dip | 1.00, 0.93, 0.88, 0.26, 0.20, 0.154 |
| Support | 1.00, 0.94, 0.87, 0.38, 0.16, 0.095 |
| **Arms straight** (target z = −1.0) | 1.00, 0.95, 0.88, 0.59, 0.048, **0.006** |

With straight arms the smallest singular direction is **(0.035, 0, 0.999, …)**,
almost pure pelvis **z**. The pelvis can't go any lower without the hands
leaving the bar.

Damped gain in that direction: `0.006/(0.006² + 0.05²) ≈ 2.4`. The undamped
gain would be `1/0.006 ≈ 167`.

Note also that at the hang the smallest σ (0.052) is right at γ₂ = 0.05.
That's why γ₂ costs a few mm of pelvis lag (5.8 mm vs 2.1 mm) even in the
normal motion.

### 17.3 What each method does (measured)

**Pelvis too low (`low`).** The arms are fully straight at z ≈ −0.78 to −0.79.
γ = 0 still tracks at −0.78 and fails at −0.80. Each cell is the hand error
and the max joint speed.

| Target z | Grip-first (update 2) | Equal weights (update 1) | No damping (γ = 0) |
|---|---|---|---|
| −0.78 | 0.3 mm / 0.7 rad/s | 1.0 mm / 0.8 rad/s | 0.1 mm / 0.9 rad/s |
| −0.80 | 0.3 mm / 1.1 rad/s | 6.8 mm / 1.3 rad/s | 137 mm / 165 rad/s |
| −0.85 | 0.3 mm / 1.6 rad/s | 23 mm / 1.8 rad/s | 771 mm / 572 rad/s |
| −1.00 | 0.3 mm / 2.1 rad/s | 126 mm / 59 rad/s | 853 mm / 905 rad/s |
| −1.50 | 0.4 mm / 3.4 rad/s | 238 mm / 91 rad/s | 1610 mm / 1670 rad/s |
| −3.00 | 0.7 mm / 5.5 rad/s | — | — |

**Too far left or right (`side`, at the top of the pull).** The far arm is
straight at |y| ≈ 0.30–0.35, and +y and −y are mirror images to 0.1 mm. Each
cell is the hand error, pelvis error, and max joint speed.

| Target \|y\| | Grip-first | Equal weights | No damping |
|---|---|---|---|
| 0.30 | 2.3 mm / 15 mm / 2.1 rad/s | 5.2 mm / 5 mm / 2.1 rad/s | 0.2 mm / 0.2 mm / 2.2 rad/s |
| 0.40 | 6.2 mm / 69 mm / 3.0 rad/s | 34 mm / 34 mm / 3.3 rad/s | 86 mm / 0.2 mm / 55 rad/s |
| 0.60 | 10 mm / 256 mm / 4.6 rad/s | 180 mm / 115 mm / 68 rad/s | 1077 mm / 0.2 mm / 734 rad/s |
| 1.00 | 12 mm / 650 mm / 6.9 rad/s | — | — |

**How to explain the three columns:**

1. **Grip-first.** The robot hangs at full stretch with its hands locked on
   the bar. The pelvis stops at the edge of reach, so its error is just the
   unreachable distance (220 mm for a −1.00 target). The elbows stop at
   1.41–1.43 rad, just short of straight. Joint speeds stay low.
2. **Equal weights.** Least squares splits the conflict, so the hands slide
   off the bar. Past about 20 cm out of reach the joint speeds jump (59 rad/s)
   and shoulders go past their limits.
3. **No damping.** 1/σ → ∞. Each step `dt·q̇` is enormous and lands far
   outside where the linearization holds, so the next step is worse still.
   The elbows reach 4–11 rad (limit 2.09), which is bending backwards. The
   hands end up 0.8–1.6 m from the bar. The pelvis error is a perfect 0.2 mm,
   because the pelvis rows are the easy part of the Jacobian. This is the
   "arms ripping out of their sockets" picture. **Priority alone doesn't save
   you without damping.**
4. **Sideways, grip-first's hand error is larger (up to 12 mm) than when
   too low (0.3 mm).** Near the edge of reach the feedforward velocity is
   badly approximated by the linearization, and the far shoulder is pushing
   against its limit.

### 17.4 What each update 2 design choice buys (measured with it removed)

| Change | Without it |
|---|---|
| Grip before pelvis | hands 23–238 mm off the bar (z = −0.85 to −1.5) |
| `N1 @` in front of `(J2 N1)#` | 6–10 mm grip leak (vs 0.3 mm) at z = −0.85 to −1.0 |
| γ₂ = 0.05 for the pelvis (vs 0.02) | 88 rad/s, elbows bent backwards to 1.76 rad at z = −1.0 |
| Pelvis error cap 0.10 m | 75 rad/s, 126 mm hand error at z = −1.5 |
| Joint-limit push (`lamlim = 50`) | joints 4.5 rad past their limits at y = 0.40 |

---

## 18. Measured results

The muscle-up, from `python3 report/check2.py`:

| Metric | Value |
|---|---|
| Worst grip-point error | 1.7 mm |
| Worst pelvis error | 5.8 mm (2.1 mm with equal weights, which have less damping) |
| Fastest joint | 1.49 rad/s |
| Joint-limit violations | none |
| Closest to a limit | elbows −1.038 vs −1.047 (in the dip) |
| Hand roll about the bar | about 187° per rep |
| Cycle 2 vs cycle 1 | 0.0000 rad |

What isn't modeled: forces, torques, dynamics (can 25 N·m shoulders lift the
G1?), collisions with the bar, and finger contact.

---

## 19. "Walk us through X": spoken answers

Each of these is about a minute out loud. Say the bold sentence first.

**"Walk us through your DOFs."**
**35 in the IK: 6 for the floating pelvis and 29 joints.** The 29 are 12 legs,
3 waist, and 7 per arm; the 14 finger joints are fixed angles, not in the IK.
We specify 16 rows (5 per hand plus 6 for the pelvis), so there are 19 spare.
The legs appear in no task Jacobian, so they're decoupled and only follow the
posture task. The DOFs that matter are 23 (pelvis, waist, arms), so the real
redundancy is 7: 3 in the waist and 2 per arm (elbow swivel and roll about
the bar).

**"Walk us through how one hand is constrained."**
**3 position constraints and 2 orientation constraints; roll about the bar is
free.** We get the palm pose and Jacobian from our HW5 chain, shift to the grip
point with `Jv − [r]× Jw`, and add the pelvis columns to get a 6×35 world
Jacobian. The first three rows and `PHAND − p` give the position task. For
orientation we take the hand axis `a` that should lie along the bar. `a × ŷ` is
the small rotation that would align it. We keep only its x and z components
with `PERP`, and the matching ω_x and ω_z Jacobian rows. Dropping the ω_y row
is what leaves the roll free. The hands roll about 187° during a rep.

**"Walk us through one IK step."**
**Grip first, then the pelvis in what's left, then posture in what's left of
that.**

1. Build the 10 grip rows and the 6 pelvis rows.
2. The references are λe for the grip, and the spline velocity plus capped λe
   for the pelvis.
3. Solve the grip with a weighted, damped pseudo-inverse.
4. Compute the grip's null-space projector N1.
5. Solve for the pelvis motion the grip step hasn't already delivered, using
   `(J2 N1)#`, and project it with N1 again.
6. Add the posture and joint-limit push through the null space of all 16
   rows.
7. Integrate the pelvis position, the pelvis rotation (an exact rotation about
   ω), and the joints.

**"Walk us through what happens at a singularity."**
**When the pelvis target is too low, the arms straighten and the IK can no
longer lower the pelvis without moving the hands.** In our terms `J2 N1` loses
rank: its smallest σ drops to 0.006, in the pelvis-z direction. Damping turns
1/σ into σ/(σ² + γ²), so the gain in that direction goes to 0 instead of
infinity. The grip is level 1, so the hands stay on the bar at 0.3 mm. The
pelvis just stops at z ≈ −0.78, and the elbows stop at 1.42, short of straight.
With our old equal weights the hands slide 13–24 cm off the bar. With no
damping, joints hit about 900 rad/s and the elbows bend backwards.

**"Walk us through the trajectory."**
**Only the pelvis is planned: position, pitch, and a leg posture, through four
waypoints.** The waypoints are hang, pull, dip, and support, over a 12 s
loop. The splines are cubic Hermite, with Catmull–Rom velocities at the
interior knots and zero velocity at the holds. The trajectory hands the IK
pd, vd, Rd = Roty(pitch), wd = ŷ·pitch_dot, and qnom. The spline velocities
are feedforward; pd and Rd give the feedback error.

**"Why weighted, and why damped?"**
**Weighted decides which joints do the work. Damped keeps the speeds finite
near singularities.** We weight the waist at 0.15, so it costs about 44×
more and the arms do the pulling. Damping adds γ²I before the inverse, which
caps the gain at 1/(2γ). Grip γ = 0.02: the grip Jacobian always has full rank,
because the pelvis columns are in it. Pelvis γ₂ = 0.05: that's where the
singularity lives.

**"How does it start?"**
**Newton–Raphson onto the bar.** It's the same step with λ = 1/dt, so
Δq = J# e, run 200 times from an arms-up guess. The guess picks the solution
branch.

---

## 20. Quiz questions and answers

**DOFs**

1. *How many DOFs?* 35 = 6 floating + 29 actuated (12 legs, 3 waist, 14 arms).
   The fingers (14) aren't in the IK.
2. *How many do you specify?* 16 (10 grip + 6 pelvis). 19 are left over.
3. *Are they all needed?* The 12 legs aren't. They're decoupled: zero columns
   in every task, so the IK just runs their posture term. The 23 that matter
   are the pelvis, waist, and arms. Locking the waist made the shoulder rolls
   hit their limits.
4. *Why does the pelvis need DOFs if it isn't actuated?* Because it moves.
   The hands are fixed, so moving the pelvis is how the body moves. We
   integrate it and broadcast its TF.
5. *Redundancy per arm?* 7 − 5 = 2: elbow swivel, and roll about the bar.
6. *What's in each chain?* pelvis → 3 waist → 3 shoulder → elbow → 3 wrist →
   palm, which is 10 joints. The waist is shared by both chains.

**The hand constraints**

7. *Name the 5.* Grip point x, y, z, plus no tilt of the grip axis about world
   x or world z.
8. *How did you code the 2 orientation rows?* The error is
   `PERP @ cross(R @ GRIPAXIS, ŷ)` and the rows are `PERP @ Jh[3:6]`, with
   `PERP = [[1,0,0],[0,0,1]]`.
9. *Why does `a × ŷ` have only 2 components?* It's perpendicular to ŷ, so
   its y component is always 0.
10. *Why not constrain all 6?* The hand has to roll about 187° over the bar
    during the turnover.
11. *Why the `GRIP` offset?* The bar sits at the base of the fingers, not at
    the palm-frame origin. We use `p + R·GRIP` and `Jv − [r]× Jw`.

**Tasks and priority**

12. *Do the tasks have equal weight?* No: grip > pelvis > posture. Update 1
    was equal weight. They give the same answer when everything is reachable
    and differ only when the tasks conflict.
13. *Write the formula.* `q̇ = J1#ẋ1 + N1(J2N1)#(ẋ2 − J2J1#ẋ1) + (I − J12#J12)q̇_sec`.
14. *What does N1 mean physically?* The motions that leave both grips
    unchanged: moving the body while the hands stay on the bar.
15. *Are the joint weights task weights?* No. They're on the columns (DOFs)
    and decide which joints move. Priority is on the rows (tasks).
16. *Why can a joint still exceed a limit?* The push is level 3. It can't
    override motion that a primary task demands.

**Singularities and damping**

17. *What's the singularity?* A straight arm. `J2 N1` loses rank in pelvis z
    (σ goes to 0.006).
18. *What happens when the target is too low?* The hands stay at 0.3 mm. The
    pelvis stops at −0.78 and the elbows at 1.42, with speeds of 2 rad/s or
    less.
19. *Damping in SVD terms?* 1/σ becomes σ/(σ² + γ²). The max gain is
    1/(2γ), and the gain goes to 0 as σ → 0.
20. *Why two γ?* The grip Jacobian is never singular, because it includes
    the pelvis columns. The pelvis level is where the singularity is.
21. *Why the error cap?* λe grows with the distance to an impossible target.
    The cap limits the request to 2 m/s.
22. *What does damping cost?* Accuracy: the pelvis lags 5.8 mm vs 2.1 mm,
    and 15 mm at 0.30 m sideways.

**Mechanics**

23. *λ?* 20 /s. The error decays as `e^(−λt)`, a 50 ms time constant.
24. *Feedforward vs feedback?* Feedforward is the spline velocity (zero for
    the grip, since the bar is still). Feedback is `λe`.
25. *Why `Rotn` for the pelvis?* It's the exact rotation about a constant ω,
    so R stays orthonormal.
26. *How is the start found?* Newton–Raphson: λ = 1/dt gives Δq = J# e.
27. *Why Catmull–Rom?* So the motion flows through the interior knots. The
    holds get zero velocity.
28. *Who publishes the link transforms?* We publish `world → pelvis`;
    `robot_state_publisher` turns `/joint_states` into everything below it.
29. *What's missing from the model?* Dynamics and torques, collisions, and
    finger contact.

---

## 21. Symbols, gains, and where they live in the code

| Symbol | Value | Meaning | Code |
|---|---|---|---|
| q̇ | 35 | `[v, ω, 29 joints]` | `qdot` |
| J1, e1 | 10×35, 10 | grip task | `tasks()` → `Jgrip, egrip` |
| J2, e2 | 6×35, 6 | pelvis task `[I 0]` | `tasks()` → `Jpel, epel` |
| N1 | 35×35 | grip null-space projector | `step()`, line 215 |
| λ | 20 /s | task feedback gain | `self.lam` |
| λ_s | 5 /s | posture gain | `self.lams` |
| λ_lim | 50 /s | joint-limit push gain (0.15 rad band) | `self.lamlim` |
| γ | 0.02 | grip damping | `self.gamma` |
| γ₂ | 0.05 | pelvis damping | `self.gamma2` |
| e_max | 0.10 m | pelvis feedback error cap | `self.emax` |
| W | 1; waist 0.15 | joint weights (columns) | `self.W` |
| dt | 0.01 s | step / timer period | `self.dt` |
| `GRIP` | (0.062, ∓0.038, 0) | palm → grip point | `g1.py`, line 89 |
| `GRIPAXIS` | −z (left), +z (right) | palm axis along the bar | line 90 |
| `PERP` | `[[1,0,0],[0,0,1]]` | keeps the x, z rows | line 91 |
| `PHAND` | bar ± 0.22 ŷ | hand spots | line 82 |
