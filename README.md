# ME 133a Project, Update 1: G1 Pull-up + Typewriter

The Unitree G1 hangs from a bar with a wide grip, does a pull-up, then
"typewriters": at the top of the pull-up it slides its body toward the
left hand (left arm bends, right arm extends), then across to the right
hand (right arm bends, left arm extends). After 3 typewriter reps it returns
to the center, lowers back into the hang, rests, and starts again. It repeats
until you stop it.

One cycle is 27 s:

| Time in cycle (s) | Phase       | Pelvis motion                                      |
|-------------------|-------------|----------------------------------------------------|
| 0 – 2             | Rest        | Held at `HANG`                                     |
| 2 – 4             | Pull-up     | `HANG` → `UP` (rises 0.22 m)                       |
| 4 – 5.5           | Shift left  | `UP` → `LEFT` (moves `shift` toward the left hand) |
| 5.5 – 23.5        | Typewriter  | 3 reps of `LEFT` → `RIGHT` → `LEFT`, 3 s per side (6 s per rep) |
| 23.5 – 25         | Center      | `LEFT` → `UP`                                      |
| 25 – 27           | Lower       | `UP` → `HANG`, then the next cycle starts with the rest |

The hands are 0.72 m apart (±0.36 m). With the default `shift` of 0.15 m,
the pelvis travels 0.30 m from side to side.

### Are these the G1's default hands?

**No.** The course's default G1 model (`g1_description/urdf/g1_29dof.urdf`,
the same as `robots/g1_29dof.urdf`) has no fingers. Each wrist ends in a
fixed, rounded `left_rubber_hand` / `right_rubber_hand` link, a one-piece
"mitten" with no joints.

This project uses `urdf/g1_29dof_hands.urdf` instead. It's Unitree's G1 with
the optional **Dex3-1 three-finger hands** (thumb, index, middle; 7 joints
per hand), taken from the unitreerobotics/unitree_ros GitHub repo. Its mesh
paths point at `package://g1_description/meshes`, and the course's
`g1_description` already includes all the Dex3 meshes, so nothing extra needs
installing. From the pelvis to the wrists, the body is identical to the
course's `g1_29dof.urdf`, with the same 29 joints and the same limits.

How the hands are used:
- **The IK chains end at `left_hand_palm_link` / `right_hand_palm_link`**,
  and the grip point is offset from the palm frame by `GRIP` (section 3.2).
- **The 14 finger joints are not part of the IK.** They're published at fixed
  angles (`FINGERS` in `g1.py`), chosen so each finger segment wraps tangent
  to the bar. The fingers look closed around the bar, but they don't move or
  carry load.
- **With the default rubber hands** you'd have to end the chains at
  `*_rubber_hand` and pick a new grip offset. There'd be no fingers to show
  wrapping the bar.

---

## 1. Running it

The ROS package is named **`muscleup_update1`**. ROS package names can't
contain `-`, so the folder name `muscleup-update1` doesn't matter to colcon.
It can sit next to the original `muscleup` package; just don't launch both at
the same time, because they both publish `/joint_states`.

**Mac Terminal**: zip the folder and copy it into the VM:
```bash
cd ~/Downloads/me133a_project
zip -rq muscleup-update1.zip muscleup-update1 -x '*.DS_Store' '*__pycache__*'
multipass list
multipass transfer muscleup-update1.zip jazzy:/home/ubuntu/
```

**Ubuntu Terminal** (`multipass shell jazzy`): extract and build:
```bash
cd ~/robotws/src
unzip -oq ~/muscleup-update1.zip
ls ~/robotws/src/muscleup-update1          # package.xml, setup.py, launch/, ...
cd ~/robotws
colcon build --symlink-install --packages-select muscleup_update1
source ~/robotws/install/setup.bash
ros2 pkg prefix muscleup_update1           # /home/ubuntu/robotws/install/muscleup_update1
```
This assumes `utils` and `g1_description` are already built in `~/robotws`
(you did that for the original `muscleup` package).

**Ubuntu desktop terminal** (over RDP, because RViz needs a display): launch:
```bash
source ~/robotws/install/setup.bash
ros2 launch muscleup_update1 typewriter.launch.py
```
To try a different side-to-side distance, for example to see what happens
when an arm runs out of reach (section 5):
```bash
ros2 launch muscleup_update1 typewriter.launch.py shift:=0.25
```

**Offline check** (no RViz needed; works in `multipass shell`):
```bash
source ~/robotws/install/setup.bash
cd ~/robotws/src/muscleup-update1
python3 report/check.py           # default shift 0.15
python3 report/check.py 0.30      # an over-extended shift
```
It runs two full 27 s cycles. Every 1 s it prints the hand and pelvis
tracking errors, the fastest joint, both elbow angles, and each arm's smallest
singular value. At the end it prints the worst values and any joint-limit
violations. It also prints how much the joints at the end of cycle 2 differ
from the end of cycle 1; 0 means the loop repeats exactly, with no drift.

---

## 2. Files

| File | What it does |
|------|--------------|
| `muscleup_update1/typewriter.py` | The **trajectory** (`HANG`, `UP`, `REPS`, `segments(shift)`, `trajectory(t, shift)`) and the **ROS node**. Each 10 ms tick it asks the trajectory for the desired pelvis motion, runs one IK step, and publishes. Same structure as the HW5/HW6 nodes. |
| `muscleup_update1/g1.py` | The **robot model** (joint names, limits, bar, grip, finger angles) and the **whole-body IK** (`class G1`). It has no ROS node, so `check.py` can run it offline. |
| `muscleup_update1/KinematicChain.py` | HW5 kinematic chain: `fkin(q)` returns `(p, R, Jv, Jw)` from the URDF. Unchanged. |
| `launch/typewriter.launch.py` | Starts `robot_state_publisher` (with the URDF), RViz, and the node. Takes the `shift:=` argument. |
| `urdf/g1_29dof_hands.urdf` | G1 with Dex3 hands; meshes come from `g1_description`. Unchanged. |
| `rviz/muscleup.rviz` | RViz view, now nearly front-on so the side-to-side motion is visible. `hands.rviz` is a close-up of the left hand. |
| `report/check.py` | Runs the motion offline and reports errors, speeds, singularity, and limits. |

---

## 3. How the motion is built

### 3.1 The trajectory (cubic splines)

Only the **pelvis** gets a trajectory. Everything else (arms, waist, legs)
is solved by the IK. The pelvis stays upright (`Rd = Reye()`, `wd = 0`) and
its position moves between four points measured from the bar center (x
forward, y toward the robot's left, z up):

```
HANG  = bar + (-0.05, 0,      -0.64)     bottom of the pull-up
UP    = bar + (-0.15, 0,      -0.42)     top of the pull-up
LEFT  = UP  + (0,  +shift, 0)            body under the left hand
RIGHT = UP  + (0,  -shift, 0)            body under the right hand
```

One cycle is a list of moves, `segments(shift)`, each a
`(start, end, duration)`:

```
(HANG,  HANG,  2.0)                       rest (start == end, so it holds still)
(HANG,  UP,    2.0)                       pull up
(UP,    LEFT,  1.5)                       shift to the left hand
(LEFT,  RIGHT, 3.0), (RIGHT, LEFT, 3.0)   one typewriter rep, repeated REPS = 3 times
(LEFT,  UP,    1.5)                       back to the center
(UP,    HANG,  2.0)                       lower down
```

`trajectory(t, shift)` wraps the time into the cycle (`t % 27`, like `fmod`
in your HW5 and HW6 nodes), walks the list subtracting each duration until it
finds the current move, and evaluates that move. Each cycle ends at `HANG`,
where the next one starts, so the loop is seamless.

Each move uses `goto(t, T, p0, pf)` from `utils/TrajectoryUtils.py`, a
cubic polynomial with zero velocity at both ends:

```
tau = t/T
p(t) = p0 + (pf - p0) (3 tau^2 - 2 tau^3)
v(t) =      (pf - p0)/T (6 tau - 6 tau^2)
```

- **Why `goto`:** every segment starts and ends at rest, so position and
  velocity are continuous where one segment hands over to the next. The
  robot briefly stops at each side, which suits a typewriter. The peak
  speed, at mid-segment, is 1.5 × (distance / T): 0.15 m/s for a 0.30 m
  shift over 3 s.
- **What a cubic doesn't give you:** acceleration jumps at each boundary
  (for example from +a to −a at the far side). Kinematically that's
  harmless; on a real robot it means a sudden change of force. `goto5`
  (quintic) would also make acceleration continuous, at the same cost.
- **Changing the routine** means editing the list: more reps (`REPS`),
  longer rests, or new moves. No other code changes.
- `trajectory()` returns `(pd, vd, Rd, wd)`. The IK uses `vd` as a
  **feed-forward** term and `pd` to compute the **error**.

### 3.2 Kinematics

**Degrees of freedom.** The pelvis isn't bolted to anything, so it is
treated as 6 extra unactuated DOFs (linear velocity `v` and angular velocity
`w`) in front of the 29 joints:

```
qdot = [ v_pelvis (3), w_pelvis (3), qdot_joints (29) ]     35 total
```

**Chains.** There are two HW5 `KinematicChain`s, `pelvis → left_hand_palm_link`
and `pelvis → right_hand_palm_link`. Each goes through the 3 waist joints and
the 7 arm joints, so the waist is shared by both hands.

**Grip point.** The bar doesn't sit at the palm frame's origin. It sits
against the palm at the base of the fingers, offset by `r = R·GRIP`. Moving
the tip point by `r` changes the Jacobian the same way as in lecture:

```
p_grip  = p_palm + r
Jv_grip = Jv - [r]x Jw          ([r]x = crossmat(r))
```

**Floating base.** Converting from pelvis coordinates to world coordinates
gives `p = p_pelvis + R_pelvis · p_local`, and its velocity:

```
pdot = v_pelvis + w_pelvis × (p - p_pelvis) + R_pelvis Jv qdot
```

So the hand's 6×35 world Jacobian (`G1.hand()`) is

```
        [ I    -[p - p_pelvis]x    R_pelvis Jv  (in this arm's 10 columns) ]
J_hand = [ 0          I            R_pelvis Jw                             ]
```

### 3.3 Tasks (what the IK must satisfy)

| Rows | Task | Error |
|------|------|-------|
| 3 | Left grip point on its spot on the bar | `ep(PHAND_left, p)` |
| 2 | Left grip axis parallel to the bar | `PERP · (a × ŷ)` |
| 3 | Right grip point on the bar | same |
| 2 | Right grip axis parallel to the bar | same |
| 6 | Pelvis position and orientation | `ep(pd, p)`, `eR(Rd, R)` |
| **16** | of 35 DOFs, which leaves **19 redundant DOFs** | |

Each hand gets only **5** constraints, not 6. `a = R·GRIPAXIS` is the hand
axis that runs along the bar, and `a × ŷ` is zero when it lies along the
bar's direction ŷ. `PERP` keeps only the x and z components, so rolling the
hand *about* the bar is left free, the way a real hand rolls on a bar during
a pull-up.

### 3.4 Inverse kinematics (one step every 10 ms)

```
xr_dot = xd_dot + lam · e                        lam = 20
qdot   = W (J W)^T [ (J W)(J W)^T + gamma^2 I ]^-1  xr_dot     (J# for short)
       + (I - J# J) · qdot_secondary
q     += dt · qdot_joints
p_pelvis += dt · v,    R_pelvis = Rotn(w/|w|, |w| dt) · R_pelvis
```

- `xd_dot` is zero in the 10 hand rows (the bar doesn't move) and
  `(vd, wd)` in the pelvis rows.
- `lam · e` is the HW5 feedback term that pulls drift back to zero.
- **Weighted pseudo-inverse** (HW6 P1): `W` is 1 for every DOF except the
  waist, which gets 0.15. A low weight makes a joint "expensive", so the
  shoulders and elbows do the work rather than the robot bending at the waist.
- **Damped** (HW6 P2): `gamma = 0.02`. In SVD terms, each `1/σ` becomes
  `σ/(σ² + γ²)`. That's nearly `1/σ` for healthy directions, but it caps
  the gain at `1/(2γ) = 25` instead of letting it blow up as σ → 0 (a
  singularity). Section 5 shows why this matters.
- **Null space** `(I - J# J)`: velocities in it don't disturb the 16 tasks
  (approximately; see the damping note in section 5). That's where the
  secondary task goes:
  - **Posture:** `lams · (QNOM - q)` with `lams = 5`, applied only to the
    legs, waist, and wrists. Legs stay together and slightly bent, and the
    waist and wrists stay near neutral.
  - **Joint limits:** any joint within 0.15 rad of a limit gets pushed back
    with gain `lamlim = 50`.
- **Starting pose (Newton–Raphson):** the constructor runs the same step 200
  times with `lam = 1/dt` and the pelvis held at `HANG`. Then
  `dt · qdot = J# e`, which is exactly a Newton–Raphson update `Δq = J# e`,
  so the hands snap onto the bar before the motion starts. The initial guess
  (arms up and out: shoulder pitch −2.8, roll ±1.3, elbow 1.2) decides
  *which* solution it converges to.

### 3.5 What the G1's joints mean here

- **Elbow:** 0 rad means bent 90°. The arm is straightest, with the longest
  reach (about 0.41 m shoulder-to-palm, 0.49 m to the grip point), at about
  1.4–1.6 rad. Beyond that the joint is bending *backwards* (the URDF allows
  up to 2.09 rad). The fully folded limit is −1.05 rad.
- **Shoulder roll** (abduction) tops out at 2.25 rad. That limit is what
  stops the grip from being any wider.

---

## 4. Results (default `shift = 0.15`, from `report/check.py`)

| Metric | Value |
|---|---|
| Worst hand error (grip point off its spot) | 3.8 mm |
| Worst pelvis error | 4.3 mm |
| Fastest joint | 1.59 rad/s |
| Joint-limit violations | none |
| Elbows (bent side / extended side at full shift) | −0.95 / 1.14 rad |
| Closest body-to-bar distance | 2.3 cm (head, during the pull-up and again while lowering)* |
| Cycle 2 vs cycle 1 (largest joint difference) | 0.0000 rad, so the loop repeats exactly |

\*Measured with a mesh-distance check adapted from the original package's
`collisions.py`. That check isn't included here.

---

## 5. What happens at the edges

Every number below comes from actually running these cases (`check.py`, or
the same loop with one setting changed).

### 5.1 One arm is fully extended, and we keep shifting toward the other hand

As the body moves toward the left hand, the right arm has to reach further.
Once it's straight it **can't** reach further: the elbow is at a
**singularity**. In the 3×4 Jacobian of the shoulder and elbow joints, the
smallest singular value heads toward 0, because no joint velocity moves the
hand straight away from the shoulder. The task is now *infeasible*, and the
damped least-squares solver does the best it can:

| `shift` | Worst hand error | Worst pelvis error | Fastest joint | Extended elbow | Limits hit |
|---|---|---|---|---|---|
| 0.15 (default) | 3.8 mm | 4.3 mm | 1.6 rad/s | 1.14 | none |
| 0.20 | 12 mm | 12 mm | 2.3 rad/s | 1.31 | none |
| 0.25 | 32 mm | 33 mm | 2.8 rad/s | 1.35 | none |
| 0.30 | 53 mm | 54 mm | 3.3 rad/s | 1.36 | both elbows, just barely |
| 0.40 | 118 mm | 99 mm | **50 rad/s** | 1.61 | elbows, shoulder roll, waist roll |

What you'd see, and why:

1. **The error splits evenly between the hands and the pelvis.** All 16
   tasks have the same priority, and least squares minimizes the *total*
   squared error. So when the shift can't be achieved, the pelvis stops
   short of its target *and* the grip points get dragged off their spots on
   the bar by the same amount. In RViz the hands visibly slip off their
   grip spots. A real hand can't do that; the robot would lose its grip. To
   make the grip win instead, weight the pelvis rows lower than the hand
   rows (or move the pelvis task into the null space). The pelvis then lags
   and the hands stay put.
2. **The extended elbow stalls just short of straight** (about 1.35 rad).
   Damping refuses the very large joint speed that the last bit of
   straightening would need.
3. **The other arm runs out of bend.** Pushing further toward the left hand
   folds the left elbow tighter. At `shift = 0.30` it reaches −1.08 rad, past
   its −1.05 limit. The joint-limit push can't stop it, because it works in
   the null space of the primary tasks, and the primary tasks are demanding
   that motion. **Primary tasks beat secondary tasks.**
4. **Far beyond reach (0.40), it goes unstable:** the joint speed jumps to
   50 rad/s. The solver pushes the extended elbow right through
   straight (1.61 rad), and other joints hit their limits as they
   try to make up the distance.

### 5.2 Both arms extended (the hang is set too low)

Setting `HANG` lower than the arms can reach (for example z = −0.80
instead of −0.64) makes **both** arms singular at once. Newton–Raphson
can't close the gap. The hand error is about 10 cm, the pelvis error about
5 cm, the elbows sit at 1.38–1.43 rad (essentially straight), and joints
spike to 18 rad/s. The robot hangs stretched out with its hands off the bar.

### 5.3 Turning off the damping (`gamma = 0`) while over-extended

At `shift = 0.30` with no damping, `1/σ` blows up as σ → 0:

- Joint speeds reach **219 rad/s** (at `shift = 0.40`, 331 rad/s).
- Joints crash through limits all over the arms (elbows, shoulders, wrists).
- The hands end up 32 cm off the bar (67 cm at 0.40), even though the pelvis
  error stays at 0.2 mm. The pelvis rows are the trivially easy part of the
  Jacobian, so they stay solved while everything else falls apart.

Each enormous step `dt · qdot` throws the arm far from where the
linearization was valid, so each next step is worse. **Damping is what keeps
the singularities in 5.1 and 5.2 graceful.**

### 5.4 Too much damping (`gamma = 0.1`)

Even the *default* motion degrades: the hand error is 25 mm and the pelvis
error 33 mm. The extended elbow never gets past 0.61 rad, because heavy
damping makes long reaches "expensive" everywhere, not just at the
singularity. The trade-off is that small γ tracks well but is fragile near
singularities, while large γ is robust but sloppy.

### 5.5 No joint-limit push (`lamlim = 0`)

In the default motion nothing changes (4.2 mm, no violations), because the
motion stays clear of limits by design. At `shift = 0.30` the violations
just *move*: the shoulder rolls and waist roll go 0.08 rad over their limits
instead of the elbows. The push only redistributes motion within the
redundancy. It can't create reach that isn't there.

### 5.6 The IK is local: a 1 cm change can pick a different arm configuration

With `HANG` at z = −0.64 everything is clean. At **−0.65**, Newton–Raphson
converges to a *different* arm configuration (for example, the bent elbow
then only folds to −0.49 rad instead of −0.96), and that one carries the shoulder roll 0.05 rad past its limit
during the pull-up. Each IK step only follows the local gradient, so the
starting configuration decides the "branch" for the whole motion. That's why
the initial guess and the hang height matter.

### 5.7 Grip width

The grip width (`WIDTH` in `g1.py`) is limited from both sides:

- **0.80 m** (±0.40): the shoulder roll exceeds its 2.25 rad limit at the
  top of the pull-up. The G1 can't spread its arms that wide while pulled up.
- **0.60 m** (±0.30): the bent arm folds past the elbow limit (−1.05) during
  the shift, because the hand ends up too close to the shoulder.
- **0.72 m** (±0.36): the width that clears both limits.

### 5.8 What this simulation doesn't model

It's kinematics only:

- **No forces or dynamics.** Nothing checks whether the G1's motors could
  actually hold its weight, and nothing models swinging.
- **No grip physics.** The hands "stick" because they're commanded to. A
  hand sliding on the bar (5.1) would be a real slip.
- **No collision checking inside the IK.** Bar clearance was checked
  afterwards (section 4); self-collision wasn't checked at all.

---

## 6. Knobs

| Where | Name | Default | Effect |
|---|---|---|---|
| launch arg | `shift` | 0.15 m | Side-to-side distance (section 5.1) |
| `typewriter.py` | `HANG`, `UP` | see 3.1 | Bottom and top of the pull-up |
| `typewriter.py` | `REPS` | 3 | Typewriter reps per cycle |
| `typewriter.py` | durations in `segments()` | 2.0, 2.0, 1.5, 3.0, 1.5, 2.0 s | Rest length and speed of each phase |
| `g1.py` | `WIDTH` | 0.36 m | Half of the grip width (5.7) |
| `g1.py` | `lam` | 20 | How hard errors get corrected |
| `g1.py` | `gamma` | 0.02 | Damping (5.3, 5.4) |
| `g1.py` | `W` (waist) | 0.15 | Lower means less waist motion |
| `g1.py` | `lams`, `lamlim` | 5, 50 | Posture and joint-limit gains (5.5) |
