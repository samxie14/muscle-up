# ME 133a Project, Update 2: G1 Muscle-up

The Unitree G1 hangs from a bar and does a full **muscle-up**: hang, pull,
transition (chest over the bar), press to straight-arm support, hold, and
lower back down. It repeats every 12 s.

New in update 2:

1. **The full muscle-up motion** (update 1 was a pull-up plus typewriter).
2. **Task priority.** The grip is priority 1, the pelvis is priority 2, and
   posture is priority 3. Update 1 put all 16 task rows into one
   equal-weight least-squares solve.
3. **Out-of-reach tests.** The pelvis is asked to go far too low, or far too
   far left or right. With the new priority the hands stay on the bar and the
   body stops at the edge of reach. Joint speeds stay at 7 rad/s or less.

| Time (s) | Phase      | Pelvis motion                      |
|----------|------------|------------------------------------|
| 0 – 1    | Hold       | `HANG`                             |
| 1 – 3    | Pull       | `HANG` → `PULL`                    |
| 3 – 4.5  | Transition | `PULL` → `DIP` (chest over the bar) |
| 4.5 – 6  | Press      | `DIP` → `SUPPORT`                  |
| 6 – 7.5  | Hold       | `SUPPORT`                          |
| 7.5 – 12 | Lower      | `SUPPORT` → `DIP` → `PULL` → `HANG` |

---

## 1. Running it

The ROS package is still named **`muscleup_update1`**. It now holds both
Python modules (`muscleup_update1` for the typewriter and `muscleup_update2`
for the muscle-up), so update 1 still runs unchanged. Build it the same way
as update 1, then:

```bash
ros2 launch muscleup_update1 muscleup.launch.py                          # the muscle-up
ros2 launch muscleup_update1 muscleup.launch.py motion:=low  target:=-1.0  # pelvis far too low
ros2 launch muscleup_update1 muscleup.launch.py motion:=side target:=0.40  # far too far left
ros2 launch muscleup_update1 muscleup.launch.py motion:=side target:=-0.40 # far too far right
ros2 launch muscleup_update1 muscleup.launch.py motion:=low target:=-1.0 priority:=false  # update 1's equal weights
ros2 launch muscleup_update1 muscleup.launch.py view:=muscleup.rviz      # front view instead of side
ros2 launch muscleup_update1 typewriter.launch.py                        # update 1, unchanged
```

See `STUDY_GUIDE.md` for the full explanation of every piece.

**Offline check** (no RViz needed):
```bash
python3 report/check2.py                      # muscle-up
python3 report/check2.py low  -1.0            # pelvis target z = -1.0 m below the bar
python3 report/check2.py side  0.40           # pelvis target 0.40 m toward the left hand
python3 report/check2.py side -0.40           # ... toward the right hand
python3 report/check2.py low  -1.0 --equal    # update 1's equal weights, for comparison
python3 report/check2.py low  -1.0 --gamma 0  # no damping
```

## 2. Files

| File | What it does |
|------|--------------|
| `muscleup_update2/g1.py` | Robot model (joints, limits, bar, grip, fingers) and the **prioritized whole-body IK** (`class G1`). No ROS. |
| `muscleup_update2/muscleup.py` | Trajectory (`HANG`, `PULL`, `DIP`, `SUPPORT`, `knots`, `trajectory`) and the ROS node. Parameters: `motion`, `target`. |
| `launch/muscleup.launch.py` | Starts `robot_state_publisher`, RViz (side view, `rviz/muscleup_side.rviz`), and the node. Args: `motion`, `target`, `priority`, `view`. |
| `report/check2.py` | Runs a motion offline for two cycles and reports errors, speeds, elbows, singular values, and limits. |
| `muscleup_update1/KinematicChain.py` | HW5 chain, reused unchanged. |

## 3. Method

**DOFs: 35** = 6 floating pelvis (v, ω) + 29 joints (12 legs, 3 waist,
14 arms). The 14 Dex3 finger joints are fixed and not part of the IK. Each
hand's chain is pelvis → 3 waist → 7 arm joints → palm.

**Tasks, in priority order:**

| Priority | Task | Rows |
|---|---|---|
| 1 | Each hand: grip point on the bar (3) + grip axis along the bar (2) | 10 |
| 2 | Pelvis position (3) + orientation (3) | 6 |
| 3 | Posture (legs, waist, wrists) + joint-limit push | null space, 35 − 16 = 19 |

Each hand gets 5 constraints. The 6th, rotation about the bar, is left
free: `PERP = [x̂; ẑ]` keeps only the ω_x, ω_z rows of the hand's angular
Jacobian, and the matching two components of the alignment error
`a × ŷ`. During the muscle-up each hand rolls about 187° about the bar.

**One IK step (`G1.step`):**
```
xr1  = lam e1                                   grip: the bar is still
xr2  = [vd; wd] + lam cap(e2)                   pelvis: feedforward + feedback
qdot = J1# xr1 + N1 (J2 N1)# (xr2 - J2 J1# xr1) + (I - J12# J12) qsec
N1   = I - J1# J1
J#   = W (JW)^T [(JW)(JW)^T + gamma^2 I]^-1     weighted, damped
```
- `lam = 20`. The waist has weight `W = 0.15`, so the arms do the work.
- Grip damping is `gamma = 0.02`. Pelvis damping is `gamma2 = 0.05`, so an
  unreachable pelvis can't whip the arms.
- `cap` limits the pelvis feedback error to 0.10 m.
- Posture uses `lams = 5`. The joint-limit push uses `lamlim = 50` within
  0.15 rad of a limit.
- The start pose comes from Newton–Raphson (200 steps with `lam = 1/dt`).

**Trajectory:** cubic splines through the waypoints. Interior knots get the
Catmull–Rom velocity, and holds get zero velocity. Waypoints are given as
(x, y, z) relative to the bar, plus pelvis pitch and the leg hip/knee
posture:

```
HANG    = (-0.03, 0, -0.73)  pitch  0.00   hip -0.10  knee 0.10
PULL    = (-0.13, 0, -0.44)  pitch -0.10   hip -0.35  knee 0.20
DIP     = (-0.25, 0, -0.15)  pitch  0.40   hip -0.70  knee 0.40
SUPPORT = (-0.12, 0, +0.12)  pitch  0.12   hip -0.15  knee 0.05
```

## 4. Results (muscle-up, `report/check2.py`)

| Metric | Value |
|---|---|
| Worst hand (grip point) error | 1.7 mm |
| Worst pelvis error | 5.8 mm |
| Fastest joint | 1.49 rad/s |
| Joint-limit violations | none (the elbows reach −1.04 in the dip, limit −1.047) |
| Cycle 2 vs cycle 1 | 0.0000 rad, so the loop repeats exactly |

## 5. Out-of-reach tests

All three columns use the same pelvis error cap. **Grip-first** is the
update 2 IK. **Equal weights** is update 1's single stacked solve. **No
damping** is the update 2 IK with `gamma = gamma2 = 0`.

### 5.1 Pelvis too low (`low`): the arms are fully straight at z ≈ −0.78

| Target z | Grip-first: hand err / max speed | Equal weights | No damping |
|---|---|---|---|
| −0.78 | 0.3 mm / 0.7 rad/s | 1.0 mm / 0.8 rad/s | 0.1 mm / 0.9 rad/s |
| −0.85 | 0.3 mm / 1.6 rad/s | 23 mm / 1.8 rad/s | 771 mm / 572 rad/s |
| −1.00 | 0.3 mm / 2.1 rad/s | 126 mm / 59 rad/s | 853 mm / 905 rad/s |
| −1.50 | 0.4 mm / 3.4 rad/s | 238 mm / 91 rad/s | 1610 mm / 1670 rad/s |
| −3.00 | 0.7 mm / 5.5 rad/s | | |

**Grip-first:** the robot hangs at full stretch with its hands on the bar.
The pelvis stops at z ≈ −0.78, and the elbows stop at 1.41–1.42 rad, just
short of straight (~1.5). No limit is crossed. This is the straight-arm
singularity: `J2 N1` loses rank in the vertical direction, and damping sends
that gain to 0.

**Equal weights:** the hands slide off the bar, because least squares splits
the error between the pelvis and the grip.

**No damping:** 1/σ → ∞. The arms whip through every limit, and the elbows
wind to 4–11 rad.

### 5.2 Too far left / right (`side`): the far arm is straight at |y| ≈ 0.30–0.35

| Target \|y\| | Grip-first: hand / pelvis short / speed | Equal weights: hand / pelvis / speed | No damping: hand / speed |
|---|---|---|---|
| 0.30 | 2.3 mm / 15 mm / 2.1 rad/s | 5.2 mm / 5 mm / 2.1 rad/s | 0.2 mm / 2.2 rad/s |
| 0.40 | 6.2 mm / 69 mm / 3.0 rad/s | 34 mm / 34 mm / 3.3 rad/s | 86 mm / 55 rad/s |
| 0.60 | 10 mm / 256 mm / 4.6 rad/s | 180 mm / 115 mm / 68 rad/s | 1077 mm / 734 rad/s |
| 1.00 | 12 mm / 650 mm / 6.9 rad/s | | |

- **Left and right are mirror images.** Every number matches to 0.1 mm.
- **The far shoulder roll goes slightly past its limit** (0.007–0.025 rad)
  from |y| ≈ 0.25 up, with every method. The limit push is priority 3, so it
  can't overrule the grip or the pelvis. Turning the push off lets the joints
  run 4.5 rad past their limits.
- **The price of γ2 = 0.05** is pelvis lag near the reach edge: 15 mm at
  0.30, against 5 mm with equal weights at γ = 0.02.

### 5.3 What each update 2 change buys (measured with it removed)

| Change | Without it |
|---|---|
| Grip before pelvis | hands 23–238 mm off the bar (z = −0.85 to −1.5) |
| `N1 @` in front of `(J2 N1)#` | 6–10 mm grip leak (0.3 mm with it) |
| γ2 = 0.05 for the pelvis | 88 rad/s, elbows bent backwards (z = −1.0) |
| Pelvis error cap 0.10 m | 75 rad/s, 126 mm hand error (z = −1.5) |

It's still kinematics only: no forces, torques, dynamics, or collision
checks.
