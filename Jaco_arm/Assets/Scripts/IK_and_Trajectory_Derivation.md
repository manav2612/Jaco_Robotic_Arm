# Manual IK + Trajectory Control — Derivation & Debug Walkthrough

This document derives, from first principles, the two pieces of math behind
`ManualJointControl.cs`:

1. **Inverse kinematics (IK)** — how a desired end-effector position gets
   turned into 6 joint angle updates that move together in sync.
2. **Minimum-jerk trajectory smoothing** — how the raw, jittery mouse target
   gets turned into a smooth motion path before being fed to the IK solver.

It also walks step-by-step through the code so each formula below is tied to
an exact line in `ManualJointControl.cs`.

---

## 1. System setup

The j2n6s200 arm is a serial chain of 6 revolute joints
`j2n6s200_link_1 … link_6`, each an `ArticulationBody` with an `xDrive`
(a PD position controller: give it a target angle in degrees, physics moves
the joint there over time).

Define:

- `p_i` = world-space position of joint *i*'s pivot (its rotation axis
  passes through this point).
- `a_i` = world-space unit vector of joint *i*'s rotation axis.
- `p_ee` = world-space position of the end effector (we use `link_6`'s
  transform as the reference point).
- `q = (q_1, …, q_6)` = the 6 joint angles (radians).
- `x = f(q) ∈ ℝ³` = forward kinematics: end-effector position as a function
  of joint angles.

We don't have (or need) a symbolic `f(q)` — Unity's physics engine already
computes it for us every frame via the transform hierarchy. We only need its
**derivative**, the Jacobian, which we can get from live transforms directly
(see §2).

**Code**: `Start()` in `ManualJointControl.cs` finds all 6
`ArticulationBody` references and sets `m_EndEffector` to `link_6`'s
transform.

---

## 2. The geometric Jacobian for a revolute-joint chain

### 2.1 Angular velocity → linear velocity at a point

For a rigid body rotating with angular velocity `ω` about an axis through
point `p_i`, any other point `p_ee` attached further down the kinematic
chain gets a linear velocity contribution:

```
v = ω × (p_ee − p_i)
```

This is the standard rigid-body kinematics identity: linear velocity of a
point due to rotation equals the cross product of the angular velocity
vector and the lever arm from the rotation axis to that point.

### 2.2 Stacking all 6 joints

Each joint *i* contributes independently (this is the chain rule applied to
the serial kinematic chain — each joint's motion propagates to the
end-effector regardless of the other joints' current values, because we're
differentiating, i.e. looking at instantaneous velocity, not finite
motion). If joint *i* spins at angular rate `q̇_i` about axis `a_i` through
point `p_i`:

```
∂x/∂q_i = a_i × (p_ee − p_i)
```

Stack these 6 column vectors into the **3×6 geometric Jacobian**:

```
J = [ a_1×(p_ee−p_1)   a_2×(p_ee−p_2)   …   a_6×(p_ee−p_6) ]
```

so that for small joint changes `dq`:

```
dx ≈ J · dq
```

This is exact to first order and doesn't require any DH table or symbolic
forward-kinematics model — just the *current* world position and axis of
each joint, which Unity's transform hierarchy already gives us, and which
updates correctly automatically as the arm moves (`J` is recomputed every
call, not cached).

### 2.3 Getting `a_i` and `p_i` from Unity

For an `ArticulationBody` configured as a revolute joint by the URDF
Importer, the drive rotates the body about the **local X axis of the joint
frame**, defined by `anchorRotation` relative to the body's own transform.
Since rotating a frame about its own X axis never changes where that X axis
points, we can read the *current* world-space axis directly:

```
a_i = joint.transform.TransformDirection(joint.anchorRotation * Vector3.right)
p_i = joint.transform.TransformPoint(joint.anchorPosition)
```

**Code**: `SolveAndApplyIkStep()`, the loop building `jacobianColumns[i]`.

---

## 3. From Jacobian to joint motion: solving the inverse problem

We want the opposite direction: given a desired small end-effector
displacement `e = x_desired − x_actual ∈ ℝ³`, find `dq ∈ ℝ⁶` such that
`J·dq ≈ e`.

`J` is 3×6 — **underdetermined** (6 unknowns, 3 equations), so there are
infinitely many solutions. We want the one that moves the joints as little
as possible (the minimum-norm solution), which keeps the motion looking
natural instead of making huge, arbitrary swings in the null space.

### 3.1 Minimum-norm least-squares solution (pseudo-inverse)

We minimize `‖dq‖²` subject to the constraint `J·dq = e`. Using Lagrange
multipliers:

```
L(dq, λ) = ½‖dq‖² + λᵀ(J·dq − e)
```

`∂L/∂dq = dq + Jᵀλ = 0  ⟹  dq = −Jᵀλ`

Substituting into the constraint: `J(−Jᵀλ) = e ⟹ λ = −(J·Jᵀ)⁻¹·e`

```
dq = Jᵀ (J·Jᵀ)⁻¹ e
```

This is the **right pseudo-inverse** `J⁺ = Jᵀ(J·Jᵀ)⁻¹`. It works great —
until `J·Jᵀ` (a 3×3 matrix here) becomes singular or near-singular, which
happens whenever the arm approaches a kinematic singularity (e.g. fully
extended — at that pose, the Jacobian columns become linearly dependent
and `J·Jᵀ` loses rank). Near-singular inverses blow up, producing huge,
erratic joint velocities.

### 3.2 Damped least squares (DLS)

Instead of exactly solving `J·dq = e`, we minimize a *regularized* cost:

```
C(dq) = ‖J·dq − e‖² + λ²‖dq‖²
```

The second term penalizes large joint motions, trading a bit of tracking
accuracy for numerical stability. Expanding and taking the gradient:

```
∂C/∂dq = 2Jᵀ(J·dq − e) + 2λ²dq = 0
⟹ (JᵀJ + λ²I)·dq = Jᵀe
```

Using the identity `Jᵀ(J·Jᵀ + λ²I)⁻¹ = (JᵀJ + λ²I)⁻¹Jᵀ` (true for any matrix
`J`, by the matrix inversion / push-through identity), the solution can be
computed via the smaller **3×3** system instead of a 6×6 one:

```
dq = Jᵀ (J·Jᵀ + λ²I)⁻¹ e
```

This is the **damped least squares / Levenberg-Marquardt inverse**. As
`λ → 0` it reduces to the exact pseudo-inverse; as `λ` grows it smoothly
degrades to a scaled gradient-descent step, which stays well-behaved even
when `J·Jᵀ` is singular, because `J·Jᵀ + λ²I` never is (for `λ > 0`).

**Code**: `SolveAndApplyIkStep()` builds the 3×3 symmetric matrix
`m00..m22 = J·Jᵀ + λ²I`, inverts it with `Invert3x3Symmetric` (closed-form
cofactor/adjugate formula for a 3×3 — fast, no iterative solver needed),
then computes `weighted = (J·Jᵀ + λ²I)⁻¹·e` and
`dq_i = Jᵀ_row_i · weighted = jacobianColumns[i] · weighted`.

### 3.3 Singularity-robust adaptive damping

A fixed `λ` is a compromise: too small and it's unstable near singularities,
too large and it's sluggish everywhere else. Instead we scale `λ`
dynamically using **manipulability** (Yoshikawa, 1985):

```
w = √det(J·Jᵀ)
```

`w` is proportional to the volume of the "velocity ellipsoid" the arm can
achieve at its current pose — it's large in the middle of the workspace and
goes to zero exactly at a singularity. We ramp damping up sharply only when
`w` drops below a threshold `w₀`:

```
λ_eff = λ_base · (1 + 4·(1 − w/w₀)²)   if w < w₀
λ_eff = λ_base                          otherwise
```

(The quadratic ramp is a design choice — it's smooth (`C¹` continuous at
`w = w₀`) and only kicks in meaningfully close to the singularity, rather
than a hard cutoff that would cause a visible jerk.)

**Code**: `SolveAndApplyIkStep()`, the `manipulability` / `singularityFactor`
/ `effectiveLambda` block, right after building `J·Jᵀ`.

### 3.4 Applying the result

Finally, each joint's `xDrive.target` is nudged by `dq_i` (converted
radians → degrees), clamped to that joint's `lowerLimit`/`upperLimit`:

```
θ_i(new) = clamp(θ_i(old) + dq_i · 180/π, lowerLimit, upperLimit)
```

Because this runs **every frame** against the *live* error (closed-loop,
not open-loop — see §4), small per-step linearization error from treating
`J` as constant over the step self-corrects on the next frame instead of
accumulating.

---

## 4. Closed-loop vs. open-loop control

Early versions of this script applied the **raw mouse delta** directly each
frame (open-loop): `dq` was computed from "how far did the mouse move," not
from "how far is the end effector from where it should be." Any error from
the linear (`dx ≈ J·dq`) approximation would then **compound** frame after
frame, since nothing was measuring or correcting it.

The fix: recompute the error from the *actual* live end-effector position
every frame:

```
e = x_target − x_actual(t)
```

and solve DLS for *that*. This is standard closed-loop (feedback) control —
any drift from a previous step's approximation shows up as part of next
frame's error and gets corrected automatically, instead of silently
accumulating.

**Code**: `Update()` — `var error = smoothedTarget - m_EndEffector.position;`
is recomputed from `m_EndEffector.position` (the live transform) every
single frame, not from a remembered/assumed position.

---

## 5. Minimum-jerk trajectory smoothing

Feeding the IK solver a goal that jumps around with every noisy mouse-move
sample makes the arm visibly twitch. We want the *motion profile* itself to
be smooth, independent of how jittery the input is.

### 5.1 The minimum-jerk model

Flash & Hogan (1985) showed that human point-to-point reaching movements
closely match the trajectory that minimizes the integral of **squared
jerk** (rate of change of acceleration) over the movement duration:

```
minimize  ∫₀ᵀ (d³x/dt³)² dt
subject to:
  x(0) = x_start,  ẋ(0) = 0,  ẍ(0) = 0
  x(T) = x_end,    ẋ(T) = 0,  ẍ(T) = 0
```

i.e. start and end at rest, with zero acceleration at both ends too (no
abrupt "kick" at the start or sudden stop at the end). This is a calculus
of variations problem; solving its Euler-Lagrange equation
(`d⁶x/dt⁶ = 0`) gives a **quintic (5th-order) polynomial** in normalized
time `t ∈ [0, 1]`:

```
x(t) = x_start + (x_end − x_start) · s(t)
s(t) = 10t³ − 15t⁴ + 6t⁵
```

### 5.2 Deriving the coefficients

Write `s(t) = c₀ + c₁t + c₂t² + c₃t³ + c₄t⁴ + c₅t⁵`. The 6 boundary
conditions (position/velocity/acceleration at `t=0` and `t=1`) give 6
linear equations in the 6 unknowns `c₀…c₅`:

```
s(0)  = 0   ⟹ c₀ = 0
ṡ(0)  = 0   ⟹ c₁ = 0
s̈(0)  = 0   ⟹ 2c₂ = 0 ⟹ c₂ = 0
s(1)  = 1   ⟹ c₃ + c₄ + c₅ = 1
ṡ(1)  = 0   ⟹ 3c₃ + 4c₄ + 5c₅ = 0
s̈(1)  = 0   ⟹ 6c₃ + 12c₄ + 20c₅ = 0
```

Solving the last 3 equations simultaneously:

```
From eq (ṡ(1)):  3c₃ + 4c₄ + 5c₅ = 0
From eq (s̈(1)):  6c₃ + 12c₄ + 20c₅ = 0  ⟹  3c₃ + 6c₄ + 10c₅ = 0

Subtract: (3c₃+6c₄+10c₅) − (3c₃+4c₄+5c₅) = 0 ⟹ 2c₄ + 5c₅ = 0 ⟹ c₄ = −2.5c₅
Substitute into 3c₃+4c₄+5c₅=0: 3c₃ − 10c₅ + 5c₅ = 0 ⟹ 3c₃ = 5c₅ ⟹ c₃ = (5/3)c₅
Substitute into c₃+c₄+c₅=1: (5/3)c₅ − 2.5c₅ + c₅ = 1 ⟹ (5/3 − 5/2 + 1)c₅ = 1
  5/3 − 5/2 + 1 = 10/6 − 15/6 + 6/6 = 1/6
  ⟹ c₅/6 = 1 ⟹ c₅ = 6
c₄ = −2.5·6 = −15
c₃ = (5/3)·6 = 10
```

```
s(t) = 10t³ − 15t⁴ + 6t⁵   ✓ matches the known minimum-jerk formula
```

Sanity checks: `s(0)=0`, `s(1)=10−15+6=1`, `ṡ(t)=30t²−60t³+30t⁴`, so
`ṡ(0)=ṡ(1)=0` ✓, and `s̈(t)=60t−180t²+120t³`, so `s̈(0)=s̈(1)=0` ✓.

### 5.3 Re-planning on the fly

Because the mouse goal moves continuously while dragging, we can't just run
one trajectory from drag-start to drag-end — we don't know the end until
the user stops. Instead, **every time the goal moves**, we re-plan a fresh
short (`m_TrajectoryDuration ≈ 0.12s`) minimum-jerk segment:

```
start  = (wherever the smoothed point currently is)   ← for continuity, no pop
end    = (new mouse-driven goal)
t      = clamp01((now − replan_time) / duration)
output = lerp(start, end, s(t))
```

Starting each new segment from the *current smoothed position* (not the
raw goal) guarantees `C⁰` continuity (no position jump) at every re-plan,
even though we're constantly interrupting one trajectory to start another.
It is **not** `C¹`-continuous across re-plans (velocity can have a small
kink when the goal direction changes abruptly), but because re-plans happen
every frame during a drag and `duration` is short, this is imperceptible in
practice — the net effect is simply a responsive low-pass/ease filter on
the raw mouse signal.

**Code**: `EvaluateMinimumJerkTrajectory()` implements `s(t)` and the lerp;
`Update()` re-baselines `m_TrajStartPos` / `m_TrajStartTime` / `m_TrajEndPos`
whenever `worldDelta` is non-zero.

---

## 6. Full per-frame pipeline (debug checklist)

```
1. Read mouse/keyboard input → worldDelta (§ drag-mode plane projection)
2. m_DesiredPosition += worldDelta                     (raw goal, shown in GUI)
3. Re-plan minimum-jerk trajectory toward m_DesiredPosition   (§5.3)
4. smoothedTarget = EvaluateMinimumJerkTrajectory(now)  (shown in GUI as "Trajectory pt")
5. error = smoothedTarget − m_EndEffector.position      (closed-loop, §4)
6. Build J (3×6) from live joint transforms             (§2)
7. Build J·Jᵀ (3×3), compute manipulability w            (§3.3)
8. λ_eff = adaptive damping from w                       (§3.3)
9. Solve dq = Jᵀ(J·Jᵀ + λ_eff²I)⁻¹ · clamp(error, maxStep) (§3.2)
10. Apply dq to all 6 xDrive.target, clamped to joint limits (§3.4)
11. UpdateUnreachableFlag compares error shrinkage over a 1.5s window (§ unreachable detection)
```

If motion looks wrong, check the on-screen debug panel in this order:

| Symptom | Likely cause | Where to look |
|---|---|---|
| No motion at all, "camera is null" | `Camera.main` not tagged/found | `Start()`, scene's Main Camera tag |
| No motion, "mouse held but delta is zero" | Input axes returning 0 (unfocused Game view) | Click into the Game view first |
| Huge/erratic jump on tiny mouse move | Stale serialized `m_Sensitivity` from an old component version | Re-add the component fresh (see `ManualControlFixer.ResetManualJointControlAndOpen`) |
| "singular Jacobian (det~0)" | All joint axes momentarily (near-)parallel w.r.t. end effector | Should self-heal via adaptive damping; check `m_ManipulabilityThreshold` |
| "Desired pos" turns red / UNREACHABLE | Error not shrinking ≥15% over 1.5s | Compare `m_DesiredPosition` distance from `m_ArmBasePosition` against `m_MaxReachMeters` |
| Jittery even with trajectory smoothing | `m_TrajectoryDuration` too short, or `m_MaxStepMeters` too large | Increase duration, decrease max step |
