# Localization tuning

> Why `ekf_node` and `mcl_node` were burning 80 % of a CPU core between them,
> what the corrected camera geometry means for their noise models, and what the
> filters can and cannot be trusted to do. Everything here was measured on the
> robot or derived from the projection geometry; none of it is a guess, and the
> two places where it is still a guess are called out as such.

---

## 1. Why this exists

[perception_gpu_migration.md §7.4](perception_gpu_migration.md) ended on a
caveat: after perception moved to C++, localization became the most expensive
thing on the machine. `ekf_node` at 42.8 % of a core and `mcl_node` at 37.4 %
together burned **80.2 %**, more than any other pair outside the camera process.

That document also fixed a projection bug that had put every field-line point
2.5–8.5 m from its true position. The filters downstream had been tuned — or
had accidentally compensated — against that wrong geometry. So there were two
questions, and the second matters more than the first:

1. Where is the CPU actually going?
2. Now that the inputs are correct, are the noise models still right?

The answer to (1) turned out to be "almost entirely in the wrong place from
where it looked". The answer to (2) is "no, and one of them was wrong by six
orders of magnitude".

---

## 2. Where the CPU was going

### 2.1 The EKF was not doing arithmetic

The obvious hypothesis for a 42.8 % filter is unoptimised matrix maths. That
hypothesis is wrong. Measured in isolation, in percent of one core:

| Timer rate | Bare `rclpy` spin | + 5×5 EKF predict/update | + TF and Odometry publish |
| --- | --- | --- | --- |
| 200 Hz | 18.2 | 19.0 | 29.7 |
| 100 Hz | 10.3 | 11.3 | 16.8 |
| 50 Hz | 5.5 | 6.0 | 9.3 |
| 25 Hz | 2.7 | 3.0 | 4.7 |

The 5×5 linear algebra costs **0.4 % of a core**, under 1 % of the node's total.
**61 % of the node's cost was the executor waking up 200 times a second before
doing any work at all.** The rest was message construction and publishing:
`get_clock().now().to_msg()` 0.2 %, the `Odometry` constructor 0.5 %, the
`TransformStamped` constructor 0.2 %, the full publish path 2.9 %.

Nothing needed 200 Hz. tf2 interpolates between transforms, and the only
consumer of `/odom` is the MCL, whose measurement update runs at ~22 Hz.

### 2.2 The MCL's measurement update was a Python loop

`ParticleFilter.update()` looped over particles in Python, transforming each
observation into that particle's frame one at a time. Measured cost of a single
`update()` call, and what that costs as a fraction of one core at the measured
22.2 Hz `field_features` rate:

| Particles | Observations | Loop | Vectorised | Speedup | Core freed |
| --- | --- | --- | --- | --- | --- |
| 150 | 60 | 15.44 ms | 0.81 ms | 19.1× | 32.5 % |
| 150 | 200 | 18.26 ms | 3.02 ms | 6.0× | 33.8 % |
| **300** | **60** | **30.12 ms** | **1.41 ms** | **21.3×** | **63.7 %** |
| 300 | 200 | 33.44 ms | 5.83 ms | 5.7× | 61.3 % |
| 500 | 60 | 50.72 ms | 2.37 ms | 21.4× | 107.3 % |
| 500 | 200 | 55.67 ms | 10.09 ms | 5.5× | 101.2 % |

`resample()` costs 0.009 ms and is irrelevant. Building `SoccerbotField()` costs
3.6 ms once at startup.

### 2.3 The measured baseline was hiding the real cost

> **Important.** At the time of the 37.4 % measurement, `field_features` was
> carrying **zero line points**. Sampled over 170 consecutive messages the
> minimum, mean and maximum count were all 0 — the robot is indoors, on a desk,
> with no grass and no field lines, so the HSV fallback finds nothing.

`update()` therefore returned on its second line. **The 37.4 % baseline excludes
the entire measurement cost.** On a pitch, with the old loop, `mcl_node` would
have been at roughly 100 % of a core, not 37 %. The baseline understated the
on-pitch cost by about 3×.

---

## 3. What the corrected geometry means for the noise models

This is the part that matters more than the CPU work.

### 3.1 The observation model was built on a 2 m assumption

The MCL scored particles against a likelihood field baked with a single fixed
`line_sigma = 0.15 m`, applied to every observation regardless of range. That is
not how ground projection error behaves.

A pixel error `δα` at the image plane maps to a ground range error through the
derivative of the flat-ground projection. For a camera at height `h` looking at
a point `r` metres away:

$$\frac{\partial r}{\partial \alpha} = h + \frac{r^2}{h}$$

The $r^2/h$ term dominates fast. With the ZED Mini's real intrinsics
($f_y = 732.9$ px), a 0.30 m mount, 2 px of segmentation noise, 0.5° of tilt
calibration error and 1 cm of mount-height error:

| Range | Pixel noise | Tilt error | Height error | Total σ |
| --- | --- | --- | --- | --- |
| 1.0 m | 0.010 m | 0.032 m | 0.033 m | **0.069 m** |
| 2.0 m | 0.037 m | 0.119 m | 0.067 m | **0.150 m** |
| 3.0 m | 0.083 m | 0.264 m | 0.100 m | **0.298 m** |
| 4.0 m | 0.146 m | 0.467 m | 0.133 m | **0.509 m** |
| 5.0 m | 0.228 m | 0.728 m | 0.167 m | **0.782 m** |
| 6.0 m | 0.328 m | 1.047 m | 0.200 m | **1.116 m** |

Two conclusions fall out of this table.

**The shipped 0.15 m is exactly the error at 2 m.** It is an honest number for a
2 m observation and nothing further. Meanwhile the field-line node's range gate
was set to 6 m, where the real error is 1.116 m. The filter was treating those
points as **7.4× more precise than they are**.

**Tilt calibration dominates beyond 2 m, not segmentation quality.** At 4 m the
tilt term is 3.2× the pixel term. Training a better segmentation network buys
almost nothing at range; measuring the camera's tilt properly buys a lot. That
is a concrete steer on where to spend effort, and it is the opposite of where
one would naturally look.

### 3.2 So where should the range gate be?

Solving the model for the range at which total σ crosses a threshold:

| Tolerated σ | Usable range |
| --- | --- |
| 0.15 m | 2.00 m |
| 0.25 m | 2.71 m |
| 0.50 m | 3.96 m |

**4 m is the defensible gate.** Past it, each extra metre adds points whose
error grows faster than their information content. `max_range_m` in both
[perception_gpu.yaml](../../ros2_ws/src/soccer_perception_gpu/config/perception_gpu.yaml)
and the MCL is now 4.0.

### 3.3 The EKF was under-trusting its own IMU by six orders of magnitude

The EKF hardcoded `R = diag(1e-3, 1e-3)` for the IMU's yaw and yaw-rate
measurements. The ZED actually reports its covariances, and they are nothing
like that:

| Quantity | Hardcoded | ZED reports | Ratio |
| --- | --- | --- | --- |
| Yaw variance | 1e-3 rad² | 2.70e-10 rad² | 3.7e6 |
| Yaw-rate variance | 1e-3 rad²/s² | 5.20e-06 rad²/s² | 192 |

The filter was not choking on noisy data. It was **ignoring good data**. `R` now
comes from the message.

It does not come from the message unconditionally, though. 2.7e-10 rad² is 16
microradians, which describes the ZED's short-term orientation noise, not the
unbounded yaw drift of a gyro with no magnetometer to anchor it. Accepting it
literally would make the filter certain of a heading that is slowly wrong.
`yaw_variance_floor` and `yaw_rate_variance_floor` (both 1e-6) put a floor under
it. **Those two floors are the least evidence-backed numbers in this document**
— they are a judgement call, not a measurement, and they should be replaced with
a measured Allan-variance figure when the robot can be left stationary on a
pitch for a few minutes.

### 3.4 Process noise was a function of the callback rate

`predict()` added a fixed 0.02 m of position noise **per call**, with no
reference to how much the robot had actually moved or how much time had passed.
At the 200 Hz odometry rate that is a 0.28 m/s random walk while standing
perfectly still. Measured over 200 predict steps with zero motion, the particle
cloud spread to **0.250 m**. With noise scaled by motion and $\sqrt{dt}$:
**0.018 m**.

This mattered beyond accuracy. Because diffusion depended on the callback rate,
dropping the odometry rate would have silently changed the filter's behaviour.
Fixing the process noise was a **prerequisite** for the EKF rate change in §4,
not an independent improvement.

---

## 4. What changed

### 4.1 EKF

| Change | Why |
| --- | --- |
| Predict moved onto the IMU callback | The node wakes for the IMU anyway; the prediction is then free |
| Timer does nothing but publish, at 50 Hz | §2.1: the wakeups were the cost |
| `R` taken from the IMU message, with floors | §3.3 |
| `inv(S)` → `solve()` | With `R` as small as the ZED reports, `S` is far better conditioned through a solve than an explicit inverse |
| `P` symmetrised after each update | `(I − KH)P` loses symmetry to rounding over the ~10⁵ updates in a match |
| `process_noise` exposed as a parameter | Tunable without a rebuild |

### 4.2 MCL

| Change | Why |
| --- | --- |
| Measurement update vectorised | §2.2 |
| Metric distance field replaces the baked Gaussian | Lets each observation carry its own σ |
| σ derived per observation from range | §3.1 |
| Range gate at 4 m, 60-observation budget | §3.2 |
| `z_rand` outlier floor | One bad point can no longer veto a particle |
| Independent-observation budget | 200 points along one line are not 200 independent measurements |
| Motion-scaled process noise | §3.4 |
| Real covariance published | It was previously always zero |

### 4.3 Field-line node

| Change | Why |
| --- | --- |
| Range gate on radial distance, not forward distance | A ray leaving the optical axis sideways just below the horizon lands at small `x` and enormous `y`, and passed a forward-only gate while being tens of metres away |
| Filter, then thin | The budget was being spent on points that were about to be discarded, so a frame that is mostly sky delivered a fraction of the points requested |
| `max_range_m` 6.0 → 4.0 | §3.2 |
| `max_mask_pixels` cap | An overexposed frame or a white wall in a green scene could turn into six figures of per-frame work |
| TensorRT shape validation at load | A zero mask dimension divided by zero on the first frame; a non-square input would be fed a distorted letterboxed crop, making every projected point wrong |

---

## 5. Measured results

### 5.1 CPU, end to end on the robot

`top` steady-state, percent of one core, measured from the rebuilt images rather
than a patched container:

| Node | Before | After | Saved |
| --- | --- | --- | --- |
| `ekf_node` | 43.2 % | **27.9 %** | 15.3 pts (−35 %) |
| `mcl_node` | 36.6 % | **19.1 %** | 17.5 pts (−48 %) |
| **Combined** | **79.8 %** | **47.0 %** | **32.8 pts (−41 %)** |

The MCL's saving here comes from the odometry rate drop, not the vectorised
update — the update is still returning early indoors (§2.3). **On a pitch the
vectorised update adds ~3 % of a core; the loop it replaced would have added
~64 %.** So the on-pitch comparison is roughly 100 % → 22 %.

`ekf_node` at 27.9 % is above the 9.3 % the timer alone would suggest, because
the ~99 Hz IMU subscription still costs about 10 % in bare wakeups plus
deserialisation and the update itself. That is inherent to the sensor rate.

### 5.2 Filter accuracy

Steady state, 12 cycles, 10 seeds, against synthetic observations corrupted with
the **range-dependent** error from §3.1 rather than a flat error:

| Configuration | Mean error | Max error | Effective sample size |
| --- | --- | --- | --- |
| Legacy: flat 0.15 m σ, no outlier floor | 0.163 m | 0.339 m | 251.5 / 300 |
| + outlier floor | 0.061 m | 0.140 m | 209.9 / 300 |
| + range-dependent σ | 0.077 m | 0.276 m | 231.0 / 300 |
| + correlation budget | 0.064 m | 0.135 m | 235.4 / 300 |
| **+ range gate (shipped)** | **0.075 m** | **0.173 m** | **277.0 / 300** |

**2.2× more accurate than the legacy configuration**, with a healthier particle
population. Reproduce with
[tools/profile_localization.py](../../tools/profile_localization.py).

### 5.3 Divergence checks

The brief was explicitly to make sure the filters do not diverge on the new
accurate inputs. They do not:

| Check | Result |
| --- | --- |
| 200 cycles stationary with observations | worst error 0.118 m |
| 400 cycles with **zero** observations | drift 0.002 m |
| Random garbage observations | estimate stays finite, reported spread 0.021 m |
| 2000 EKF updates | `P` symmetric to 1e-15, all eigenvalues positive |
| EKF clock jump of 30 s | clamped, state untouched |

---

## 6. What these filters cannot do

Validation turned up two limitations that are not bugs in the tuning and are not
fixed here. Both need recording before someone trusts the output on a pitch.

### 6.1 The field is symmetric and the line set cannot break the symmetry

The set of field lines is invariant under x-mirror, y-mirror and 180° rotation.
For any pose $(x, y, \theta)$ these four poses produce **identical** line
observations:

$$(x, y, \theta),\quad (-x, y, \pi-\theta),\quad (x, -y, -\theta),\quad (-x, -y, \theta+\pi)$$

No amount of tuning fixes this. It is a property of the map. A particle filter
fed only line points cannot know which half of the pitch it is on. The symmetry
must be resolved by the **initial** pose and preserved by odometry, or broken by
an asymmetric landmark or by team communication.

`soccer_msgs/FieldFeature.msg` anticipates this: it defines `TYPE_GOALPOST` and
describes goalposts as "the asymmetric anchors that disambiguate the symmetric
field". That path is currently dead in three separate ways:

1. [projection_node.py](../../ros2_ws/src/soccer_perception/soccer_perception/projection_node.py)
   publishes `TYPE_GOALPOST`, but [mcl_node.py](../../ros2_ws/src/soccer_localization/soccer_localization/mcl_node.py)
   filters for `TYPE_LINE_POINT` only and drops them.
2. It would not matter if it did not, because the detector's classes are
   `["ball", "robot"]` and stock COCO has no goalpost class, so the branch that
   emits a goalpost never runs.
3. **Even if both were fixed, goalposts at (±3, ±1) share all the same
   symmetries as the lines.** Goalposts alone do not disambiguate anything.

This is a recommendation, not a fix. Doing it properly means distinguishing the
two goals from each other, which needs either a fine-tuned detector that can
tell them apart or team communication.

### 6.2 Kidnap recovery is weak, and adding machinery did not help

Measured over 25 seeds, allowing ~11 s of observations after teleporting the
robot:

| `explorer_frac` | Augmented MCL | Recovered | Median cycles |
| --- | --- | --- | --- |
| 0.00 | no | 0 / 25 | – |
| 0.00 | yes | 7 / 25 | 4 |
| 0.02 | no | 6 / 25 | 34 |
| 0.02 | yes | 6 / 25 | 18 |
| 0.05 | **no** | **9 / 25** | 14 |
| 0.05 | yes | 7 / 25 | 7 |

Augmented MCL (the standard $w_{slow}/w_{fast}$ injection scheme) was
implemented, debugged twice, and then **removed**. At `explorer_frac = 0.05` it
did not beat a plain explorer fraction, and its peak trigger value was 0.002 —
negligible. It was unearned complexity and it is gone.

9/25 is not good. The reason is structural: a single lucky explorer particle
rarely outvotes a large cluster sitting on a wrong-but-plausible pose,
especially given §6.1. Improving this needs clustering or multi-hypothesis
tracking, not parameter tuning.

---

## 7. Parameters, and what to change if you need to

All defaults are in [mcl_node.py](../../ros2_ws/src/soccer_localization/soccer_localization/mcl_node.py),
[ekf_node.py](../../ros2_ws/src/soccer_localization/soccer_localization/ekf_node.py)
and [perception_gpu.yaml](../../ros2_ws/src/soccer_perception_gpu/config/perception_gpu.yaml).

### 7.1 If you need more CPU back

| Change | Saves | Costs |
| --- | --- | --- |
| `num_particles` 300 → 150 | ~0.7 % of a core on pitch | Mean error 0.075 → ~0.09 m |
| `publish_rate_hz` 50 → 25 | ~4.6 % of a core | Coarser TF; tf2 interpolates, so probably nothing |
| `max_observations` 60 → 30 | ~0.7 % of a core on pitch | Noisier updates |

The single largest remaining lever is **the IMU rate**. `imu/data` at ~99 Hz
costs about 10 % of a core in bare executor wakeups. Halving it at the ZED
driver would halve that. Not done here because it affects any other IMU consumer.

### 7.2 If localization is inaccurate on the pitch

Check in this order:

1. **Tilt calibration.** §3.1 — it dominates everything past 2 m. If the real
   tilt differs from `tilt_rad` by more than about 1°, nothing else matters.
2. **Mount height.** `mount_height_m` must match reality; the error scales as
   $r/h$.
3. `pixel_sigma_px` — raise it if the segmentation is ragged. It is currently
   2.0 px, an estimate, not a measurement.
4. `z_rand` — raise it if there are false line detections (crowd, markings,
   other robots' feet).
5. Only then consider `num_particles`.

### 7.3 Numbers in here that are still estimates

Stated plainly, because the rest of this document is measured and these are not:

- `pixel_sigma_px = 2.0` — plausible for HSV, unmeasured.
- `tilt_sigma_rad = 0.0087` (0.5°) — a guess at calibration quality. It is also
  the dominant error term, so it is the worst thing to be guessing about.
- `yaw_variance_floor`, `yaw_rate_variance_floor` — judgement, see §3.3.
- `independent_observations = 10.0` — a plausible number of genuinely
  independent constraints in a frame of line points; not derived.

---

## 8. Validation constraints

The robot is on a desk in a room. There are no field lines, so **none of this
has been validated against a real pitch**, and the field-line node's output
cannot be checked visually at all. Everything above is one of:

- a CPU measurement on the real hardware (§2, §5.1),
- a property derived from the projection geometry (§3),
- a synthetic-data experiment with the derived noise applied (§5.2, §5.3),
- a geometric unit test (33 Python tests in
  [soccer_localization/test](../../ros2_ws/src/soccer_localization/test), 20
  gtest cases in [soccer_perception_gpu/test](../../ros2_ws/src/soccer_perception_gpu/test)).

**Still outstanding for the first time the robot stands on a pitch:**

1. Confirm the tilt and mount height against the real mounting (§7.2).
2. Confirm the HSV fallback actually finds lines under the venue's lighting.
   Its thresholds have never seen grass.
3. Check the observation count per frame is in the tens, not the thousands or
   the zeros.
4. Re-measure `mcl_node` CPU with real observations flowing, which is the first
   time the vectorised update will have run in anger.

---

## 9. `ekf_node` does not know where it is

Separate from all of the above, and found while profiling.

The EKF's state is $[p_x, p_y, \theta, v, \omega]$, but **nothing observes
$v$**. There is no wheel encoder, no leg odometry and no VIO input wired to this
node — only the IMU, which supplies $\theta$ and $\omega$. So $v$ stays at its
initial zero forever and $p_x, p_y$ never move.

Confirmed on the running robot: 1200 consecutive `/odom` samples reported
position exactly (0, 0) and linear velocity exactly 0.0.

The node's docstring always described it as a stand-in for ZED Visual-Inertial
Odometry, so this is not a regression. But it was publishing a pose with an
all-zero covariance, which asserts perfect confidence in a number that means
nothing — the same class of failure as the intrinsics bug. It now publishes
`1e6` variance for `x`, `y` and `vx`, and warns on startup.

**`/odom` translation must not be trusted until a translation source is wired
in.** The MCL's motion model is currently receiving zero translation, which
means it is running open-loop between observations.

---

## 10. Decision log

| Decision | Rationale |
| --- | --- |
| Predict on the IMU callback, publish on a slow timer | The cost was executor wakeups (§2.1), not arithmetic |
| 50 Hz publishing, not 200 Hz | tf2 interpolates; the only consumer runs at 22 Hz |
| Vectorise rather than reduce particle count | 21× for free beats trading accuracy away |
| Metric distance field instead of a baked Gaussian | Fixed σ cannot express range-dependent error |
| Derive σ from geometry, not by fitting | Fitting against a broken projection is what caused the problem |
| 4 m range gate | The range at which σ reaches 0.5 m (§3.2) |
| Floor the IMU covariance rather than trust it outright | The reported figure describes noise, not drift (§3.3) |
| Remove augmented MCL after implementing it | Measured no benefit over a plain explorer fraction (§6.2) |
| Document the field symmetry rather than work around it | It is a property of the map; a workaround would hide it |
| Publish honest covariances everywhere | A confident wrong number is worse than a wide right one |
