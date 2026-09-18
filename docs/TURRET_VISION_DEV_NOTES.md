# Turret + Vision Development Notes

_Last updated: 2026-07-28. Branch: `main`._

This documents the turret aiming, PhotonVision, and dashboard work: what was tried,
what failed, what worked, and what still needs doing. Written so anyone (or future
you) can pick this up without re-deriving the afternoon of debugging.

---

## Current state (what's in the code now)

**Build status:** Compiles clean against the 2026 WPILib + vendordep toolchain
(verified with `gradlew compileJava`). Only 3 non-blocking warnings: PhotonVision's
`PhotonPoseEstimator` constructor / `setMultiTagFallbackStrategy` / `update` are
deprecated-for-removal. They still work; see TODO below.

**Controls (`RobotContainer`):**
| Control | Action |
|---|---|
| Left stick / right stick | Field-oriented swerve drive |
| **Y** | Zero gyro |
| **B** (hold) | Lock wheels (X pattern) |
| **A** (hold) | Turret aim (`turretAim` command) |
| **Right trigger** | Spin shooter (`SpinShooter`, 3000 RPM) |

Hood, Intake, Indexer subsystems exist but are **not instantiated** in
`RobotContainer` — they are dormant and touch nothing on A / right-trigger.

---

## VisionSubsystem (PhotonVision) — WORKING

- Two cameras: `Thriftyfront` (front, chassis-fixed) and `ThriftyTurret` (on the turret).
  Names must match the PhotonVision UI exactly (`VisionConstants.kCameraLeftName/RightName`).
- Each camera has its own `PhotonPoseEstimator` (MULTI_TAG_PNP + LOWEST_AMBIGUITY fallback).
- **Front camera feeds the swerve pose estimator; the turret camera is EXCLUDED from pose
  estimation** (its transform is invalid while the turret rotates). Turret camera is used
  for aiming only.
- Target getters for the aim command: `getTargets(name)`, `getTurretCameraTargets()`,
  `getLatestTargets()`.
- Publishes per-camera targeting telemetry to `Vision/<cameraName>/...` and overall
  `Vision/HasTarget`, etc.

**Confirmed working on the robot:** vision sees tags (`Turret Tag Yaw` showed ~8.5° with a
tag in view).

---

## TurretSubsystem — WORKING (with caveats)

### Home on start (replaced the hand-taught "Set Clamp" clamps)
- The turret must be sitting at its **home end** when the robot powers on / code restarts.
  That position is **0°**. It is the + end of travel; the turret only moves **negative**, down
  to **-206.4°** (the LONG arc, not the short 153.6° way). Values in
  `TurretConstants.kHomeRotations / kMinRotations / kMaxRotations`.
- (Old hand-taught frame for reference: home read 86.4°, far end -120°.)
- If the code restarted with the turret elsewhere: put it at home by hand and click
  **`Re-home Turret`** on the dashboard (works while disabled).
- Motor is **coast** idle mode so it can be moved by hand. The relative encoder keeps
  counting while powered, so hand movement doesn't lose the zero; only a reboot does.
- Two-layer travel protection: `setAngle()` software clamp **and** SparkMax firmware soft
  limits at the same 0° / -206.4° range.

### Aiming math (`turretAim`)
- **Counter-rotation debt accumulator (Pigeon IMU):** each loop accumulates the chassis
  yaw change and only drains it by how far the turret _actually_ moved. This keeps the
  position error high until the turret catches up (fixes the "never counter-rotates" bug).
- **Vision:** full proportional correction on the target tag's yaw (no gain — re-anchored
  to the measured angle each loop, the form that settled cleanly).
- **No wrap-around:** a target past a travel limit just holds at that limit; angles are
  never wrapped to ±180°, so the turret always travels the long arc.

### Hardware
- SparkMax (confirmed — **not** SparkFlex), CAN ID `TurretConstants.kTurnMotorID`.
- Gear ratio `200/14` (`kMotorRotationsPerTurretRotation`).
- Closed-loop position PID: `kP = 0.12` (`TurretConstants`). **`setSetpoint` is correct
  for REVLib 2026** (NOT `setReference` — that's the old pre-2026 API).

---

## What was ATTEMPTED and FAILED

### Dashboard: showing turret status in "one box"
Tried several ways to group the clamp status + button into a single Glass widget. **All
of these failed to appear in Glass:**
1. `SmartDashboard.putData("Turret Status", this)` — publishing the **subsystem itself**
   as a Sendable. Collides with `SubsystemBase`'s LiveWindow registration → silently never
   published.
2. Standalone `Sendable` object with `initSendable` — also did not show up reliably.
3. Writable-boolean "Set Clamp" trigger inside a Sendable — same problem.

### The counter-rotation "always works" misconception
- The **1:1 ratio is exact** (no gain to tune). But at high spin it loses the tag due to
  **position-loop velocity lag** (`error ≈ speed / kP`), latency, and the turret's physical
  slew limit. A velocity feedforward (with a tuned constant) would largely fix the lag;
  raising `kP` helps. NOT yet implemented.

### `setReference` vs `setSetpoint`
- Briefly changed all closed-loop calls to `setReference` — **WRONG** for REVLib 2026.
  Reverted. `setSetpoint`/`getSetpoint` are correct in the 2026 API.

---

## What WORKED

- **`SmartDashboard.putData("Set Clamp", Commands.runOnce(...).ignoringDisable(true))`** —
  a **Command** widget (Set Clamp is gone now; `Re-home Turret` uses the same pattern). This reliably shows in Glass under `NetworkTables → SmartDashboard`.
  Confirmed on the robot ("THAT WORKED. i can see the command").
- **Plain `SmartDashboard.putNumber/putString/putBoolean` in `periodic()`** — reliable.
  These publish to the `SmartDashboard` NT table and show in Glass's **NetworkTables tree
  view** (Sources), NOT in the widget menu (see below).

---

## KEY LESSON: where things show up in Glass

This caused hours of "it's not showing up":

- **Sendable widgets** (a `Command`, `SendableChooser`, `Gyro`, `Field2d`) appear in the
  **`NetworkTables` menu** (the dropdown submenu). This is why `Set Clamp` (a Command) and
  the old degree `SendableChooser` showed there.
- **Plain values** (`putNumber`/`putString`/`putBoolean`, e.g. all the `Turret/...` keys)
  do **NOT** appear in that menu. They live in the **NetworkTables View (tree / Sources
  panel)**. Open `NetworkTables → NetworkTables View`, expand `SmartDashboard → Turret`,
  and drag keys onto the workspace.

A `Turret/BUILD = "diagnostics-build-A"` marker key was added (published in the constructor)
as a deploy sanity-check: if you see it in the tree, the new code is running.

---

## KEY LESSON: WSL + Windows deploy gotcha

Files are edited from WSL (`/mnt/c/...`) but built/deployed from Windows. Two traps:
1. **Gradle "UP-TO-DATE" caching** — WSL edits don't always bump timestamps Windows Gradle
   notices, so it skips recompiling and **redeploys stale bytecode**. Fix: delete `build/`
   (or `gradlew clean deploy`) to force a full recompile.
2. **VS Code editor buffer** — if the file is open in VS Code with an older copy, deploying
   from VS Code can save that stale buffer over the disk edits. Fix: **File → Revert File**
   before deploying, or deploy from a terminal (`gradlew deploy`), not VS Code.

**Recommended deploy flow after any edit from WSL:**
```
# from the project folder on Windows
gradlew clean deploy
```
Then verify `Turret/BUILD` appears in the Glass NetworkTables tree.

Note: WSL2 cannot reach the roboRIO (no network route), so deploy must run from Windows.
Compiling to _verify code_ can be done from WSL via the Windows JDK:
`/mnt/c/Windows/System32/cmd.exe /c "gradlew.bat compileJava --offline --rerun-tasks"`.

---

## TODO / open items

- [ ] **Verify the turret diagnostics actually appear** in the Glass NetworkTables tree
      (`SmartDashboard → Turret`) after a clean deploy. Look for `Turret/BUILD` first.
- [ ] **Set real values** in `Constants.TurretConstants`: `kTurnMotorID`, and confirm the
      `200/14` gear ratio. Camera names + `kRobotToCamera*` transforms in `VisionConstants`.
- [x] **Turret boot position:** home-on-start replaces hand-taught clamps (boot = 0°,
      travel to -206.4°). Still relies on the drive team putting it at home before power-on;
      an absolute encoder or limit switch would remove that step.
- [ ] **Confirm counter-rotation SIGN on the robot.** If the turret moves the wrong way vs
      chassis spin, flip the sign in `turretAim` (the `pendingChassisRotations` term).
- [ ] **High-speed tracking:** consider a velocity feedforward (tuned `kV`-style constant)
      and/or higher `kP` so the turret holds the tag during fast spins. Lower camera
      exposure / higher FPS to reduce motion blur.
- [ ] **PhotonVision deprecation:** migrate off the deprecated `PhotonPoseEstimator`
      constructor / `setMultiTagFallbackStrategy` / `update(result)` before a future
      photonlib version removes them.
- [ ] Remove the `Turret/BUILD` marker once deployment is confirmed reliable.

---

## Relevant commit history (main)

- `ea176e3` Turret clamp: hand-taught clamps + dashboard status, side-selection
- `b7a2c42` Turret aim: debt accumulator, full vision, wrap-around
- `226c987` Fix turret counter-rotation: accumulate the target
- `a1aa8b2` Turret: centralize config, two-layer clamp, IMU counter-rotation
- `dc07581` Turret: proportional yaw aim + vision target getters
- `d2a4e65` Merge PR #8 (vision → main)

_Uncommitted at time of writing: `TurretSubsystem.java` (motor diagnostics + `Turret/BUILD`
marker)._
