// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

//import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

@SuppressWarnings("PropertyName")
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double DEADBAND = 0.15;
  }
  public static final double maxSpeed = 4.7;
  
  public static class ShooterConstants {

    public static final int shooterMotorLeftID = 51;

    // Preferences key (saved on the roboRIO, survives reboots) for a fixed RPM added to EVERY
    // shot map entry. Positive = faster at every distance. Edit it under "Preferences" in Glass.
    public static final String kRpmOffsetKey = "Shooter/RPM Offset";

    // Preferences key for the meters subtracted from the turret-to-hub-center distance before the
    // shot map lookup. The shot map was calibrated by distance to the TAG, and the tag face is
    // 0.604 m (23.77 in) from the hub center (2026 layout), so this is geometry, not a tuning knob.
    // kHubDistanceOffsetDefault is only used the first time the key is created.
    public static final String kHubDistanceOffsetKey = "Shooter/Hub Distance Offset (m)";
    public static final double kHubDistanceOffsetDefault = 0.6;

  }

  /**
   * Shoot on the move: aim at a "virtual hub" shifted against the turret's field velocity by the
   * ball's flight time, so the ball's sideways carry from the robot's motion lands it in the hub.
   */
  public static class ShootOnMoveConstants {
    // Preferences keys (saved on the roboRIO). Turn it off, or scale the lead, without redeploying.
    public static final String kEnabledKey = "ShootOnMove/Enabled";
    public static final String kLeadGainKey = "ShootOnMove/Lead Gain";

    // How far ahead (seconds) the turret aims for the chassis' rotation, to cover the turret
    // position loop's lag and the loop delay while the robot spins.
    public static final double kTurretYawLeadSeconds = 0.08;

    // Ball time of flight (seconds) by distance from the turret to the hub CENTER (meters).
    // PLACEHOLDER values: tune them. Each row's TIME is a Preference ("ShootOnMove/TOF at <d> m (s)"),
    // editable from the dashboard without redeploying. These times are only the first-time defaults;
    // once the keys exist on the roboRIO, the dashboard values win. Lead Gain scales all of them.
    public static final double[][] kTimeOfFlight = {
        {0.76, 0.75},
        {1.80, 0.85},
        {2.74, 0.95},
        {3.34, 1.00},
        {4.00, 1.10},
    };
  }

  /**
   * All turret tunables live here so aiming, limits, and safety can be adjusted in ONE place.
   */
  public static class TurretConstants {
    // --- Hardware ---
    public static final int kTurnMotorID = 52;          // CAN ID of the turret SparkMax
    public static final double kTurretGearTeeth = 200.0; // teeth on the turret ring gear
    public static final double kMotorGearTeeth = 14.0;   // teeth on the motor pinion
    // Motor rotations per one full turret rotation (encoder counts motor shaft rotations).
    public static final double kMotorRotationsPerTurretRotation = kTurretGearTeeth / kMotorGearTeeth;
    // --- HOME + TRAVEL LIMITS (in TURRET rotations) ---
    // The encoder is RELATIVE (in the SparkMax), which reads 0 at power-on, so the turret must
    // be at its HOME end when the robot is POWERED ON. Redeploying does not re-zero it: the
    // SparkMax stays powered and keeps its count. (Or click "Reset - Zero Turret" on the dashboard
    // after putting it at home by hand.)
    // The turret can only move between kMinRotations and kMaxRotations. Enforced in two layers:
    // TurretSubsystem.setAngle() and the SparkMax firmware soft limits.
    // The boot position IS zero. It is the + end of travel, so the turret only moves negative.
    // (In the old hand-taught frame this spot read 86.4 deg and the far end read -120 deg,
    // so the far end is -120 - 86.4 = -206.4 deg from home.)
    // This is the LONG 206.4 deg arc, not the short 153.6 deg way around. Angles are never
    // wrapped to +/-180 anywhere, and going positive past 0 is blocked, so the turret can only
    // ever reach the far end by sweeping the long arc.
    public static final double kHomeRotations = 0.0;
    public static final double kMinRotations = -206.4 / 360.0;
    public static final double kMaxRotations = 0.0;

    // --- Closed-loop position PID (SparkMax, in motor rotations) ---
    public static final double kP = 0.15;
    // Keep I at 0. The SparkMax adds kI * error EVERY 1 ms, with error in MOTOR rotations
    // (14.3 per turret rotation). 0.001 built up to full output within a second of a big move
    // and flung the turret past its target. If I is ever needed, also set an iZone.
    public static final double kI = 0.0;
    public static final double kD = 0.002;
    public static final int kSmartCurrentLimitAmps = 40;

    // --- MOUNTING GEOMETRY (used by turretAim and the turret camera) ---
    // Positive turret angle turns the turret CLOCKWISE seen from above, so travel from home
    // (0 -> -206.4 deg) is counter-clockwise. At home the shooter points ~3 deg LEFT of straight
    // at the robot's right (toward the front). Robot-relative shooter heading (CCW-positive,
    // 0 = robot front) is therefore:
    //   heading = kHomeHeadingDegrees - turretAngle
    // (0 deg = 3 deg left of right, -87 deg = front, -177 deg = left, -206.4 deg = past left.)
    // Turret aiming barely depends on this (it cancels out of the camera math); it mainly sets
    // which hub directions count as reachable and what the dashboard shows.
    public static final double kHomeHeadingDegrees = -87.0;
    // Turret rotation axis relative to robot center (x = forward, y = left), METERS.
    // Measured: 7 in BEHIND robot center, centered left/right.
    public static final Translation2d kRobotToTurret = new Translation2d(Units.inchesToMeters(-7.0), 0.0);
    // The indexer only feeds when the turret is within this many degrees of the hub.
    public static final double kAimToleranceDegrees = 5.0;

    // --- Vision aiming ---
    // Every AprilTag on each hub (all four faces, 2026 field layout). Which list applies is
    // decided at the moment of use by SelectHub.isOurHubTag(), NOT here: a constant computed
    // at class load would lock in whatever alliance the DS had at boot (and crash if none).
    public static final int[] kRedHubTagIds = {2, 3, 4, 5, 8, 9, 10, 11};
    public static final int[] kBlueHubTagIds = {18, 19, 20, 21, 24, 25, 26, 27};
  
  }
  /**
   * Everything the PhotonVision-based {@code VisionSubsystem} needs.
   * This robot runs TWO cameras; each one has its own name and mounting position.
   */
  public static class VisionConstants {
    // --- CAMERA NAMES ---
    // These MUST exactly match the camera "nicknames" you set in the PhotonVision web UI
    // (http://photonvision.local:5800). If they don't match, PhotonVision returns no data.
    // TODO: replace with the real nicknames of your two cameras.
    public static final String kCameraLeftName = "Thriftyfront";
    public static final String kCameraRightName = "ThriftyTurret";

    // --- CAMERA MOUNTING (robot-to-camera transforms) ---
    // Where each camera sits RELATIVE TO THE ROBOT'S CENTER.
    // Translation is (x = forward, y = left, z = up) in METERS.
    // Rotation is (roll, pitch, yaw) in RADIANS: pitch up is negative, yaw left is positive.
    // These must be accurate or vision poses will be wrong.
    // Front camera, measured on the 25x25 in frame: 12 in back from the front edge, 2.5 in in from
    // the right edge, lens 24.5 in off the floor, level (no tilt), turned 30 deg left of straight
    // ahead (aiming front-left).
    public static final Transform3d kRobotToCameraLeft = new Transform3d(
        new Translation3d(Units.inchesToMeters(0.5), Units.inchesToMeters(-10.0), Units.inchesToMeters(24.5)),
        new Rotation3d(0.0, 0.0, Units.degreesToRadians(30.0)));
    // The turret camera rides ON the turret, facing the same way as the shooter. This is where
    // it sits relative to the turret's rotation axis (TurretConstants.kRobotToTurret) with the
    // turret pointing robot-forward. VisionSubsystem rotates it by the live turret angle for
    // every frame. It is tilted UP 21.15 deg, which is NEGATIVE pitch here.
    // Measured: the lens is 6 in forward of the turret's rotation axis (toward where the shooter points).
    public static final Transform3d kTurretToCamera = new Transform3d(
        new Translation3d(Units.inchesToMeters(6), Units.inchesToMeters(0), Units.inchesToMeters(25)),
        new Rotation3d(0.0, Units.degreesToRadians(-21.15), Units.degreesToRadians(0)));

    // --- MEASUREMENT TRUST (standard deviations) ---
    // How much to trust a vision pose: [x meters, y meters, theta radians].
    // Only the FRONT camera feeds the robot pose; the turret camera never does.
    // SMALLER = trust vision MORE. A single tag is jitterier than several, so it's trusted a bit
    // less. Heading (theta) is effectively NOT taken from vision: the Pigeon is far more accurate.
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(0.5, 0.5, 9999.0);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.3, 0.3, 9999.0);

    // A single tag can "flip" to a mirror-image solution. Skip single-tag frames whose
    // ambiguity is above this (0 = certain, 1 = coin flip).
    public static final double kMaxSingleTagAmbiguity = 0.2;

    // --- Turret camera fix filtering (VisionSubsystem.updateTurretFix) ---
    // The turret camera usually sees one small tag, so its heading jumps 20-80 deg on some
    // frames. Instead of taking every frame as the new truth:
    // Looser ambiguity limit than the front camera: the heading check below catches the flips.
    public static final double kTurretMaxTagAmbiguity = 0.4;
    // Reject a frame whose robot heading differs from the gyro-tracked heading by more than this.
    public static final double kTurretMaxHeadingJumpDegrees = 15.0;
    // Fraction of the camera's correction applied per accepted frame (1.0 = old behavior).
    public static final double kTurretHeadingBlend = 0.2;
    public static final double kTurretPositionBlend = 0.3;
    // After this many rejected frames IN A ROW, trust the camera outright (the tracked
    // heading is probably the wrong one). Tags are often only seen on a fraction of frames,
    // so keep this small: at 25 it took ~10 s to recover from a bad fix outdoors.
    public static final int kTurretResyncFrames = 10;
    // If the last accepted fix is older than this, the tracked pose has been dead-reckoning
    // too long to judge a new frame by: take the new frame outright.
    public static final double kTurretFixStaleSeconds = 1.0;

    // --- Camera cross-check (VisionSubsystem) ---
    // Each camera's turret fix is compared as a residual against the gyro + wheel track at the
    // frame's own time, so robot motion cancels out. The other camera's residual counts if it is
    // at most this old.
    public static final double kCrossCheckWindowSeconds = 0.3;
    // The two cameras "agree" when their residuals differ by less than these.
    public static final double kCrossCheckMaxMeters = 0.25;
    public static final double kCrossCheckMaxDegrees = 5.0;

    // Ignore any estimate whose average tag distance is beyond this (meters).
    public static final double kMaxAverageTagDistanceMeters = 4.0;
  }
  public static class IndexerConstants {
    // TEMPORARY: set false to skip the indexer Kraken (TalonFX 54) entirely while it isn't
    // answering on CAN. The feeder SparkMax still runs. Set back to true once it's fixed.
    public static final boolean kEnableIndexerTalon = true;
  }

  public static class IntakeConstants {

    // TEMPORARY: set false to skip the intake roller Kraken (TalonFX 56) entirely while it
    // isn't answering on CAN. The arm SparkMax still runs. Set back to true once it's fixed.
    public static final boolean kEnableIntakeRollerTalon = true;

    public static final double gearRatio = 25*32 / 12;
    public static final double IntakeDownPosition = -0.25;
    public static final double IntakeUpPosition = 0.00;

    // RoboRIO DIO port for the arm's bottom limit switch. Wired to ground, so
    // the input reads low (false) when the arm is at the bottom of its travel.
    public static final int ArmLowerLimitSwitchPort = 0;

    // How close (in output rotations) the arm must get before a move is
    // considered finished.
    public static final double ArmPositionTolerance = 0.01;

    // Safety cap on a single arm move, so a stalled or blocked arm can never
    // hang a command group forever.
    public static final double ArmMoveTimeoutSeconds = 1.0;

    // TEMPORARY: open-loop speed for the manual D-pad jog used to bench-test the
    // limit switch. Keep this slow.
    public static final double ArmJogSpeed = 1.0;

  }

  public static class HoodConstants {
    // TEMPORARY: set false to skip the hood SparkMax (CAN 50) entirely while it isn't
    // answering on CAN. Hood commands become no-ops. Set back to true once it's fixed.
    public static final boolean kEnableHoodMotor = true;

    // The real hood reduction is about 10:1, but we deliberately leave it at 1 so every hood
    // position (clamps, setpoints, the PrepareShot tree map, "Hood Position" on the dashboard)
    // is in raw MOTOR rotations. That keeps the tuned tree map numbers direct.
    public static final double gearRatio = 1;
    public static final double HoodDownPosition = 0;
    public static final double HoodUpPosition = 0.6;
  }
}
