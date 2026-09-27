// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

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
    // The encoder is RELATIVE, so the code assumes the turret is sitting at its HOME end when
    // the robot boots and seeds the encoder to kHomeRotations. ALWAYS turn the turret to the
    // home end before powering on / redeploying (or click "Re-home Turret" on the dashboard
    // after putting it there by hand).
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
    public static final double kP = 0.18;
    // Keep I at 0. The SparkMax adds kI * error EVERY 1 ms, with error in MOTOR rotations
    // (14.3 per turret rotation). 0.001 built up to full output within a second of a big move
    // and flung the turret past its target. If I is ever needed, also set an iZone.
    public static final double kI = 0.0001;
    public static final double kD = 0.001;
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
    // TODO: measure on the real robot.
    public static final Translation2d kRobotToTurret = new Translation2d(0.0, 0.0);
    // The indexer only feeds when the turret is within this many degrees of the hub.
    public static final double kAimToleranceDegrees = 5.0;

    // --- Vision aiming ---
    final static Optional<Alliance> alliance = DriverStation.getAlliance();
    public static final int kHubTagId = (alliance.get() == Alliance.Red ? 10 : 26); // AprilTag the turret aims at
    public static final int kTrenchLeftTagId = (alliance.get() == Alliance.Red ? 6 : 6); // AprilTag the turret aims at
    public static final int kTrenchRightTagId = (alliance.get() == Alliance.Red ? 6 : 6); // AprilTag the turret aims at
  
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
    // These must be accurate or vision poses will be wrong. TODO: measure on the real robot.
    public static final Transform3d kRobotToCameraLeft = new Transform3d(
        new Translation3d(Units.inchesToMeters(10.0), Units.inchesToMeters(10.0), Units.inchesToMeters(8.0)),
        new Rotation3d(0.0, Units.degreesToRadians(-20.0), Units.degreesToRadians(30.0)));
    // The turret camera rides ON the turret, facing the same way as the shooter. This is where
    // it sits relative to the turret's rotation axis (TurretConstants.kRobotToTurret) with the
    // turret pointing robot-forward. VisionSubsystem rotates it by the live turret angle for
    // every frame. It is tilted UP 21.15 deg, which is NEGATIVE pitch here.
    public static final Transform3d kTurretToCamera = new Transform3d(
        new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(25)),
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
    public static final double IntakeDownPosition = -0.29;
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
