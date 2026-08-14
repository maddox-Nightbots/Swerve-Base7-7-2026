// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
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

    public static final int actuatorLeftServo = 0;
    public static final int actuatorRightServo = 0;
    public static final int shooterMotorLeftID = 2;
    public static final int TurnMotorID = 1;
    public static final double gearRatio = 1;

  }

  /**
   * All turret tunables live here so aiming, limits, and safety can be adjusted in ONE place.
   */
  public static class TurretConstants {
    // --- Hardware ---
    public static final int kTurnMotorID = 1;          // CAN ID of the turret SparkMax
    public static final double kTurretGearTeeth = 200.0; // teeth on the turret ring gear
    public static final double kMotorGearTeeth = 14.0;   // teeth on the motor pinion
    // Motor rotations per one full turret rotation (encoder counts motor shaft rotations).
    public static final double kMotorRotationsPerTurretRotation = kTurretGearTeeth / kMotorGearTeeth;

    // --- SAFETY: absolute turret backstop (in TURRET rotations, symmetric about boot position) ---
    // The wiring on the moving part of the turret CANNOT wind past this without damage. This is
    // the WIDEST the turret can ever go; the hand-taught clamps operate inside it. Enforced in
    // two layers: TurretSubsystem.setAngle() and the SparkMax firmware soft limits.
    // WARNING: the encoder is RELATIVE, so this window is measured from wherever the turret sat
    // at boot. Home the turret before enabling, or move to an absolute encoder.
    // TODO: set this to the real wire-safe range measured on the robot.
    public static final double kBackstopRotations = 180.0 / 360.0; // 0.5 rot = +/-180 deg

    // Smallest allowed spread between the two taught clamps. Capturing both at nearly the same
    // spot would pin the turret in place, which reads as "broken" on the field.
    public static final double kMinClampSpreadRotations = 5.0 / 360.0; // 5 deg

    // --- Closed-loop position PID (SparkMax, in motor rotations) ---
    public static final double kP = 0.12;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final int kSmartCurrentLimitAmps = 40;

    // --- Vision aiming ---
    final static Optional<Alliance> alliance = DriverStation.getAlliance();
    public static final int kTargetTagId = (alliance.get() == Alliance.Red ? 6 : 6); // AprilTag the turret aims at
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
    public static final Transform3d kRobotToCameraRight = new Transform3d(
        new Translation3d(Units.inchesToMeters(10.0), Units.inchesToMeters(-10.0), Units.inchesToMeters(8.0)),
        new Rotation3d(0.0, Units.degreesToRadians(-20.0), Units.degreesToRadians(-30.0)));

    // --- MEASUREMENT TRUST (standard deviations) ---
    // How much to trust a vision pose: [x meters, y meters, theta radians].
    // SMALLER = trust vision MORE. A single tag is jittery; multiple tags triangulate
    // a solid pose, so we trust them much more.
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4.0, 4.0, 8.0);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1.0);

    // Ignore any estimate whose average tag distance is beyond this (meters).
    public static final double kMaxAverageTagDistanceMeters = 4.0;
  }
  public static class IntakeConstants {

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
    public static final double ArmJogSpeed = 0.10;

  }

  public static class HoodConstants {
    public static final double gearRatio = 5;
    public static final double HoodDownPosition = 0.25;
    public static final double HoodUpPosition = 0.1;
  }
}
