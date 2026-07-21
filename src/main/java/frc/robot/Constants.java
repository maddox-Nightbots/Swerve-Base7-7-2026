// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double DEADBAND = 0.15;
  }

  public static class ShooterConstants {
    // TODO: set to the real CAN ID of the shooter flywheel SparkFlex.
    public static final int shooterMotorLeftID = 0;
    // TODO: set to the real PWM channels for the hood actuator servos.
    public static final int actuatorLeftServo = 0;
    public static final int actuatorRightServo = 1;
  }

  public static final double maxSpeed = 4.7;
}
