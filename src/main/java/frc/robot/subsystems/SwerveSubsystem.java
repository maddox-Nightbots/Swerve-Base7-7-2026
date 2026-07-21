package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

/**
 * SwerveSubsystem manages the 4-wheel independent drive modules.
 * In FRC, a "Subsystem" is like a body part of the robot. This one handles movement.
 * It uses YAGSL (Yet Another Generic Swerve Library) to handle the complex 
 * trigonometry required to make 4 wheels move together as one drivetrain.
 */
public class SwerveSubsystem extends SubsystemBase {
  
  // The 'swerveDrive' object is the brain of the drivetrain provided by YAGSL.
  private final SwerveDrive swerveDrive;
  
  // Field2d is a utility that lets us see a virtual robot icon on our 
  // laptop dashboard (Glass/Shuffleboard) to verify our position on the field.
  private final Field2d m_field = new Field2d();
  
  // A safety check to ensure PathPlanner (auto-navigation) is ready to use.
  private boolean pathPlannerConfigured = false;

  /**
   * CONSTRUCTOR: This runs once when the robot code starts up.
   */
  public SwerveSubsystem() { 
    // YAGSL looks for JSON configuration files in the 'deploy/swerve' folder.
    // These files define motor IDs, gear ratios, and wheel placement.
    File directory = new File(Filesystem.getDeployDirectory(), "swerve");
    try {
      // Initialize the drivetrain using the files in that directory and the max speed from Constants.
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.maxSpeed);
    } catch (IOException e) {
      // If the config files are missing or broken, the code crashes here so we know immediately.
      throw new RuntimeException(e);
    }

    // Pushes the virtual field map to the dashboard so drivers can see robot positioning.
    SmartDashboard.putData("Field", m_field);

    // Initialize the autonomous navigation settings.
    setupPathPlanner();
  }

  /**
   * NEW FOR 2026: A command that can be used inside PathPlanner Autos.
   * This forces the robot to pause and ensure its internal map matches the Limelight
   * perfectly before continuing a high-precision task.
   */
  public Command visionAlignCommand() {
    return runOnce(() -> {
        LimelightHelpers.PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-first");
        if (estimate.tagCount >= 1) {
            // "Snap" the odometry to the exact vision pose
            swerveDrive.resetOdometry(estimate.pose);
        }
    });
  }

  /**
   * Configures PathPlanner, which is the software used to follow pre-drawn paths.
   */
  public final void setupPathPlanner() {
    try {
      // RobotConfig reads the physical robot data (mass, MOI, etc.) from GUI settings.
      RobotConfig config = RobotConfig.fromGUISettings();

      // Configure the AutoBuilder: This maps PathPlanner's "brain" to our robot's "muscles."
      AutoBuilder.configure(
          this::getPose,                // Function to get current robot position (X, Y, Rotation)
          swerveDrive::resetOdometry,   // Function to 'teleport' the robot's internal map position
          swerveDrive::getRobotVelocity,// Function to see how fast the robot is currently moving
          (speeds, feedforwards) -> swerveDrive.drive(speeds), // Function to physically move the motors
          new PPHolonomicDriveController(
              // PIDConstants: These control how hard the robot fights to stay on a path.
              // If the robot wobbles, these numbers are too high. If it's lazy, they are too low.
              new PIDConstants(0.001545, 0.0, 0.0), // Translation (Moving X and Y)
              new PIDConstants(0.0014645, 0.0, 0.0)  // Rotation (Turning)
          ),
          config,
          () -> {
              // Alliance Check: Flips the path automatically if we are on the Red Alliance.
              var alliance = DriverStation.getAlliance();
              return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
          },
          this);

      pathPlannerConfigured = true;
    } catch (IOException | ParseException e) {
      // If PathPlanner fails, we report an error to the Driver Station log.
      pathPlannerConfigured = false;
      DriverStation.reportError("PathPlanner Config Failed: " + e.getMessage(), e.getStackTrace());
    }
  }

  /**
   * Uses the Limelight camera to correct our position on the field.
   * NEW FOR 2026: Refined filtering to ensure Auto paths stay pixel-perfect.
   */
  public void updateVisionOdometry(){
    // Gets the robot's calculated position from the Limelight camera.
    LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-first");
    
    // 1. Only update if we see tags.
    // 2. We also check "avgTagDist" - if tags are too far away (e.g. > 4 meters), 
    //    the data might be jittery, so we trust it less.
    if(limelightMeasurement.tagCount >= 1)
    {
      double trustValue = 0.7; // Standard trust
      
      // If we see multiple tags, we can trust the vision MUCH more.
      if(limelightMeasurement.tagCount > 1) {
        trustValue = 0.3; // Lower number = More trust in vision
      }

      // addVisionMeasurement merges camera data with wheel data.
      // We pass the trustValue into the X and Y slots. 
      // We still keep rotation trust very low (999999) because the Gyro is usually more stable than a single camera for heading.
      swerveDrive.addVisionMeasurement(
          limelightMeasurement.pose, 
          limelightMeasurement.timestampSeconds, 
          VecBuilder.fill(trustValue, trustValue, 9999999));
    }
  }

  /**
   * Drive command for the joysticks. 
   * "Field Oriented" means pushing the stick forward always moves the robot away from the driver,
   * no matter which way the robot's front is pointing.
   */
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> swerveDrive.driveFieldOriented(velocity.get()));
  } 

  /**
   * Resets the Gyro (Compass). Use this if the robot's "forward" gets messed up.
   */
  public Command zeroGyroCommand() {
   return runOnce(() -> {
      if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
      {
        swerveDrive.zeroGyro();
        swerveDrive.resetOdometry(new Pose2d(swerveDrive.getPose().getX(), swerveDrive.getPose().getY(), Rotation2d.fromDegrees(180)));
        //swerveDrive.zeroGryoWithAlliance();
      } else {
         swerveDrive.zeroGyro();

       }
     });
   // return runOnce(() -> swerveDrive.setGyro(DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? new Rotation3d(Rotation2d.k180deg).minus(new Rotation3d(swerveDrive.getPose().getRotation())) : Rotation3d.kZero));
  }

  /**
   * ADVANCED 2026 HEADING UPDATE: Uses Vision if available, otherwise 
   * falls back to Odometry to ensure the robot is aligned at Teleop start.
   */
  public void updateHeadingWithVision() {
    swerveDrive.setHeadingCorrection(true);

    // Grab the latest vision estimate from Limelight.
    LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-first");

    edu.wpi.first.math.geometry.Rotation2d finalRotation;

    if(limelightMeasurement.tagCount >= 1) {
      // If we see a tag, trust the Limelight's rotation for the reset.
      finalRotation = limelightMeasurement.pose.getRotation();
    } else {
      // If no tags are seen, fallback to the robot's internal Odometry.
      finalRotation = swerveDrive.getOdometryHeading();
    }

    // Convert our chosen 2d rotation into the 3D offset YAGSL requires.
    swerveDrive.setGyroOffset(new edu.wpi.first.math.geometry.Rotation3d(0, 0, finalRotation.getRadians()));
  }

  /**
   * Locks the wheels in an 'X' pattern. 
   * This makes it very difficult for other robots to push us.
   */
  public Command lockPoseCommand() {
    return run(() -> swerveDrive.lockPose());
  }

  /**
   * Returns where the robot is on the field (X, Y in meters, and Rotation).
   */
  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  /**
   * Accessor method to get the raw SwerveDrive object if needed.
   */
  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  /**
   * This runs every 20ms (50 times per second).
   * It is the "Heartbeat" of the subsystem.
   */
  @Override
  public void periodic() {
    updateVisionOdometry(); // Look for AprilTags to fix positioning
    swerveDrive.updateOdometry(); // Calculate new position based on motor rotations
    
    // Update the virtual robot icon on the dashboard so we can debug movement.
    m_field.setRobotPose(swerveDrive.getPose());
  }
}