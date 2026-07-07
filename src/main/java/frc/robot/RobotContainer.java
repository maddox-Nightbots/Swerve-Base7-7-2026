package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder; // New Import

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; // New Import
import edu.wpi.first.wpilibj2.command.Command; // New Import
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.OLEDPongSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

/**
 * RobotContainer is where the bulk of the robot members are declared.
 * This class ties subsystems, controllers, and button bindings together.
 */
public class RobotContainer {

  // 1. SUBSYSTEMS: Creating the "Body Parts"
  // We create an instance of SwerveSubsystem so we can tell the drivetrain what to do.
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();

  private final SendableChooser<Command> autoChooser;

<<<<<<< Updated upstream
  //OLED Pong - Fun extra, disabled if causing issues
  // Set ENABLE_OLED_PONG to false to completely disable
  private static final boolean ENABLE_OLED_PONG = true;
  private final OLEDPongSubsystem pongSubsystem = ENABLE_OLED_PONG ? new OLEDPongSubsystem() : null;
=======
    // OLED Pong - Fun extra, disabled if causing issues
    // Set ENABLE_OLED_PONG to false to completely disable
    private static final boolean ENABLE_OLED_PONG = true;
    private final OLEDPongSubsystem pongSubsystem = ENABLE_OLED_PONG ? new OLEDPongSubsystem() : null;
>>>>>>> Stashed changes

  // 2. CONTROLLERS: Defining the Inputs
  // CommandXboxController is a wrapper that makes it easy to link buttons to commands.
  // kDriverControllerPort is usually '0' (set in Constants.java).
  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    // Register Named Commands for PathPlanner
    // This allows you to drag a "Wait and Align" event into your path in the GUI.
    com.pathplanner.lib.auto.NamedCommands.registerCommand("VisionAlign", m_swerveSubsystem.visionAlignCommand());

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    configureBindings();
  }

  /**
   * NEW FOR 2026: This logic ensures that as soon as Teleop begins,
   * the robot's heading is aligned with its last known position from Auto.
   * It uses Vision (Limelight) if a tag is visible for maximum precision.
   */
  public void setupTeleopHeading() {
    m_swerveSubsystem.updateHeadingWithVision();
  }

  /**
   * BINDINGS: This is where we define how the robot reacts to the controller.
   */
  private void configureBindings() 
  {
    // --- DRIVING LOGIC ---
    
    // SwerveInputStream is a YAGSL tool that "cleans up" joystick data.
    // It handles things like deadbands (so the robot doesn't drift if the stick is loose)
    // and scaling (so the robot isn't too twitchy).
    SwerveInputStream driveInputStream = SwerveInputStream.of(
        m_swerveSubsystem.getSwerveDrive(), 
        // Forward/Backward (Y-Axis). Note: Up on the stick is usually negative, so we multiply by 1 or -1 if needed.
        () -> m_driverController.getLeftY() * -1,
        // Left/Right Strafe (X-Axis).
        () -> m_driverController.getLeftX() * -1) 

        // Rotation: We use the Right Stick to spin the robot. 
        // We multiply by -1 here because usually "Right" on the stick should be "Clockwise."
        .withControllerRotationAxis(() -> m_driverController.getRightX() * -1)
        
        // Deadband: If the stick is pushed less than X% (e.g., 0.1), ignore it. 
        // This prevents "stick drift" where the robot moves even when you aren't touching it.
        .deadband(OperatorConstants.DEADBAND)
        
        // Scale: Reduces speed to make the robot easier to control for the driver.
        .scaleTranslation(1.0)
        
        // Alliance Relative: If true, "Forward" is always away from the driver's wall,
        // regardless of which way the robot is facing.
        .allianceRelativeControl(true);

    // --- ASSIGNING COMMANDS TO BUTTONS ---

    // SET DEFAULT COMMAND: This tells the robot "If I am not pressing any other buttons, do THIS."
    // In this case, the robot should always be listening to the joysticks to drive.
    m_swerveSubsystem.setDefaultCommand(m_swerveSubsystem.driveFieldOriented(driveInputStream));

    // Y BUTTON: Resets the Gyro. 
    // If the robot's "Forward" direction gets confused, the driver points the robot 
    // away from them and presses Y to reset the "North" star.
    m_driverController.y().onTrue(m_swerveSubsystem.zeroGyroCommand());

    // B BUTTON: "Parking Brake" / Lock Pose.
    // While the driver holds B, the wheels turn into an 'X' shape so the robot cannot be pushed.
    m_driverController.b().whileTrue(m_swerveSubsystem.lockPoseCommand());
  }

  /**
   * AUTONOMOUS: This method is called by the main Robot.java file when the 15-second 
   * auto period starts.
   * * @return The command to run (in this case, the path named "Test Auto" from PathPlanner).
   */
  public Command getAutonomousCommand()
  {
    // This tells the SwerveSubsystem to look for the selected auto 
    // created in the PathPlanner GUI and execute those movements.
    return autoChooser.getSelected();
  }

    public SwerveSubsystem getM_swerveSubsystem() {
        return m_swerveSubsystem;
    }
}