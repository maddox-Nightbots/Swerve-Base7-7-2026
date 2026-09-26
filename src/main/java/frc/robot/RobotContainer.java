package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder; // New Import
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; // New Import
import edu.wpi.first.wpilibj2.command.Command; // New Import
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ShootSequence;
import frc.robot.commands.SpinShooter;
import frc.robot.commands.turretAim;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OLEDPongSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import swervelib.SwerveInputStream;

/**
 * RobotContainer is where the bulk of the robot members are declared.
 * This class ties subsystems, controllers, and button bindings together.
 */
public class RobotContainer {
  private static final double RIGHT_TRIGGER_THRESHOLD = 0.5;
  public static Boolean shooting = false;

  public double targetHoodPosition = 0.0;

  // 1. SUBSYSTEMS: Creating the "Body Parts"
  // We create an instance of SwerveSubsystem so we can tell the drivetrain what to do.
  public final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();

  public final TurretSubsystem m_TurretSubsystem = new TurretSubsystem();
  public final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
    public final IndexerSubsystem m_IndexerSubsystem = new IndexerSubsystem();
  public final VisionSubsystem m_VisionSubsystem = new VisionSubsystem(m_swerveSubsystem);
  public final SpinShooter m_spinShooterCommand = new SpinShooter(m_ShooterSubsystem, m_IndexerSubsystem);
  public final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  public final HoodSubsystem m_HoodSubsystem = new HoodSubsystem();


  private final SendableChooser<Command> autoChooser;


    // OLED Pong - Fun extra, disabled if causing issues
    // Set ENABLE_OLED_PONG to false to completely disable
    private static final boolean ENABLE_OLED_PONG = true;
    private final OLEDPongSubsystem pongSubsystem = ENABLE_OLED_PONG ? new OLEDPongSubsystem() : null;

    


  // 2. CONTROLLERS: Defining the Inputs
  // CommandXboxController is a wrapper that makes it easy to link buttons to commands.
  // kDriverControllerPort is usually '0' (set in Constants.java).
  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    SmartDashboard.putNumber("Target Hood Position", targetHoodPosition);

    // Register Named Commands for PathPlanner
    //Added the command registers for auto.
    com.pathplanner.lib.auto.NamedCommands.registerCommand("VisionAlign", m_VisionSubsystem.visionAlignCommand());
    com.pathplanner.lib.auto.NamedCommands.registerCommand("ShootSequence", new ShootSequence(m_ShooterSubsystem, m_HoodSubsystem, m_IntakeSubsystem, m_IndexerSubsystem, () -> m_VisionSubsystem.getTurretCameraTargets(), m_TurretSubsystem, () -> m_swerveSubsystem.getGyroYaw(), () -> m_driverController.getLeftTriggerAxis()>0.5,  m_VisionSubsystem));
    m_driverController.rightTrigger(RIGHT_TRIGGER_THRESHOLD).onFalse(m_IndexerSubsystem.unstuckBalls().withTimeout(1));
    com.pathplanner.lib.auto.NamedCommands.registerCommand("Intaking", m_IntakeSubsystem.Intaking());
    
autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(stream -> 
    stream.flatMap(auto -> {
        // 1. Get the original name from the PathPlanner file (e.g., "Go to fuel Left")
        String originalName = auto.getName();
        
        // 2. Create the mirrored copy (Y-axis only)
        var mirrored = new PathPlannerAuto(originalName, true);
        
        // 3. Clean up the name string by replacing "Left" with "Right"
        String rightName;
        if (originalName.contains("Left")) {
            rightName = originalName.replace("Left", "Right");
        } else if (originalName.contains("left")) {
            rightName = originalName.replace("left", "right");
        } else {
            // Fallback just in case a path file doesn't have the word "Left" in it
            rightName = originalName + " (Right Side)"; 
        }
        
        // 4. Set the updated name on the mirrored command
        mirrored.setName(rightName);
        
        // 5. Send both options to your dashboard dropdown menu
        return java.util.stream.Stream.of(auto, mirrored);
    })
);


    SmartDashboard.putData("Auto Chooser", autoChooser);


    configureBindings();
  }

  /**
   * NEW FOR 2026: This logic ensures that as soon as Teleop begins,
   * the robot's heading is aligned with its last known position from Auto.
   * It uses Vision (Limelight) if a tag is visible for maximum precision.
   */
  public void setupTeleopHeading() {
    m_VisionSubsystem.updateHeadingWithVision();
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
        () -> m_driverController.getLeftY() * (shooting ? -0.3 : -1),
        // Left/Right Strafe (X-Axis).
        () -> m_driverController.getLeftX() * (shooting ? -0.3 : -1)) 

        // Rotation: We use the Right Stick to spin the robot. 
        // We multiply by -1 here because usually "Right" on the stick should be "Clockwise."
        .withControllerRotationAxis(() -> m_driverController.getRightX() * (shooting ? -0.3:-1))
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
    m_IntakeSubsystem.setDefaultCommand(Commands.run(() -> m_IntakeSubsystem.Stop(), m_IntakeSubsystem));
    m_IndexerSubsystem.setDefaultCommand(Commands.run(() -> m_IndexerSubsystem.Stop(), m_IndexerSubsystem));

    // Y BUTTON: Resets the Gyro. 
    // If the robot's "Forward" direction gets confused, the driver points the robot 
    // away from them and presses Y to reset the "North" star.
    m_driverController.y().onTrue(m_swerveSubsystem.zeroGyroCommand());

    // B BUTTON: "Parking Brake" / Lock Pose.
    // While the driver holds B, the wheels turn into an 'X' shape so the robot cannot be pushed.
    m_driverController.x().whileTrue(m_swerveSubsystem.lockPoseCommand());

    // A BUTTON: Turret aim. Tracks tag 6 with the turret camera AND counter-rotates
    // against chassis spin (Pigeon IMU) so the aim holds when the robot turns.
    // Releasing stops the turret.
    m_driverController.a().whileTrue(new turretAim(
        m_TurretSubsystem,
        () -> m_VisionSubsystem.getTurretCameraTargets(),
        () -> m_swerveSubsystem.getGyroYaw(),
        m_VisionSubsystem));

    // Set hood postion for test
    m_driverController.b().onTrue(Commands.runOnce(() -> m_HoodSubsystem.setPosition(SmartDashboard.getNumber("Target Hood Position", targetHoodPosition)), m_HoodSubsystem));

    // RIGHT TRIGGER: Spin the shooter while held for scoring. Releasing stops the shooter.
    m_driverController.rightTrigger(RIGHT_TRIGGER_THRESHOLD).whileTrue(Commands.parallel(new ShootSequence(m_ShooterSubsystem, m_HoodSubsystem, m_IntakeSubsystem, m_IndexerSubsystem, () -> m_VisionSubsystem.getTurretCameraTargets(), m_TurretSubsystem,  () -> m_swerveSubsystem.getGyroYaw(), () -> m_driverController.getLeftTriggerAxis()>0.5, m_VisionSubsystem), Commands.run(() -> shooting = true)));
    m_driverController.rightTrigger(RIGHT_TRIGGER_THRESHOLD).onFalse(Commands.parallel(m_IndexerSubsystem.unstuckBalls().withTimeout(1), Commands.runOnce(() -> shooting = false)));

    // RIGHT BUMPER: Spin the shooter while held for passing. Releasing stops the shooter.
    // TEMP DISABLED: SpinShooter currently requires the indexer (for testing), which conflicts with
    // indexer.SpinIndexer() inside PassSequence's parallel group and crashes the robot on boot.
    // To re-enable: remove the indexer requirement from SpinShooter, then uncomment the two lines below.
    // m_driverController.rightBumper().whileTrue(new PassSequence(m_ShooterSubsystem, m_IntakeSubsystem, m_IndexerSubsystem, m_VisionSubsystem::getTurretCameraTargets, m_TurretSubsystem, m_HoodSubsystem, m_swerveSubsystem::getGyroYaw));
    // m_driverController.rightBumper().onFalse(m_IndexerSubsystem.unstuckBalls().withTimeout(1));
    m_driverController.povRight().whileTrue(m_spinShooterCommand);

    // LEFT TRIGGER: Lower intake and spin it keep trying to set intake position while intaking because balls can move it.
    m_driverController.leftTrigger(RIGHT_TRIGGER_THRESHOLD).whileTrue(m_IntakeSubsystem.Intaking());
    m_driverController.leftTrigger(RIGHT_TRIGGER_THRESHOLD).whileFalse(Commands.run(() -> m_IntakeSubsystem.intaking=false));
    
    m_driverController.leftBumper().whileTrue(Commands.run(() -> {
                                                                  m_IntakeSubsystem.setVelocityRPM(2000);
                                                                  m_IndexerSubsystem.setIndexerVelocityRPM(2000);}));
    // manual arm controlls for bench-testing the lower limit switch.
    // D-pad up drives the arm up, D-pad down drives it down (blocked at the switch).
    // Restore the line below when the test is done:
    m_driverController.povLeft().onTrue(m_IntakeSubsystem.IntakeUp());
    m_driverController.povUp().whileTrue(m_IntakeSubsystem.ManuallyRaise());
    m_driverController.povDown().whileTrue(m_IntakeSubsystem.ManuallyLower());
  }

  public void updateDashboard() {
    double rightTriggerAxis = m_driverController.getRightTriggerAxis();
    SmartDashboard.putNumber("Shooter/Right Trigger Axis", rightTriggerAxis);
    SmartDashboard.putNumber("Shooter/Right Trigger Threshold", RIGHT_TRIGGER_THRESHOLD);
    SmartDashboard.putBoolean("Shooter/Right Trigger Pressed", rightTriggerAxis > RIGHT_TRIGGER_THRESHOLD);
    SmartDashboard.putBoolean("Shooter/Command Scheduled", m_spinShooterCommand.isScheduled());
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
