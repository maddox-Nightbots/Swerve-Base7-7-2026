package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {

  public final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();

  private final SendableChooser<Command> autoChooser;

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(stream ->
        stream.flatMap(auto -> {
          String originalName = auto.getName();
          var mirrored = new PathPlannerAuto(originalName, true);
          String rightName;
          if (originalName.contains("Left")) {
            rightName = originalName.replace("Left", "Right");
          } else if (originalName.contains("left")) {
            rightName = originalName.replace("left", "right");
          } else {
            rightName = originalName + " (Right Side)";
          }
          mirrored.setName(rightName);
          return java.util.stream.Stream.of(auto, mirrored);
        })
    );

    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    SwerveInputStream driveInputStream = SwerveInputStream.of(
        m_swerveSubsystem.getSwerveDrive(),
        () -> -m_driverController.getLeftY(),
        () -> -m_driverController.getLeftX())
        .withControllerRotationAxis(() -> -m_driverController.getRightX())
        .deadband(OperatorConstants.DEADBAND)
        .scaleTranslation(1.0)
        .allianceRelativeControl(true);

    m_swerveSubsystem.setDefaultCommand(m_swerveSubsystem.driveFieldOriented(driveInputStream));

    m_driverController.y().onTrue(m_swerveSubsystem.zeroGyroCommand());
    m_driverController.x().whileTrue(m_swerveSubsystem.lockPoseCommand());
  }

  public void updateDashboard() {
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public SwerveSubsystem getM_swerveSubsystem() {
    return m_swerveSubsystem;
  }
}
