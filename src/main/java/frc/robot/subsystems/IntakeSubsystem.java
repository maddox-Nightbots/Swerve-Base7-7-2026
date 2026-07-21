package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import static com.revrobotics.PersistMode.kPersistParameters;
import static com.revrobotics.ResetMode.kResetSafeParameters;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
/*import frc.robot.LimelightHelpers;*/

public class IntakeSubsystem extends SubsystemBase {

    //create motors
    private final TalonFX intakeMotor;
    private final SparkMax armMotor;


    private final VelocityVoltage intakevelocityRequest = new VelocityVoltage(0);

    private final SparkClosedLoopController intakepidController;


    public IntakeSubsystem() {

        //config motors
        intakeMotor = new TalonFX(5);
        armMotor = new SparkMax(6, MotorType.kBrushless);

        //config arm motor
        SparkMaxConfig armConfig = new SparkMaxConfig();

        armConfig.inverted(true);
        armConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);

        armConfig.smartCurrentLimit(40);
        armConfig.closedLoop.p(0.0002).i(0.000001).d(0.0004);
        armMotor.configure(armConfig, kResetSafeParameters, kPersistParameters);

        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
          // Match the 40A baseline to protect your robot battery during high loads
        intakeConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;


        intakeConfig.Slot0.kP = 0.11;  // Proportional gain (corrects error)
        intakeConfig.Slot0.kV = 0.12;  // Feedforward gain (scales voltage to expected velocity)
        intakeConfig.Slot0.kI = 0.0;   // Integral gain (rarely needed for FRC velocity)
        intakeConfig.Slot0.kD = 0.0;   // Derivative gain (helps reduce overshoot)

        // Apply configs to Kraken
        intakeMotor.getConfigurator().apply(intakeConfig);

        intakepidController = armMotor.getClosedLoopController();

    }

    /**
     * Drives the Kraken at a specific velocity target.
     * @param targetRPM The desired engine speed in Revolutions Per Minute (Max ~6000 RPM)
     */
    public void setVelocityRPM(double targetRPM) {
        // Convert RPM to Rotations Per Second (RPS)
        double targetRPS = targetRPM / 60.0;

        // Use the request object to smoothly command the motor
        intakeMotor.setControl(intakevelocityRequest.withVelocity(targetRPS));
    }

    public void setIntakePosition(double position) {
        intakepidController.setSetpoint(position * IntakeConstants.gearRatio, ControlType.kPosition);
    }

    public double getIntakePosition() {
        return intakepidController.getSetpoint() / IntakeConstants.gearRatio;
    }

    public Command SpinIntakeWheels(){
        return this.run(() -> {
            this.setVelocityRPM(2000);
        });
    }

    public Command IntakeDown() {
        return this.runOnce(() -> {
        this.setIntakePosition(IntakeConstants.IntakeDownPosition);
    });
    }

    public Command IntakeUp() {
        return this.runOnce(() -> {
        this.setIntakePosition(IntakeConstants.IntakeUpPosition);
    });
    }

    public Command IntakeUpDown() {
        return Commands.sequence(
            this.runOnce(() -> this.setIntakePosition(IntakeConstants.IntakeDownPosition*3/4)).until(() -> this.getIntakePosition() == IntakeConstants.IntakeDownPosition*3/4),
        
            this.runOnce(() -> this.setIntakePosition(IntakeConstants.IntakeDownPosition/4)).until(() -> this.getIntakePosition() == IntakeConstants.IntakeDownPosition/4)
        ).repeatedly();
    }

    public Command Intaking(){
        return Commands.sequence(this.runOnce(() -> this.IntakeDown()), this.run(() -> this.SpinIntakeWheels()));
    }
}
