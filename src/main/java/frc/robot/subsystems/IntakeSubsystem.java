package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import static com.revrobotics.PersistMode.kPersistParameters;
import com.revrobotics.RelativeEncoder;
import static com.revrobotics.ResetMode.kResetSafeParameters;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
/*import frc.robot.LimelightHelpers;*/

public class IntakeSubsystem extends SubsystemBase {

    //create motors
    private final TalonFX intakeMotor;
    private final SparkMax armMotor;

    private final RelativeEncoder armEncoder;

    private final VelocityVoltage intakevelocityRequest = new VelocityVoltage(0);

    private final SparkClosedLoopController intakepidController;

    // Bottom-of-travel limit switch. Reads low (false) when the arm is down.
    private final DigitalInput armLowerLimit;

    // Motor position captured the moment the limit switch trips. Holding this
    // fixed value (rather than chasing the live encoder) is what gives the PID a
    // real error to work against at the limit.
    private double lowerLimitHoldPosition = 0.0;
    private boolean wasAtLowerLimit = false;

    // TEMPORARY: true while a manual jog owns the arm motor. Open-loop set() and the
    // closed-loop hold in periodic() would otherwise overwrite each other every loop.
    private boolean jogging = false;


    public IntakeSubsystem() {

        //config motors
        intakeMotor = new TalonFX(6);
        armMotor = new SparkMax(5, MotorType.kBrushless);

        //config arm motor
        SparkMaxConfig armConfig = new SparkMaxConfig();

        armConfig.inverted(true);
        armConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);

        armConfig.smartCurrentLimit(40);
        armConfig.closedLoop.p(0.002).i(0.000001).d(0.0004);
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
        armEncoder = armMotor.getEncoder();

        armLowerLimit = new DigitalInput(IntakeConstants.ArmLowerLimitSwitchPort);

    }

    @Override
    public void periodic() {
        updateLowerLimitLatch();

        SmartDashboard.putNumber("intake pos", getArmPosition());
        SmartDashboard.putBoolean("intake at lower limit", atLowerLimit());

        // The switch can trip while a setpoint is already commanding us down
        // (a held command, or the arm coasting past the trip point), so cut the
        // downward command off here too rather than only at the call site.
        if (!jogging && wasAtLowerLimit && intakepidController.getSetpoint() < lowerLimitHoldPosition) {
            intakepidController.setSetpoint(lowerLimitHoldPosition, ControlType.kPosition);
        }
    }

    /** True when the limit switch is pulled low, i.e. the arm is at the bottom. */
    public boolean atLowerLimit() {
        return !armLowerLimit.get();
    }

    /**
     * Captures the arm position on the rising edge of the limit switch. Called from
     * both periodic() and setIntakePosition() so the latch is correct no matter
     * which runs first within a loop.
     */
    private void updateLowerLimitLatch() {
        boolean atLimit = atLowerLimit();
        if (atLimit && !wasAtLowerLimit) {
            armEncoder.setPosition(-0.27);
            lowerLimitHoldPosition = armEncoder.getPosition();
            armMotor.set(0);
        }
        wasAtLowerLimit = atLimit;
    }

    /** Actual arm position from the encoder, in output rotations. */
    public double getArmPosition() {
        return armEncoder.getPosition() / IntakeConstants.gearRatio;
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
        double target = position * IntakeConstants.gearRatio;

        jogging = false;
        updateLowerLimitLatch();

        // Down is the negative direction. If the limit switch is tripped, refuse
        // any target below the latched trip position and hold there instead;
        // moving back up is still allowed.
        if (wasAtLowerLimit && target < lowerLimitHoldPosition) {
            target = lowerLimitHoldPosition;
        }

        intakepidController.setSetpoint(target, ControlType.kPosition);
    }

    public double getIntakePosition() {
        return intakepidController.getSetpoint() / IntakeConstants.gearRatio;
    }

    /**
     * Stops the intake wheels. Deliberately does NOT touch the arm motor: calling
     * armMotor.set(0) would overwrite the closed-loop position request and let the
     * arm fall, so the arm is left holding whatever setpoint it was last given.
     */
    public void Stop(){
        setVelocityRPM(0);
    }

    // ---- TEMPORARY manual jog, for bench-testing the lower limit switch ----

    /**
     * TEMPORARY: open-loop arm control. Negative output drives the arm down, and is
     * refused once the limit switch trips. Positive (up) is always allowed.
     */
    private void setArmOutput(double output) {
        jogging = true;
        updateLowerLimitLatch();

        if (output < 0 && wasAtLowerLimit) {
            output = 0;
        }

        armMotor.set(output);
    }

    /** TEMPORARY: jog the arm up while held. */
    public Command JogArmUp() {
        return this.runEnd(
            () -> this.setArmOutput(IntakeConstants.ArmJogSpeed),
            () -> armMotor.set(0));
    }

    /** TEMPORARY: jog the arm down while held, stopping at the limit switch. */
    public Command JogArmDown() {
        return this.runEnd(
            () -> this.setArmOutput(-IntakeConstants.ArmJogSpeed),
            () -> armMotor.set(0));
    }

    // ---- end temporary jog ----

    public Command SpinIntakeWheels(){
        return this.run(() -> {
            this.setVelocityRPM(-2000);
        });
    }

    // These use run(), not runOnce(): a runOnce finishes on the first loop, which
    // hands the subsystem straight back to the default command before the arm has
    // travelled anywhere. run() keeps the command (and the setpoint) alive for as
    // long as the button is held.
    public Command IntakeDown() {
        return this.run(() -> {
        this.setIntakePosition(IntakeConstants.IntakeDownPosition);
    });
    }

    public Command IntakeUp() {
        return this.run(() -> {
        this.setIntakePosition(IntakeConstants.IntakeUpPosition);
    });
    }

    public Command IntakeUpDown() {
        return Commands.sequence(
            MoveArmTo(IntakeConstants.IntakeDownPosition*3/4),
            MoveArmTo(IntakeConstants.IntakeDownPosition/4)
        );
    }

    /**
     * Drives the arm to a position and finishes once the encoder actually reports
     * arrival, within tolerance. Times out so a blocked or stalled arm cannot hang
     * whatever command group is waiting on it.
     */
    private Command MoveArmTo(double position) {
        return this.run(() -> this.setIntakePosition(position))
            .until(() -> Math.abs(this.getArmPosition() - position) <= IntakeConstants.ArmPositionTolerance)
            .withTimeout(IntakeConstants.ArmMoveTimeoutSeconds);
    }

    public Command Intaking(){
        return this.run(() -> {
            this.setIntakePosition(IntakeConstants.IntakeDownPosition);
            this.setVelocityRPM(-2000);
        });
    }
}
