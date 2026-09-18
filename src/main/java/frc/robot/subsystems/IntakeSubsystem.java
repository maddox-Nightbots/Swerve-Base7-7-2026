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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DataLogManager;
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

    private final DigitalInput m_limitSwitch = new DigitalInput(1);

     private boolean m_safetyTriggered = false; 

    public boolean intaking = false;

    public IntakeSubsystem() {

        //config motors
        // null when disabled in Constants, so every use below must check for null
        intakeMotor = IntakeConstants.kEnableIntakeRollerTalon ? new TalonFX(56) : null;
        armMotor = new SparkMax(55, MotorType.kBrushless);

        //config arm motor
        SparkMaxConfig armConfig = new SparkMaxConfig();

        armConfig.inverted(true);
        armConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);

        armConfig.smartCurrentLimit(40);
        armConfig.closedLoop.p(0.003).i(0.000001).d(0.0004);
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
        if (intakeMotor != null) {
            intakeMotor.getConfigurator().apply(intakeConfig);
        }

        intakepidController = armMotor.getClosedLoopController();
        armEncoder = armMotor.getEncoder();

    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("intake pos", getArmPosition());
        if (m_limitSwitch.get() && !m_safetyTriggered){
            m_safetyTriggered=true;
            this.limitTriggereCommand();
        }

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
        if (intakeMotor != null) {
            intakeMotor.setControl(intakevelocityRequest.withVelocity(targetRPS));
        }
    }

    public void setIntakePosition(double position) {
        
        double target = position * IntakeConstants.gearRatio;
        intakepidController.setSetpoint(target, ControlType.kPosition);

    }

    public double getIntakePosition() {
        return intakepidController.getSetpoint() / IntakeConstants.gearRatio;
    }

    /**
     * Completely stops the subsystem
     */
    public void Stop(){
        setVelocityRPM(0);
        armMotor.set(0);
    }

    /*
    when limit is triggered: fix encoder reading
    raise the subsystem slowly for half a second
    then fully stop the subsystem
    wait 1 second before other commands can take over again
    reset limit triggering so the bot will be usable again
    and it will prevent any other command from stopping it giving this priority
    */
    public Command limitTriggereCommand(){
        return Commands.sequence(
            this.run(() -> {
                armMotor.set(0.0);
                armEncoder.setPosition(-0.27 * IntakeConstants.gearRatio);
                DataLogManager.log("Limit switch triggered! Overriding Intake.");
            }),
            this.ManuallyRaise(),
            Commands.waitSeconds(0.25),
            this.run(() -> this.Stop()),
            Commands.waitSeconds(1),
            run(() -> m_safetyTriggered = false)
            ).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
    }

    
    public Command ManuallyRaise() {
        return this.run(
            () -> armMotor.set(0.1)
            );
    }

    
    public Command ManuallyLower() {
        return this.run(
            () -> armMotor.set(-0.1)
            );
    }

    // ---- end temporary jog ----

    public Command SpinIntakeWheels(){
        return this.run(() -> {
            this.setVelocityRPM(-4000);
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
        if (!intaking){
        // Set a setpoint once, then wait for the ARM (encoder, not setpoint) to get there.
        // The timeout keeps a blocked arm from stalling the jiggle forever.
        final double low = IntakeConstants.IntakeDownPosition * 3 / 4;
        final double high = IntakeConstants.IntakeDownPosition / 4;
        return Commands.sequence(
            this.runOnce(() -> setIntakePosition(low)),
            Commands.waitUntil(() -> MathUtil.isNear(low, getArmPosition(), 0.02)),
                //.withTimeout(IntakeConstants.ArmMoveTimeoutSeconds),
            this.runOnce(() -> setIntakePosition(high)),
            Commands.waitUntil(() -> MathUtil.isNear(high, getArmPosition(), 0.02))
                //.withTimeout(IntakeConstants.ArmMoveTimeoutSeconds)
        );
    }
    return run(() -> {
        SmartDashboard.putBoolean("intake running", intaking);
    });
}


    public Command Intaking(){
        return this.run(() -> {
            this.setIntakePosition(IntakeConstants.IntakeDownPosition);
            this.setVelocityRPM(-2000);
            intaking = true;
        });
    }
}
