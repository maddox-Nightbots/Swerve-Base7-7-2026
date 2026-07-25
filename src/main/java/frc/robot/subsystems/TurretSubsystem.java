package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase{

    private final SparkMax TurnMotor;
    private final SparkClosedLoopController controllerTurn;
    RelativeEncoder encoder;

    // Travel limits in MOTOR rotations (what the encoder/soft-limits actually count),
    // derived from the wire-safe turret-rotation limits in Constants.
    private static final double kMinMotorRotations =
        TurretConstants.kMinTurretRotations * TurretConstants.kMotorRotationsPerTurretRotation;
    private static final double kMaxMotorRotations =
        TurretConstants.kMaxTurretRotations * TurretConstants.kMotorRotationsPerTurretRotation;


    public TurretSubsystem() {
        TurnMotor = new SparkMax(TurretConstants.kTurnMotorID, MotorType.kBrushless);
        controllerTurn = TurnMotor.getClosedLoopController();

        SparkMaxConfig turretConfig = new SparkMaxConfig();
        turretConfig.inverted(false);
        turretConfig.smartCurrentLimit(TurretConstants.kSmartCurrentLimitAmps);
        turretConfig.closedLoop
            .p(TurretConstants.kP)
            .i(TurretConstants.kI)
            .d(TurretConstants.kD);

        // LAYER 2: firmware soft limits on the SparkMax itself. The controller will not
        // drive past these positions (in motor rotations) regardless of what the RIO commands.
        turretConfig.softLimit
            .forwardSoftLimit(kMaxMotorRotations)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(kMinMotorRotations)
            .reverseSoftLimitEnabled(true);

        TurnMotor.configure(turretConfig, com.revrobotics.ResetMode.kResetSafeParameters,
                com.revrobotics.PersistMode.kPersistParameters);

        encoder = TurnMotor.getEncoder();
    }

    /**
     * Command the turret to a position in TURRET rotations. LAYER 1 safety: the request
     * is clamped to the wire-safe travel window here, so NO caller (aim command, auto,
     * manual jog, etc.) can drive the turret past its limits.
     */
    public void setAngle(double angle) {
        double clamped = MathUtil.clamp(angle, TurretConstants.kMinTurretRotations, TurretConstants.kMaxTurretRotations);
        controllerTurn.setSetpoint(clamped * TurretConstants.kMotorRotationsPerTurretRotation, ControlType.kPosition);
        SmartDashboard.putNumber("turretAngle", clamped);
        SmartDashboard.putBoolean("Turret At Limit", clamped != angle);
    }

    public double getAngle(){
        return encoder.getPosition() / TurretConstants.kMotorRotationsPerTurretRotation;
    }

    public void stop() {
        TurnMotor.set(0.0);
    }
}
