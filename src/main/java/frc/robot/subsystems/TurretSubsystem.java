package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class TurretSubsystem extends SubsystemBase{

    private final SparkMax TurnMotor;
    private final SparkClosedLoopController controllerTurn;
    RelativeEncoder encoder;

    // --- Turret aim tuning ---
    private static final double kAimDeadbandDegrees = 1.0; // within this yaw, call it centered
    private static final double kAimGain = 0.3;            // fraction of the yaw error corrected per loop
    

    public TurretSubsystem() {
        TurnMotor = new SparkMax(ShooterConstants.TurnMotorID, MotorType.kBrushless);
        controllerTurn = TurnMotor.getClosedLoopController();

        SparkMaxConfig ShooterConstants = new SparkMaxConfig();
        ShooterConstants.inverted(false);
        ShooterConstants.smartCurrentLimit(40);
        ShooterConstants.closedLoop
        .p(0.11)
        .i(0.0)
        .d(0.0);

        SparkMaxConfig TurnConfig = ShooterConstants;
        TurnConfig.inverted(false);

        TurnMotor.configure(TurnConfig, com.revrobotics.ResetMode.kResetSafeParameters,
                com.revrobotics.PersistMode.kPersistParameters);

        encoder = TurnMotor.getEncoder();
    }

    public void setAngle(double angle) {
        controllerTurn.setSetpoint((angle * 200/14), ControlType.kPosition);
        SmartDashboard.putNumber("turretAngle", angle);
    }

    public double getAngle(){
        return (14*encoder.getPosition()/200);
    }

    /**
     * Aim the turret to center a target, given that target's yaw (in DEGREES) from the
     * turret camera. Converts the yaw error into turret rotations, steps a fraction
     * toward center each loop (so it eases in with no snap), and delegates to setAngle(),
     * which applies the gear ratio to get MOTOR rotations. Settles when yaw -> 0.
     *
     * Positive yaw is assumed to be the tag to the turret's LEFT; flip the sign below
     * if the turret turns AWAY from the tag on your robot.
     */
    public void aimAtYaw(double yawDegrees) {
        double currentTurretRotations = getAngle();

        // Centered enough -> hold position, don't jitter around zero.
        if (Math.abs(yawDegrees) < kAimDeadbandDegrees) {
            setAngle(currentTurretRotations);
            return;
        }

        // Degrees of error -> turret rotations. setAngle() then converts to motor rotations.
        double errorTurretRotations = yawDegrees / 360.0;

        // Command a small step toward center each loop (no big jump = no snap).
        double target = currentTurretRotations - (kAimGain * errorTurretRotations);
        setAngle(target);
    }

    public void stop() {
        TurnMotor.set(0.0);
    }
}
