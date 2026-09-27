package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

/**
 * Turret rotation, homed from its starting position.
 *
 * <p>ON START: the encoder is relative and lives in the SparkMax, which starts at 0 when it is
 * powered on. 0 is the HOME end ({@link TurretConstants#kHomeRotations}), so power the robot on
 * with the turret at home. A code restart / redeploy does NOT reset it: the SparkMax stays
 * powered and keeps its count. From there the turret may move between
 * {@link TurretConstants#kMinRotations} and {@link TurretConstants#kMaxRotations}. If the zero
 * is ever wrong, put the turret at home by hand (it coasts while disabled) and click
 * "Reset - Zero Turret" on the dashboard.
 *
 * <p>Travel is limited in two independent layers:
 * <ul>
 *   <li>LAYER 1 - {@link #setAngle(double)} clamps every request to the travel range.</li>
 *   <li>LAYER 2 - SparkMax firmware soft limits at the same range. This catches a code fault.</li>
 * </ul>
 *
 * <p>All public angles are in TURRET rotations (1.0 = one full turret revolution), NOT motor
 * rotations and not degrees.
 */
public class TurretSubsystem extends SubsystemBase {

    private final SparkMax turnMotor;
    private final SparkClosedLoopController turnController;
    private final RelativeEncoder encoder;

    private double lastCommandedRotations = TurretConstants.kHomeRotations;
    private boolean atLimit = false;

    // Recent turret angles by FPGA time, so a camera frame can be matched to where the
    // turret was when the frame was captured (frames arrive tens of ms late).
    private final TimeInterpolatableBuffer<Double> angleHistory =
        TimeInterpolatableBuffer.createDoubleBuffer(1.0);

    // Dashboard readout, published as the Alert group "Turret". Glass renders "Alerts" natively -
    // the same widget the vendor libs use for Encoders / IMU / Motors / Swerve Drive. A custom
    // Sendable type does NOT work here: Glass only draws types it has a built-in widget for, and
    // an unrecognized type is not even listed under NetworkTables > SmartDashboard.
    //
    // Constructing an Alert registers its group in NetworkTables, so there is deliberately no
    // putData("Turret", ...) call - that would collide on the same SmartDashboard/Turret subtable.
    // Alerts are created once and only toggled in periodic(), per the WPILib Alert docs.
    private final Alert statusAlert = new Alert("Turret", "", AlertType.kInfo);
    private final Alert angleAlert = new Alert("Turret", "", AlertType.kInfo);
    private final Alert atLimitAlert = new Alert("Turret", "", AlertType.kWarning);

    public TurretSubsystem() {
        turnMotor = new SparkMax(TurretConstants.kTurnMotorID, MotorType.kBrushless);
        turnController = turnMotor.getClosedLoopController();
        encoder = turnMotor.getEncoder();

        SparkMaxConfig turretConfig = new SparkMaxConfig();
        turretConfig.inverted(false);
        turretConfig.smartCurrentLimit(TurretConstants.kSmartCurrentLimitAmps);
        // Coast so the turret can be put back at home BY HAND while disabled. The relative
        // encoder still counts hand movement while the robot is powered, so this never loses
        // the zero - only a reboot/redeploy does.
        turretConfig.idleMode(SparkMaxConfig.IdleMode.kCoast);
        turretConfig.closedLoop
            .p(TurretConstants.kP)
            .i(TurretConstants.kI)
            .d(TurretConstants.kD)
            .feedForward.kS(0.3);

        // LAYER 2: firmware soft limits at the travel range. The controller refuses to drive
        // past these regardless of what the RIO commands.
        turretConfig.softLimit
            .forwardSoftLimit(TurretConstants.kMaxRotations * TurretConstants.kMotorRotationsPerTurretRotation)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(TurretConstants.kMinRotations * TurretConstants.kMotorRotationsPerTurretRotation)
            .reverseSoftLimitEnabled(true);

        turnMotor.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // No home() here. The SparkMax keeps counting as long as it has power, so a code
        // restart / redeploy picks up the real turret position instead of re-zeroing wherever
        // the turret happens to be. On a real power-up the SparkMax starts at 0, which is home
        // (kHomeRotations = 0), so the robot still has to be powered on with the turret at home.
        lastCommandedRotations = getAngle();

        // Published as a Command so it appears in Glass under NetworkTables > SmartDashboard
        // with a Run button. ignoringDisable(true) lets it work while the robot is DISABLED.
        SmartDashboard.putData("Reset - Zero Turret",
            Commands.runOnce(this::home, this).ignoringDisable(true).withName("Reset - Zero Turret"));
    }

    /** Declare "the turret is at its home end right now" and reset the encoder to match. */
    public void home() {
        stop();
        encoder.setPosition(TurretConstants.kHomeRotations * TurretConstants.kMotorRotationsPerTurretRotation);
        lastCommandedRotations = TurretConstants.kHomeRotations;
        atLimit = false;
        angleHistory.clear();
    }

    /** Lower travel limit in TURRET rotations. */
    public double getMinAngle() {
        return TurretConstants.kMinRotations;
    }

    /** Upper travel limit in TURRET rotations. */
    public double getMaxAngle() {
        return TurretConstants.kMaxRotations;
    }

    // --- Motion ---------------------------------------------------------------------------

    /** Command the turret to a position in TURRET rotations, clamped to the travel range (LAYER 1). */
    public void setAngle(double angle) {
        double clamped = MathUtil.clamp(angle, getMinAngle(), getMaxAngle());
        turnController.setSetpoint(
            clamped * TurretConstants.kMotorRotationsPerTurretRotation, ControlType.kPosition);
        lastCommandedRotations = clamped;
        atLimit = clamped != angle;
    }

    /** Current turret position in TURRET rotations. */
    public double getAngle() {
        return encoder.getPosition() / TurretConstants.kMotorRotationsPerTurretRotation;
    }

    /** Turret position in TURRET rotations at an FPGA timestamp (e.g. a camera frame's). */
    public double getAngleAt(double timestampSeconds) {
        return angleHistory.getSample(timestampSeconds).orElse(getAngle());
    }

    // --- Geometry --------------------------------------------------------------------------

    /** Robot-relative shooter heading (CCW-positive, 0 = robot front) at a turret angle. */
    public static Rotation2d headingForAngle(double turretRotations) {
        return Rotation2d.fromDegrees(TurretConstants.kHomeHeadingDegrees - turretRotations * 360.0);
    }

    /**
     * Turret angle (TURRET rotations) that points the shooter at a robot-relative heading.
     * Headings in the unreachable gap behind the travel range return the nearer limit.
     */
    public double angleForHeading(Rotation2d robotRelativeHeading) {
        double raw = (TurretConstants.kHomeHeadingDegrees - robotRelativeHeading.getDegrees()) / 360.0;
        double min = getMinAngle();
        double max = getMaxAngle();
        // The one equivalent angle in [min, min + 1). Anything above max is in the gap.
        double angle = MathUtil.inputModulus(raw, min, min + 1.0);
        if (angle > max) {
            angle = (angle - max) < (min + 1.0 - angle) ? max : min;
        }
        return angle;
    }

    /** Cut the output. Note this drops closed-loop hold, so the turret coasts. */
    public void stop() {
        turnMotor.set(0.0);
    }

    // --- Dashboard ------------------------------------------------------------------------

    /** Refresh the "Turret" Alert group that Glass draws. */
    private void updateAlerts() {
        statusAlert.setText(String.format("Homed at %.1f deg - travel %.1f to %.1f deg.",
            TurretConstants.kHomeRotations * 360.0, getMinAngle() * 360.0, getMaxAngle() * 360.0));
        statusAlert.set(true);

        angleAlert.setText(String.format("Angle %.1f deg  |  commanded %.1f deg",
            getAngle() * 360.0, lastCommandedRotations * 360.0));
        angleAlert.set(true);

        atLimitAlert.setText("AT LIMIT - request was clamped to the travel range.");
        atLimitAlert.set(atLimit);
    }

    @Override
    public void periodic() {
        angleHistory.addSample(Timer.getFPGATimestamp(), getAngle());
        updateAlerts();

        // Numeric readouts live under "TurretDiag/" so they stay clear of the
        // "SmartDashboard/Turret" subtable that the Alert group owns.
        SmartDashboard.putBoolean("TurretDiag/At Limit", atLimit);
        SmartDashboard.putNumber("TurretDiag/Current Angle (deg)", getAngle() * 360.0);
        SmartDashboard.putNumber("TurretDiag/Commanded Angle (deg)", lastCommandedRotations * 360.0);
        SmartDashboard.putNumber("TurretDiag/Min Angle (deg)", getMinAngle() * 360.0);
        SmartDashboard.putNumber("TurretDiag/Max Angle (deg)", getMaxAngle() * 360.0);

        SmartDashboard.putNumber("TurretDiag/Motor Output", turnMotor.getAppliedOutput());
        SmartDashboard.putNumber("TurretDiag/Motor Current (A)", turnMotor.getOutputCurrent());
        SmartDashboard.putNumber("TurretDiag/Motor Temp (C)", turnMotor.getMotorTemperature());
        SmartDashboard.putNumber("TurretDiag/Motor Velocity (RPM)", encoder.getVelocity());
        SmartDashboard.putNumber("TurretDiag/Encoder Rotations (motor)", encoder.getPosition());
    }
}
