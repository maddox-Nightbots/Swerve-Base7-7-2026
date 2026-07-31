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
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

/**
 * Turret rotation, with a hand-taught travel range.
 *
 * <p>SETUP PROCEDURE (do this every boot - the encoder is relative):
 * <ol>
 *   <li>Leave the robot DISABLED. The turret idles in coast so it can be turned by hand.</li>
 *   <li>Turn the turret by hand to one end of its safe travel, then click "Set Clamp" on the
 *       dashboard.</li>
 *   <li>Turn it to the other end and click "Set Clamp" again.</li>
 * </ol>
 * The turret refuses to move until both clamps are captured. "Reset Clamps" starts over.
 *
 * <p>Travel is limited in two independent layers:
 * <ul>
 *   <li>LAYER 1 - {@link #setAngle(double)} clamps every request to the taught range.</li>
 *   <li>LAYER 2 - SparkMax firmware soft limits at the absolute backstop. This catches a code
 *       fault; it is not the everyday limit.</li>
 * </ul>
 *
 * <p>All public angles are in TURRET rotations (1.0 = one full turret revolution), NOT motor
 * rotations and not degrees.
 */
public class TurretSubsystem extends SubsystemBase {

    private static final double kBackstopMotorRotations =
        TurretConstants.kBackstopRotations * TurretConstants.kMotorRotationsPerTurretRotation;

    private final SparkMax turnMotor;
    private final SparkClosedLoopController turnController;
    private final RelativeEncoder encoder;

    // --- Hand-taught travel clamps (turret rotations) ---
    private double clamp1Rotations = 0.0;
    private double clamp2Rotations = 0.0;
    private boolean clamp1Set = false;
    private boolean clamp2Set = false;
    private boolean nextIsClamp1 = true; // which clamp the next click captures

    private double lastCommandedRotations = 0.0;
    private boolean atLimit = false;

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
    private final Alert clampAlert = new Alert("Turret", "", AlertType.kInfo);
    private final Alert lockedAlert = new Alert("Turret", "", AlertType.kWarning);
    private final Alert atLimitAlert = new Alert("Turret", "", AlertType.kWarning);
    private final Alert narrowRangeAlert = new Alert("Turret", "", AlertType.kWarning);

    public TurretSubsystem() {
        turnMotor = new SparkMax(TurretConstants.kTurnMotorID, MotorType.kBrushless);
        turnController = turnMotor.getClosedLoopController();
        encoder = turnMotor.getEncoder();

        SparkMaxConfig turretConfig = new SparkMaxConfig();
        turretConfig.inverted(false);
        turretConfig.smartCurrentLimit(TurretConstants.kSmartCurrentLimitAmps);
        // Coast so the turret can be moved BY HAND while teaching the clamps.
        turretConfig.idleMode(SparkMaxConfig.IdleMode.kCoast);
        turretConfig.closedLoop
            .p(TurretConstants.kP)
            .i(TurretConstants.kI)
            .d(TurretConstants.kD);

        // LAYER 2: firmware soft limits, symmetric about the boot position. The controller
        // refuses to drive past these regardless of what the RIO commands.
        turretConfig.softLimit
            .forwardSoftLimit(kBackstopMotorRotations)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(-kBackstopMotorRotations)
            .reverseSoftLimitEnabled(true);

        turnMotor.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Published as Commands so they appear in Glass under NetworkTables > SmartDashboard with
        // type "Command" and a Run button. ignoringDisable(true) is what lets them work while the
        // robot is DISABLED, which is when the clamps are actually taught.
        SmartDashboard.putData("Set Clamp",
            Commands.runOnce(this::captureClamp).ignoringDisable(true).withName("Set Clamp"));
        SmartDashboard.putData("Reset Clamps",
            Commands.runOnce(this::resetClamps).ignoringDisable(true).withName("Reset Clamps"));
    }

    // --- Teaching the travel range -------------------------------------------------------

    /**
     * Capture the turret's current (hand-set) angle as the next clamp, alternating 1 / 2.
     * The captured value is limited to the absolute backstop so a taught clamp can never sit
     * outside what the firmware soft limits will actually allow the turret to reach.
     */
    public void captureClamp() {
        double current = MathUtil.clamp(getAngle(),
            -TurretConstants.kBackstopRotations, TurretConstants.kBackstopRotations);

        if (nextIsClamp1) {
            clamp1Rotations = current;
            clamp1Set = true;
        } else {
            clamp2Rotations = current;
            clamp2Set = true;
        }
        nextIsClamp1 = !nextIsClamp1;
    }

    /** Forget both clamps and lock the turret until they are taught again. */
    public void resetClamps() {
        clamp1Set = false;
        clamp2Set = false;
        clamp1Rotations = 0.0;
        clamp2Rotations = 0.0;
        nextIsClamp1 = true;
        atLimit = false;
        stop();
    }

    /** True once BOTH clamps have been captured - the turret is allowed to move. */
    public boolean bothClampsSet() {
        return clamp1Set && clamp2Set;
    }

    /** Lower clamp bound in TURRET rotations (backstop until both are taught). */
    public double getClampMin() {
        if (!bothClampsSet()) return -TurretConstants.kBackstopRotations;
        return Math.min(clamp1Rotations, clamp2Rotations);
    }

    /** Upper clamp bound in TURRET rotations (backstop until both are taught). */
    public double getClampMax() {
        if (!bothClampsSet()) return TurretConstants.kBackstopRotations;
        return Math.max(clamp1Rotations, clamp2Rotations);
    }

    /** True when both clamps are taught but so close together the turret can barely move. */
    public boolean isRangeTooNarrow() {
        return bothClampsSet()
            && (getClampMax() - getClampMin()) < TurretConstants.kMinClampSpreadRotations;
    }

    // --- Motion ---------------------------------------------------------------------------

    /**
     * Command the turret to a position in TURRET rotations. The turret will NOT move until both
     * clamps are taught; after that the request is clamped between them (LAYER 1).
     */
    public void setAngle(double angle) {
        if (!bothClampsSet()) {
            stop(); // locked out - do not drive until both clamps are taught
            atLimit = false;
            return;
        }

        double clamped = MathUtil.clamp(angle, getClampMin(), getClampMax());
        turnController.setSetpoint(
            clamped * TurretConstants.kMotorRotationsPerTurretRotation, ControlType.kPosition);
        lastCommandedRotations = clamped;
        atLimit = clamped != angle;
    }

    /** Current turret position in TURRET rotations. */
    public double getAngle() {
        return encoder.getPosition() / TurretConstants.kMotorRotationsPerTurretRotation;
    }

    /** Cut the output. Note this drops closed-loop hold, so the turret coasts. */
    public void stop() {
        turnMotor.set(0.0);
    }

    // --- Dashboard ------------------------------------------------------------------------

    /** Plain-English status shown on the dashboard. */
    private String getStatusMessage() {
        if (!bothClampsSet()) {
            String taught = clamp1Set ? String.format(" Clamp 1 = %.1f deg.", clamp1Rotations * 360.0)
                          : clamp2Set ? String.format(" Clamp 2 = %.1f deg.", clamp2Rotations * 360.0)
                          : "";
            return String.format(
                "LOCKED -%s Move the turret by hand, then click 'Set Clamp' to capture %s.",
                taught, nextIsClamp1 ? "Clamp 1" : "Clamp 2");
        }
        return String.format("ACTIVE - clamped between %.1f and %.1f deg.",
            getClampMin() * 360.0, getClampMax() * 360.0);
    }

    /** Refresh the "Turret" Alert group that Glass draws. */
    private void updateAlerts() {
        statusAlert.setText(getStatusMessage());
        statusAlert.set(true);

        angleAlert.setText(String.format("Angle %.1f deg  |  commanded %.1f deg",
            getAngle() * 360.0, lastCommandedRotations * 360.0));
        angleAlert.set(true);

        clampAlert.setText(String.format("Clamp 1 %s  |  Clamp 2 %s",
            clamp1Set ? String.format("%.1f deg", clamp1Rotations * 360.0) : "NOT SET",
            clamp2Set ? String.format("%.1f deg", clamp2Rotations * 360.0) : "NOT SET"));
        clampAlert.set(true);

        lockedAlert.setText(String.format(
            "LOCKED - turret will not move. Next 'Set Clamp' click captures %s.",
            nextIsClamp1 ? "Clamp 1" : "Clamp 2"));
        lockedAlert.set(!bothClampsSet());

        atLimitAlert.setText("AT LIMIT - request was clamped to the taught travel range.");
        atLimitAlert.set(atLimit);

        narrowRangeAlert.setText(String.format(
            "Taught range is only %.1f deg wide - the turret has almost no travel. "
                + "Click 'Reset Clamps' and teach it again.",
            (getClampMax() - getClampMin()) * 360.0));
        narrowRangeAlert.set(isRangeTooNarrow());
    }

    @Override
    public void periodic() {
        updateAlerts();

        // Numeric readouts live under "TurretDiag/" so they stay clear of the
        // "SmartDashboard/Turret" subtable that the Alert group owns.
        SmartDashboard.putBoolean("TurretDiag/Can Move", bothClampsSet());
        SmartDashboard.putBoolean("TurretDiag/At Limit", atLimit);
        SmartDashboard.putNumber("TurretDiag/Current Angle (deg)", getAngle() * 360.0);
        SmartDashboard.putNumber("TurretDiag/Commanded Angle (deg)", lastCommandedRotations * 360.0);
        SmartDashboard.putNumber("TurretDiag/Clamp Min (deg)", getClampMin() * 360.0);
        SmartDashboard.putNumber("TurretDiag/Clamp Max (deg)", getClampMax() * 360.0);

        SmartDashboard.putNumber("TurretDiag/Motor Output", turnMotor.getAppliedOutput());
        SmartDashboard.putNumber("TurretDiag/Motor Current (A)", turnMotor.getOutputCurrent());
        SmartDashboard.putNumber("TurretDiag/Motor Temp (C)", turnMotor.getMotorTemperature());
        SmartDashboard.putNumber("TurretDiag/Motor Velocity (RPM)", encoder.getVelocity());
        SmartDashboard.putNumber("TurretDiag/Encoder Rotations (motor)", encoder.getPosition());
    }
}
