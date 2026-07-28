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

    // Dashboard keys (all under the "Turret" group in SmartDashboard).
    private static final String kSetClampKey = "Turret/Set Clamp (click)";

    // Absolute firmware backstop (turret rotations, symmetric about 0). The hand-set clamps
    // below operate WITHIN this; firmware only catches a code fault. SAFETY: this is the
    // widest the turret can EVER go, so it must be within what the wiring can take.
    private static final double kBackstopRotations = 180.0 / 360.0; // 0.5 rot = 180 deg
    private static final double kBackstopMotorRotations =
        kBackstopRotations * TurretConstants.kMotorRotationsPerTurretRotation;

    // --- Hand-taught travel clamps (turret rotations) ---
    // Move the turret to each physical limit by hand and click the "Set Clamp" checkbox on
    // the dashboard to capture the current angle. The turret refuses to move until BOTH are
    // captured. Clicking again re-captures, alternating Clamp 1 / Clamp 2.
    private double clamp1Rotations = 0.0;
    private double clamp2Rotations = 0.0;
    private boolean clamp1Set = false;
    private boolean clamp2Set = false;
    private boolean nextIsClamp1 = true; // which clamp the next click captures

    private double lastCommandedRotations = 0.0;
    private boolean atLimit = false;

    public TurretSubsystem() {
        TurnMotor = new SparkMax(TurretConstants.kTurnMotorID, MotorType.kBrushless);
        controllerTurn = TurnMotor.getClosedLoopController();

        SparkMaxConfig turretConfig = new SparkMaxConfig();
        turretConfig.inverted(false);
        turretConfig.smartCurrentLimit(TurretConstants.kSmartCurrentLimitAmps);
        // Coast so the turret can be moved BY HAND while setting the clamps.
        turretConfig.idleMode(SparkMaxConfig.IdleMode.kCoast);
        turretConfig.closedLoop
            .p(TurretConstants.kP)
            .i(TurretConstants.kI)
            .d(TurretConstants.kD);

        // LAYER 2: firmware soft limits on the SparkMax itself, at the absolute backstop
        // (symmetric). The controller won't drive past these regardless of RIO commands.
        turretConfig.softLimit
            .forwardSoftLimit(kBackstopMotorRotations)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(-kBackstopMotorRotations)
            .reverseSoftLimitEnabled(true);

        TurnMotor.configure(turretConfig, com.revrobotics.ResetMode.kResetSafeParameters,
                com.revrobotics.PersistMode.kPersistParameters);

        encoder = TurnMotor.getEncoder();

        // Publish the "Set Clamp" checkbox so it appears on the dashboard. Clicking it (true)
        // is read + reset in periodic(). This is a plain boolean - shows in every dashboard.
        SmartDashboard.putBoolean(kSetClampKey, false);
    }

    /** Capture the turret's current (hand-set) angle as the next clamp, alternating 1 / 2. */
    public void captureClamp() {
        double current = getAngle();
        if (nextIsClamp1) {
            clamp1Rotations = current;
            clamp1Set = true;
            nextIsClamp1 = false;
        } else {
            clamp2Rotations = current;
            clamp2Set = true;
            nextIsClamp1 = true;
        }
    }

    /** True once BOTH clamps have been captured - the turret is allowed to move. */
    public boolean bothClampsSet() {
        return clamp1Set && clamp2Set;
    }

    /** Lower clamp bound in TURRET rotations (backstop until both are set). */
    public double getClampMin() {
        if (!bothClampsSet()) return -kBackstopRotations;
        return Math.min(clamp1Rotations, clamp2Rotations);
    }

    /** Upper clamp bound in TURRET rotations (backstop until both are set). */
    public double getClampMax() {
        if (!bothClampsSet()) return kBackstopRotations;
        return Math.max(clamp1Rotations, clamp2Rotations);
    }

    /**
     * Command the turret to a position in TURRET rotations. The turret will NOT move until
     * both clamps are captured; after that the request is clamped between them (LAYER 1).
     */
    public void setAngle(double angle) {
        if (!bothClampsSet()) {
            TurnMotor.set(0.0); // locked out - do not drive until both clamps are set
            return;
        }
        double clamped = MathUtil.clamp(angle, getClampMin(), getClampMax());
        controllerTurn.setSetpoint(clamped * TurretConstants.kMotorRotationsPerTurretRotation, ControlType.kPosition);
        lastCommandedRotations = clamped;
        atLimit = clamped != angle;
    }

    public double getAngle(){
        return encoder.getPosition() / TurretConstants.kMotorRotationsPerTurretRotation;
    }

    public void stop() {
        TurnMotor.set(0.0);
    }

    /** Plain-English status shown on the dashboard. */
    private String getStatusMessage() {
        if (!clamp1Set && !clamp2Set) {
            return "LOCKED - move turret by hand, click 'Set Clamp' to capture Clamp 1.";
        } else if (clamp1Set && !clamp2Set) {
            return String.format(
                "LOCKED - Clamp 1 = %.1f deg. Move turret, click 'Set Clamp' for Clamp 2.",
                clamp1Rotations * 360.0);
        } else if (!clamp1Set && clamp2Set) {
            return String.format(
                "LOCKED - Clamp 2 = %.1f deg. Click 'Set Clamp' to capture Clamp 1.",
                clamp2Rotations * 360.0);
        } else {
            return String.format(
                "ACTIVE - clamped between %.1f and %.1f deg.",
                getClampMin() * 360.0, getClampMax() * 360.0);
        }
    }

    @Override
    public void periodic() {
        // Handle the "Set Clamp" checkbox: if clicked (true), capture and reset it to false
        // so it behaves like a momentary button.
        if (SmartDashboard.getBoolean(kSetClampKey, false)) {
            captureClamp();
            SmartDashboard.putBoolean(kSetClampKey, false);
        }

        // Status readouts - all plain putX under the "Turret" group.
        SmartDashboard.putString("Turret/Status", getStatusMessage());
        SmartDashboard.putBoolean("Turret/Locked", !bothClampsSet());
        SmartDashboard.putString("Turret/Next Click Sets", nextIsClamp1 ? "Clamp 1" : "Clamp 2");
        SmartDashboard.putBoolean("Turret/Clamp 1 Set", clamp1Set);
        SmartDashboard.putBoolean("Turret/Clamp 2 Set", clamp2Set);
        SmartDashboard.putNumber("Turret/Clamp 1 (deg)", clamp1Rotations * 360.0);
        SmartDashboard.putNumber("Turret/Clamp 2 (deg)", clamp2Rotations * 360.0);
        SmartDashboard.putNumber("Turret/Current Angle (deg)", getAngle() * 360.0);
        SmartDashboard.putNumber("Turret/Commanded Angle (deg)", lastCommandedRotations * 360.0);
        SmartDashboard.putBoolean("Turret/At Limit", atLimit);
    }
}
