package frc.robot.commands;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.TurretSubsystem;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog.State;

public class turretAim extends Command {
    TurretSubsystem turret;
    static Supplier<List<PhotonTrackedTarget>> targetSupplier;
    // Raw Pigeon IMU yaw, used to counter-rotate the turret against chassis spin.
    Supplier<Rotation2d> gyroYawSupplier;

    // IMU yaw from the previous loop, so we can measure how far the chassis just
    // turned.
    private Rotation2d lastGyroYaw = new Rotation2d();
    // Turret encoder reading from the previous loop, so we can measure how far the
    // turret
    // ACTUALLY moved (used to drain the debt below).
    private double lastTurretAngle = 0.0;
    // The counter-rotation the turret still OWES the field target, in turret
    // rotations.
    // Every bit of chassis spin is ADDED here, and it is only DRAINED by how far
    // the turret
    // physically moved. So if the motor lags, this debt keeps growing -> the
    // position error
    // keeps growing -> the motor gets a bigger and bigger signal until it catches
    // up.
    private double pendingChassisRotations = 0.0;
    // Whether the target tag was seen on the most recent loop (dashboard readout
    // only).
    private static boolean tagVisible = false;
    private static boolean tagVisibleTrench = false;

    private static double  lastTagYaw = 0.0;
    private static double  lastTagYawTrench = 0.0;

    public turretAim(TurretSubsystem turret, Supplier<List<PhotonTrackedTarget>> targetSupplier,
            Supplier<Rotation2d> gyroYawSupplier) {
        addRequirements(turret);
        this.turret = turret;
        turretAim.targetSupplier = targetSupplier;
        this.gyroYawSupplier = gyroYawSupplier;
    }

    @Override
    public void initialize() {
        // Seed the baselines so the first loop's deltas aren't huge jumps, and clear
        // the debt.
        lastGyroYaw = gyroYawSupplier.get();
        lastTurretAngle = turret.getAngle();
        pendingChassisRotations = 0.0;
    }

    /**
     * @return the target tag's yaw in degrees from the turret camera, or 0 if it
     *         isn't seen.
     *         A yaw of 0 also means "already centered", so {@link #tagVisible} is
     *         set here to tell
     *         the two apart on the dashboard. Either way 0 means "no vision
     *         correction this loop".
     */
    private static double getHubTagYaw(List<PhotonTrackedTarget> targetstoAim) {
        double yawDegrees = 0.0;
        tagVisible = false;
        for (var target : targetstoAim) {
            if (target.getFiducialId() == TurretConstants.kHubTagId) {
                yawDegrees = target.getYaw();
                tagVisible = true;
                lastTagYaw = yawDegrees;
            }
        }
        return yawDegrees;
    }

    private static double getTrenchTagYaw(List<PhotonTrackedTarget> targetstoAim) {
        double yawDegrees = 0.0;
        tagVisible = false;
        for (var target : targetstoAim) {
            if (target.getFiducialId() == TurretConstants.kTrenchLeftTagId) {
                yawDegrees = target.getYaw() - 10;
                tagVisibleTrench = true;
            } else if (target.getFiducialId() == TurretConstants.kTrenchRightTagId) {
                yawDegrees = target.getYaw() + 10;
                tagVisibleTrench = true;
                lastTagYawTrench = yawDegrees;
            }
        }
        return yawDegrees;
    }

    public static Boolean ableToShoot() {
        return MathUtil.isNear(0, getHubTagYaw(targetSupplier.get()), 5);
    }
    public static Boolean ableToShootTrench() {
        return MathUtil.isNear(0, getTrenchTagYaw(targetSupplier.get()), 5);
    }

    @Override
    public void execute() {
        // --- 1. Accumulate the chassis spin (Pigeon IMU) into the debt ---
        // How far the robot rotated since last loop (CCW-positive). Rotation2d.minus()
        // handles the 180/-180 wraparound for us.
        Rotation2d currentGyroYaw = gyroYawSupplier.get();
        double chassisDeltaRotations = currentGyroYaw.minus(lastGyroYaw).getRotations();
        lastGyroYaw = currentGyroYaw;

        // The turret is bolted to the chassis, so a +delta spin drags it +delta. It
        // therefore
        // OWES -delta of counter-rotation to stay pointed at the field target. Add to
        // the debt.
        //If the tag is no longer visible it needs to add the last tag yaw, then reset last tag yaw to the pending rotation.
        if(!tagVisible){
            pendingChassisRotations += chassisDeltaRotations + lastTagYaw;
            lastTagYaw = 0.0;
        } else {
            pendingChassisRotations += chassisDeltaRotations;
        }

        // --- 2. Drain the debt ONLY by how far the turret actually moved ---
        double measured = turret.getAngle();
        double actualMovement = measured - lastTurretAngle;
        lastTurretAngle = measured;
        pendingChassisRotations -= actualMovement;

        // Anti-windup: only keep debt the turret can actually pay without leaving the taught
        // clamps. If it sits pinned at a clamp while the robot keeps turning, extra debt would
        // otherwise pile up and later yank the turret across its range.
        pendingChassisRotations = MathUtil.clamp(pendingChassisRotations,
            turret.getClampMin() - measured, turret.getClampMax() - measured);

        // --- 3. Vision fine-aim (turret camera): FULL proportional correction,
        // re-anchored
        // to the measured position each loop (the form that settled cleanly). No gain.
        // ---
        double tagYawDegrees = getHubTagYaw(targetSupplier.get());
        double visionRotations = tagYawDegrees / 360.0;

        // Command = where the turret is + counter-rotation still owed + vision
        // correction.
        double target = measured + pendingChassisRotations + visionRotations;

        // --- 4. Stay inside the taught clamps ---
        // No wrap-around: a target past a clamp just holds at that clamp, it never jumps to
        // the other end of the range. setAngle() does the clamping (single source of truth
        // in the subsystem).
        turret.setAngle(target);

        SmartDashboard.putNumber("TurretDiag/Aim Target (deg)", target * 360.0);
        SmartDashboard.putNumber("TurretDiag/Aim Tag Yaw (deg)", tagYawDegrees);
        SmartDashboard.putNumber("TurretDiag/Aim Chassis Delta (deg)", chassisDeltaRotations * 360.0);
        SmartDashboard.putNumber("TurretDiag/Aim Pending Debt (deg)", pendingChassisRotations * 360.0);
        SmartDashboard.putBoolean("TurretDiag/Aim Tag Visible", tagVisible);
    }

    public void executePass() {
        // --- 1. Accumulate the chassis spin (Pigeon IMU) into the debt ---
        // How far the robot rotated since last loop (CCW-positive). Rotation2d.minus()
        // handles the 180/-180 wraparound for us.
        Rotation2d currentGyroYaw = gyroYawSupplier.get();
        double chassisDeltaRotations = currentGyroYaw.minus(lastGyroYaw).getRotations();
        lastGyroYaw = currentGyroYaw;

        // The turret is bolted to the chassis, so a +delta spin drags it +delta. It
        // therefore
        // OWES -delta of counter-rotation to stay pointed at the field target. Add to
        // the debt.
        if(!tagVisibleTrench){
            pendingChassisRotations += chassisDeltaRotations + lastTagYawTrench;
            lastTagYawTrench = 0.0;
        } else {
            pendingChassisRotations += chassisDeltaRotations;
        }

        // --- 2. Drain the debt ONLY by how far the turret actually moved ---
        double measured = turret.getAngle();
        double actualMovement = measured - lastTurretAngle;
        lastTurretAngle = measured;
        pendingChassisRotations -= actualMovement;

        // Anti-windup: only keep debt the turret can actually pay without leaving the taught
        // clamps. If it sits pinned at a clamp while the robot keeps turning, extra debt would
        // otherwise pile up and later yank the turret across its range.
        pendingChassisRotations = MathUtil.clamp(pendingChassisRotations,
            turret.getClampMin() - measured, turret.getClampMax() - measured);

        // --- 3. Vision fine-aim (turret camera): FULL proportional correction,
        // re-anchored
        // to the measured position each loop (the form that settled cleanly). No gain.
        // ---
        double tagYawDegrees = getTrenchTagYaw(targetSupplier.get());
        double visionRotations = tagYawDegrees / 360.0;

        // Command = where the turret is + counter-rotation still owed + vision
        // correction.
        double target = measured + pendingChassisRotations + visionRotations;

        // --- 4. Stay inside the taught clamps ---
        // No wrap-around: a target past a clamp just holds at that clamp, it never jumps to
        // the other end of the range. setAngle() does the clamping (single source of truth
        // in the subsystem).
        turret.setAngle(target);

        SmartDashboard.putNumber("TurretDiag/Aim Target (deg)", target * 360.0);
        SmartDashboard.putNumber("TurretDiag/Aim Tag Yaw (deg)", tagYawDegrees);
        SmartDashboard.putNumber("TurretDiag/Aim Chassis Delta (deg)", chassisDeltaRotations * 360.0);
        SmartDashboard.putNumber("TurretDiag/Aim Pending Debt (deg)", pendingChassisRotations * 360.0);
        SmartDashboard.putBoolean("TurretDiag/Aim Tag Visible", tagVisible);
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }
}
