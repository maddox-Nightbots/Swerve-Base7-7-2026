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


public class turretAim extends Command{
    TurretSubsystem turret;
    Supplier<List<PhotonTrackedTarget>> targetSupplier;
    // Raw Pigeon IMU yaw, used to counter-rotate the turret against chassis spin.
    Supplier<Rotation2d> gyroYawSupplier;

    // IMU yaw from the previous loop, so we can measure how far the chassis just turned.
    private Rotation2d lastGyroYaw = new Rotation2d();
    // Turret encoder reading from the previous loop, so we can measure how far the turret
    // ACTUALLY moved (used to drain the debt below).
    private double lastTurretAngle = 0.0;
    // The counter-rotation the turret still OWES the field target, in turret rotations.
    // Every bit of chassis spin is ADDED here, and it is only DRAINED by how far the turret
    // physically moved. So if the motor lags, this debt keeps growing -> the position error
    // keeps growing -> the motor gets a bigger and bigger signal until it catches up.
    private double pendingChassisRotations = 0.0;
    // Cap the debt so a stalled turret can't wind it up forever (the wrap-around handles
    // anything past half a turn).
    private static final double kMaxPendingRotations = 1.0;

    public turretAim(TurretSubsystem turret, Supplier<List<PhotonTrackedTarget>> targetSupplier,
                     Supplier<Rotation2d> gyroYawSupplier){
        addRequirements(turret);
        this.turret = turret;
        this.targetSupplier = targetSupplier;
        this.gyroYawSupplier = gyroYawSupplier;
    }

    @Override
    public void initialize() {
        // Seed the baselines so the first loop's deltas aren't huge jumps, and clear the debt.
        lastGyroYaw = gyroYawSupplier.get();
        lastTurretAngle = turret.getAngle();
        pendingChassisRotations = 0.0;
    }

    /** @return the target tag's yaw in degrees from the turret camera, or 0 if it isn't seen. */
    private double getTagYaw(List<PhotonTrackedTarget> targetstoAim){
        double yawDegrees = 0.0;
        for (var target: targetstoAim){
            if(target.getFiducialId() == TurretConstants.kTargetTagId){
                yawDegrees = target.getYaw();
            }
        }
        return yawDegrees;
    }

    @Override
    public void execute() {
        // --- 1. Accumulate the chassis spin (Pigeon IMU) into the debt ---
        // How far the robot rotated since last loop (CCW-positive). Rotation2d.minus()
        // handles the 180/-180 wraparound for us.
        Rotation2d currentGyroYaw = gyroYawSupplier.get();
        double chassisDeltaRotations = currentGyroYaw.minus(lastGyroYaw).getRotations();
        lastGyroYaw = currentGyroYaw;

        // The turret is bolted to the chassis, so a +delta spin drags it +delta. It therefore
        // OWES -delta of counter-rotation to stay pointed at the field target. Add to the debt.
        pendingChassisRotations -= chassisDeltaRotations;

        // --- 2. Drain the debt ONLY by how far the turret actually moved ---
        double measured = turret.getAngle();
        double actualMovement = measured - lastTurretAngle;
        lastTurretAngle = measured;
        pendingChassisRotations -= actualMovement;

        // Anti-windup: never let the debt exceed one turn (wrap-around covers the rest).
        pendingChassisRotations = MathUtil.clamp(pendingChassisRotations, -kMaxPendingRotations, kMaxPendingRotations);

        // --- 3. Vision fine-aim (turret camera): FULL proportional correction, re-anchored
        //        to the measured position each loop (the form that settled cleanly). No gain. ---
        double tagYawDegrees = getTagYaw(targetSupplier.get());
        double visionRotations = tagYawDegrees / 360.0;

        // Command = where the turret is + counter-rotation still owed + vision correction.
        double target = measured + pendingChassisRotations + visionRotations;

        // --- 4. Wrap-around: if the target lands more than 180 deg from the middle of the
        //        travel range, the turret would wind the long way. Swap a full turn so it
        //        approaches from the other side (shorter path, avoids over-winding when the
        //        robot keeps spinning in one direction). ---
        double center = (TurretConstants.kMinTurretRotations + TurretConstants.kMaxTurretRotations) / 2.0;
        while (target - center > 0.5) {
            target -= 1.0;
        }
        while (target - center < -0.5) {
            target += 1.0;
        }

        // setAngle() clamps to the wire-safe limits (single source of truth in the subsystem).
        turret.setAngle(target);

        SmartDashboard.putNumber("Turret Target Angle (degrees)", target * 360.0);
        SmartDashboard.putNumber("Turret Tag Yaw (degrees)", tagYawDegrees);
        SmartDashboard.putNumber("Chassis Delta (degrees)", chassisDeltaRotations * 360.0);
        SmartDashboard.putNumber("Turret Pending Debt (degrees)", pendingChassisRotations * 360.0);
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }
}
