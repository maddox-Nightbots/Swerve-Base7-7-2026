package frc.robot.commands;


import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.TurretSubsystem;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class turretAim extends Command{
    TurretSubsystem turret;
    Supplier<List<PhotonTrackedTarget>> targetSupplier;
    // Raw Pigeon IMU yaw, used to counter-rotate the turret against chassis spin.
    Supplier<Rotation2d> gyroYawSupplier;
    // IMU yaw from the previous loop, so we can measure how far the chassis just turned.
    private Rotation2d lastGyroYaw = new Rotation2d();

    public turretAim(TurretSubsystem turret, Supplier<List<PhotonTrackedTarget>> targetSupplier,
                     Supplier<Rotation2d> gyroYawSupplier){
        addRequirements(turret);
        this.turret = turret;
        this.targetSupplier = targetSupplier;
        this.gyroYawSupplier = gyroYawSupplier;
    }

    @Override
    public void initialize() {
        // Seed the previous-yaw baseline so the first loop's delta isn't a huge jump.
        lastGyroYaw = gyroYawSupplier.get();
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
        // --- Chassis-spin counter-rotation (Pigeon IMU) ---
        // How far the robot rotated since last loop (CCW-positive). Rotation2d.minus()
        // handles the 180/-180 wraparound for us.
        Rotation2d currentGyroYaw = gyroYawSupplier.get();
        double chassisDeltaRotations = currentGyroYaw.minus(lastGyroYaw).getRotations();
        lastGyroYaw = currentGyroYaw;

        // The turret is bolted to the chassis, so a chassis rotation of +delta drags the
        // turret +delta. Subtract it to hold the turret pointed at the field target
        // (robot spins left -> turret commanded right, same amount, same speed).
        double turretRotations = turret.getAngle();
        double target = turretRotations - chassisDeltaRotations;

        // --- Vision fine-aim on the target tag (turret camera) ---
        // Nudge a FRACTION (kAimGain) of the yaw error toward center each loop. Using the
        // full error would overshoot at 50 Hz; the gain makes it ease in and settle.
        double tagYawDegrees = getTagYaw(targetSupplier.get());
        target += TurretConstants.kAimGain * (tagYawDegrees / 360.0);

        // setAngle() enforces the wire-safe travel limits (single source of truth in the
        // subsystem), so a spin that would exceed them holds at the limit instead of winding.
        turret.setAngle(target);

        SmartDashboard.putNumber("Turret Target Angle (degrees)", target * 360.0);
        SmartDashboard.putNumber("Turret Tag Yaw (degrees)", tagYawDegrees);
        SmartDashboard.putNumber("Chassis Delta (degrees)", chassisDeltaRotations * 360.0);
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }
}
