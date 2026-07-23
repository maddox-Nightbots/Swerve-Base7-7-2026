package frc.robot.commands;


import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.TurretSubsystem;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;


public class turretAim extends Command{
    TurretSubsystem turret;
    // A live source of targets (VisionSubsystem::getLatestTargets), read fresh every
    // loop so the turret tracks the tag as the robot/target move.
    Supplier<List<PhotonTrackedTarget>> targetSupplier;

    public turretAim(TurretSubsystem turret, Supplier<List<PhotonTrackedTarget>> targetSupplier){
        addRequirements(turret);
        this.turret = turret;
        this.targetSupplier = targetSupplier;
    }

    /** @return tag 6's yaw in degrees (from the turret camera), or 0 if it isn't seen. */
    private double getTargetYaw(List<PhotonTrackedTarget> targetstoAim){
        Optional<Alliance> alliance = DriverStation.getAlliance();
        // Which AprilTag is the hub for our alliance (both 6 for now / testing). Guard the
        // Optional so we never crash before the driver station reports an alliance.
        int hubTagId = (alliance.isPresent() && alliance.get() == Alliance.Blue) ? 6 : 6;
        for (var target: targetstoAim){
            if(target.getFiducialId() == hubTagId){
                return target.getYaw();
            }
        }
        return 0.0; // tag not seen this frame -> no correction
    }

    @Override
    public void execute() {
        double yaw = getTargetYaw(targetSupplier.get());
        turret.aimAtYaw(yaw);
        SmartDashboard.putNumber("Turret Tag Yaw", yaw);
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }
}
