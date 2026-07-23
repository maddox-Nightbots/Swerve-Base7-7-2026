package frc.robot.commands;


import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;


public class turretAim extends Command{
    TurretSubsystem turret;
    List<PhotonTrackedTarget> targets;

    public turretAim(TurretSubsystem turret, List<PhotonTrackedTarget> targets){
        addRequirements(turret);
        this.turret = turret;
        this.targets = targets;
    }

    private double getHubYaw(double turretAngle, List<PhotonTrackedTarget> targetstoAim){
        double angle = 0.0;
        for (var target: targetstoAim){
            if(target.getFiducialId() == (alliance.get() == Alliance.Blue ? 1:0)){
                angle = target.getYaw();
            }
        }
        double targetAngle = turretAngle + angle;
        return targetAngle;
    }
}
