package frc.robot.commands;


import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.TurretSubsystem;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;


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
        Optional<Alliance> alliance = DriverStation.getAlliance();
        for (var target: targetstoAim){
            if(target.getFiducialId() == (alliance.get() == Alliance.Blue ? 1:0)){
                angle = target.getYaw();
            }
        }
        double targetAngle = turretAngle + angle/360;
        return MathUtil.clamp(targetAngle, 0.0, 0.25);
    }

    @Override
    public void execute() {
        double angle = getHubYaw(turret.getAngle(), targets);
        turret.setAngle(angle);
        SmartDashboard.putNumber("Distance to Hub (inches)", angle);
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }
}
