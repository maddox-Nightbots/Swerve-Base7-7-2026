package frc.robot.commands;


import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.TurretSubsystem;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.MathUtil;


public class turretAim extends Command{
    TurretSubsystem turret;
    Supplier<List<PhotonTrackedTarget>> targetSupplier;

    public turretAim(TurretSubsystem turret, Supplier<List<PhotonTrackedTarget>> targetSupplier){
        addRequirements(turret);
        this.turret = turret;
        this.targetSupplier = targetSupplier;
    }

    private double getHubYaw(double turretAngle, List<PhotonTrackedTarget> targetstoAim){
        double yawDegrees = 0.0;
        for (var target: targetstoAim){
            if(target.getFiducialId() == 6){
                yawDegrees = target.getYaw();
            }
        }
        double targetAngle = turretAngle + yawDegrees / 360.0;
        return MathUtil.clamp(targetAngle, 0.0, 0.25);
    }

    @Override
    public void execute() {
        double angle = getHubYaw(turret.getAngle(), targetSupplier.get());
        turret.setAngle(angle);
        SmartDashboard.putNumber("Turret Target Angle (degrees)", angle * 360.0);
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }
}
