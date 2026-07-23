package frc.robot.commands;


import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;
import org.photonvision.targeting.PhotonTrackedTarget;


public class turretAim extends Command{
    TurretSubsystem turret;
    List<PhotonTrackedTarget> targets;

    public turretAim(TurretSubsystem turret, List<PhotonTrackedTarget> targets){
        addRequirements(turret);
        this.turret = turret;
        this.targets = targets;
    
    }
}
