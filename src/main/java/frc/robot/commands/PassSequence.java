package frc.robot.commands;

import java.util.List;
import java.util.function.Supplier;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;


public class PassSequence extends SequentialCommandGroup{

    IntakeSubsystem intake;
    IndexerSubsystem indexer;
    TurretSubsystem turret;
    HoodSubsystem hood;

    public PassSequence(ShooterSubsystem shooter, IntakeSubsystem intake, 
    IndexerSubsystem indexer, Supplier<List<PhotonTrackedTarget>> targetSupplier, TurretSubsystem turret, HoodSubsystem hood, Supplier<Rotation2d> gyroYawSupplier, VisionSubsystem vision){

        addRequirements(getRequirements());
        this.intake = intake;
        this.indexer = indexer;
        this.turret = turret;
        this.hood = hood;

        addCommands(
            Commands.runOnce(() -> hood.setPosition(Constants.HoodConstants.HoodUpPosition)),
            new turretAim( turret, targetSupplier, gyroYawSupplier, vision),
            new SpinShooter(shooter,indexer).onlyIf(() -> new turretAim( turret, targetSupplier, gyroYawSupplier, vision).ableToShoot()), 
            intake.IntakeUpDown().repeatedly(),
            
            // Commands.parallel(
            //     new SpinShooter(shooter,indexer).onlyIf(() -> new turretAim( turret, targetSupplier, gyroYawSupplier, vision).ableToShoot()),
            //     indexer.SpinIndexer().onlyWhile(() -> new SpinShooter(shooter,indexer).isReadyToShoot()),
            //     Commands.run(() -> hood.setPosition(Constants.HoodConstants.HoodUpPosition))
            // )
        );
    }
}
