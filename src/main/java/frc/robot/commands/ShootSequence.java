package frc.robot.commands;

import java.util.List;
import java.util.function.Supplier;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class ShootSequence extends SequentialCommandGroup{

    IntakeSubsystem intake;
    IndexerSubsystem indexer;

    ShootSequence(ShooterSubsystem shooter, HoodSubsystem hood, IntakeSubsystem intake, 
    IndexerSubsystem indexer, Supplier<List<PhotonTrackedTarget>> targetSupplier){

        addRequirements(getRequirements());
        this.intake = intake;
        this.indexer = indexer;

        addCommands(
            new PrepareShot(shooter, hood, targetSupplier), 
            intake.IntakeUpDown().repeatedly(),
            Commands.parallel(
                new PrepareShot(shooter, hood, targetSupplier),
                indexer.SpinIndexerShoot().onlyWhile(() -> new PrepareShot(shooter, hood, targetSupplier).isReadyToShoot())
            )
        );
    }
}
