package frc.robot.commands;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;


public class PassSequence extends SequentialCommandGroup{

    IntakeSubsystem intake;
    IndexerSubsystem indexer;
    TurretSubsystem turret;
    HoodSubsystem hood;

    public PassSequence(ShooterSubsystem shooter, IntakeSubsystem intake, 
    IndexerSubsystem indexer, TurretSubsystem turret, HoodSubsystem hood, Supplier<Optional<Translation2d>> aimVectorSupplier, Supplier<Optional<Rotation2d>> robotHeadingSupplier, DoubleSupplier yawRateSupplier, BooleanSupplier intaking, LightSubsystem lightSubsystem){

        turretAim aim = new turretAim(turret, aimVectorSupplier, robotHeadingSupplier, yawRateSupplier, lightSubsystem);

        addRequirements(getRequirements());
        this.intake = intake;
        this.indexer = indexer;
        this.turret = turret;
        this.hood = hood;

        addCommands(
            Commands.runOnce(() -> hood.setPosition(Constants.HoodConstants.HoodUpPosition)),
            aim,
            new SpinShooter(shooter,indexer).onlyIf(aim::ableToShoot), 
            intake.IntakeUpDown().repeatedly().until(intaking).withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf).asProxy().beforeStarting(new WaitCommand(5)),
            Commands.parallel(
                new SpinShooter(shooter,indexer).onlyIf(aim::ableToShoot),
                Commands.run(() -> hood.setPosition(Constants.HoodConstants.HoodUpPosition))
            )
        );
    }
}
