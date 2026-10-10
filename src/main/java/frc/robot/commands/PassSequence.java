package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class PassSequence extends SequentialCommandGroup{

    IntakeSubsystem intake;
    IndexerSubsystem indexer;
    HoodSubsystem hood;

    /**
     * @param aim the turret's shared aim command (RobotContainer.m_turretAim). Not added to this
     *     group (it is the turret's default command and keeps aiming on its own); only read.
     */
    public PassSequence(ShooterSubsystem shooter, IntakeSubsystem intake,
    IndexerSubsystem indexer, HoodSubsystem hood, turretAim aim, BooleanSupplier intaking){

        addRequirements(getRequirements());
        this.intake = intake;
        this.indexer = indexer;
        this.hood = hood;

        addCommands(
            Commands.runOnce(() -> hood.setPosition(Constants.HoodConstants.HoodUpPosition)),
            new SpinShooter(shooter,indexer).onlyIf(aim::ableToShoot), 
            intake.IntakeUpDown().repeatedly().until(intaking).withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf).asProxy().beforeStarting(new WaitCommand(5)),
            Commands.parallel(
                new SpinShooter(shooter,indexer).onlyIf(aim::ableToShoot),
                Commands.run(() -> hood.setPosition(Constants.HoodConstants.HoodUpPosition))
            )
        );
    }
}
