package frc.robot.commands;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Everything needed to score, all running at once while the trigger is held:
 * aim the turret, spin the shooter / set the hood, jiggle the intake to push balls in,
 * and feed the indexer ONLY while we are aimed and the shooter is up to speed.
 *
 * <p>This is parallel, not sequential: turretAim and PrepareShot never finish on their own,
 * so in a sequence nothing after the first step would ever run. Each piece uses a different
 * subsystem, so they can all run together. Releasing the trigger ends all of them.
 */
public class ShootSequence extends ParallelCommandGroup{

    private final double kFeedRPM = -4000;

    public ShootSequence(ShooterSubsystem shooter, HoodSubsystem hood, IntakeSubsystem intake,
    IndexerSubsystem indexer, Supplier<List<PhotonTrackedTarget>> targetSupplier, TurretSubsystem turret, Supplier<Rotation2d> gyroYawSupplier, BooleanSupplier intaking, VisionSubsystem vision){

        PrepareShot prepareShot = new PrepareShot(shooter, hood, targetSupplier);

        addCommands(
            new turretAim(turret, targetSupplier, gyroYawSupplier, vision),
            prepareShot,
            intake.IntakeUpDown().repeatedly().until(intaking).withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf).asProxy().beforeStarting(new WaitCommand(5)),
            // Feed only when ready; otherwise hold the balls. setIndexerVelocityRPM(0) stops
            // both the Kraken and the feeder SparkMax.
            Commands.runEnd(
                
            
                () -> indexer.setIndexerVelocityRPM(
                    new turretAim(turret, targetSupplier, gyroYawSupplier, vision).ableToShoot() && prepareShot.isReadyToShoot() ? kFeedRPM : 0),
                () -> indexer.setIndexerVelocityRPM(0),
                indexer)
        );
    }
}
