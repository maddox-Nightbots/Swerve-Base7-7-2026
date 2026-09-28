package frc.robot.commands;

import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Everything needed to score, all running at once while the trigger is held:
 * aim the turret, spin the shooter / set the hood, jiggle the intake to push balls in,
 * and feed the indexer once we are aimed and the shooter is up to speed (then keep feeding
 * through small aim/speed wobbles until the trigger is released).
 *
 * <p>This is parallel, not sequential: turretAim and PrepareShot never finish on their own,
 * so in a sequence nothing after the first step would ever run. Each piece uses a different
 * subsystem, so they can all run together. Releasing the trigger ends all of them.
 */
public class ShootSequence extends ParallelCommandGroup{


    public ShootSequence(ShooterSubsystem shooter, HoodSubsystem hood, IntakeSubsystem intake,
    IndexerSubsystem indexer, Supplier<List<PhotonTrackedTarget>> targetSupplier, TurretSubsystem turret, Supplier<Optional<Translation2d>> turretPositionSupplier, Supplier<Optional<Rotation2d>> robotHeadingSupplier, BooleanSupplier intaking){

        this.shooter = shooter;
        this.prepareShot = new PrepareShot(shooter, hood, targetSupplier);
        this.aim = new turretAim(turret, turretPositionSupplier, robotHeadingSupplier);

        addCommands(
            aim,
            prepareShot,
            intake.IntakeUpDown().repeatedly().until(intaking).withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf).asProxy().beforeStarting(new WaitCommand(5)),
            // Strict to START feeding (aimed within 5 deg, hub seen in the last second, shooter
            // within 400 RPM), lenient to KEEP feeding: once started it only stops if the aim
            // is off by more than kKeepFeedingAimDegrees or the flywheel sags more than
            // kKeepFeedingRPMTolerance (balls going through pull it down). Otherwise a
            // one-frame tag dropout or aim flicker would stop the feed mid-volley.
            // updateFeeding() decides WHEN to feed; feedBalls() does the alternating-speed feed.
            indexer.feedBalls(this::updateFeeding)
                .beforeStarting(() -> feeding = false)
                .finallyDo(() -> {
                    feeding = false;
                    SmartDashboard.putBoolean("Shooter/Feeding", false);
                })
        );
    }

    /** The start/keep feeding latch described above. Called once per loop by feedBalls(). */
    private boolean updateFeeding() {
        if (feeding) {
            feeding = aim.aimErrorDegrees() <= kKeepFeedingAimDegrees
                && shooter.isVelocityWithin(kKeepFeedingRPMTolerance);
        } else {
            feeding = aim.ableToShoot() && prepareShot.isReadyToShoot();
        }
        SmartDashboard.putBoolean("Shooter/Feeding", feeding);
        return feeding;
    }

    private static final double kKeepFeedingAimDegrees = 15.0;
    private static final double kKeepFeedingRPMTolerance = 800.0;
    private final ShooterSubsystem shooter;
    private final PrepareShot prepareShot;
    private final turretAim aim;
    private boolean feeding = false;
}
