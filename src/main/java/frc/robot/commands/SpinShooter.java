package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Spins the shooter at the dashboard "shooter_rpm" and feeds balls with the indexer
 * (IndexerSubsystem.feedBalls(), alternating speeds) until interrupted.
 */
public class SpinShooter extends ParallelCommandGroup {
    private static final double SHOOTER_RPM = -2500.0;

    private final ShooterSubsystem shooter;

    public SpinShooter(ShooterSubsystem shooter, IndexerSubsystem indexer) {
        this.shooter = shooter;
        SmartDashboard.putNumber("shooter_rpm", SHOOTER_RPM);

        addCommands(
            Commands.startEnd(
                () -> {
                    SmartDashboard.putBoolean("Shooter/Spin Command Active", true);
                    shooter.setShooterRPM(SmartDashboard.getNumber("shooter_rpm", SHOOTER_RPM));
                },
                () -> {
                    shooter.stop();
                    SmartDashboard.putBoolean("Shooter/Spin Command Active", false);
                },
                shooter),
            indexer.feedBalls()
        );
    }

    public boolean isReadyToShoot() {
        return shooter.isVelocityWithinTolerance();
    }
}
