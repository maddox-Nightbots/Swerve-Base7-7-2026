package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SpinShooter extends Command {
    private static final double SHOOTER_RPM = -3000.0;

    private final ShooterSubsystem shooter;
    private final IndexerSubsystem indexer;

    public SpinShooter(ShooterSubsystem shooter, IndexerSubsystem indexer) {
        this.shooter = shooter;
        this.indexer = indexer;
        addRequirements(shooter, indexer);
    }

    public boolean isReadyToShoot() {
        return shooter.isVelocityWithinTolerance();
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("Shooter/Spin Command Active", true);
        shooter.setShooterRPM(SHOOTER_RPM);
        indexer.setIndexerVelocityRPM(-4000);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        SmartDashboard.putBoolean("Shooter/Spin Command Active", false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
