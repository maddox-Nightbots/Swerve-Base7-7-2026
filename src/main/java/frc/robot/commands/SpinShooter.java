package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ShooterSubsystem;

public class SpinShooter extends Command {
    private static final double SHOOTER_RPM = 3000.0;

    private final ShooterSubsystem shooter;

    public SpinShooter(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("Shooter/Spin Command Active", true);
        shooter.setShooterRPM(SHOOTER_RPM);
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
