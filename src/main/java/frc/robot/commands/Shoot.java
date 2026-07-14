// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ShooterState;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
//import static frc.robot.Constants.FuelConstants.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Shoot extends Command {
    /** Creates a new Intake. */

    ShooterSubsystem ShootSubsystem;
    HoodSubsystem HoodSubsystem;

    public Shoot(ShooterSubsystem ShootSubsystem, HoodSubsystem HoodSubsystem) {
        addRequirements(ShootSubsystem, HoodSubsystem);
        this.ShootSubsystem = ShootSubsystem;
        this.HoodSubsystem = HoodSubsystem;
    }

    // Called when the command is initially scheduled. Set the rollers to the
    // appropriate values for intaking
    @Override
    public void initialize() {
        ShooterState shot = new ShooterState(1500,0.5); //values are in here for testing for the tree map
        ShootSubsystem.setShooterRPM(shot.rpm);
        HoodSubsystem.setPosition(shot.hoodPosition);
        ShootSubsystem.setFeeder(0.66);
        ShootSubsystem.setIndexer(0.66);
    }

    // Called every time the scheduler runs while the command is scheduled. This
    // command doesn't require updating any values while running
    @Override
    public void execute() {
        ShooterState shot = new ShooterState(1500,0.5); //values are in here for testing for the tree map
        ShootSubsystem.setShooterRPM(shot.rpm);
        HoodSubsystem.setPosition(shot.hoodPosition);
        ShootSubsystem.setFeeder(0.66);
        ShootSubsystem.setIndexer(0.66);
    }

    // Called once the command ends or is interrupted. Stop the rollers
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}