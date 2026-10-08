package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShootOnMoveConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Field-locked turret aim at the hub center.
 *
 * <p>Every loop: take the vector from the turret to the (virtual, shoot-on-move) hub and the
 * robot's field heading, both from the camera tag solves ({@code VisionSubsystem}; turret or front
 * camera, never the fused robot pose), work out which way the hub is relative to the robot, and
 * command the turret to that angle. Between frames the vision subsystem adds the gyro's turn since
 * the frame, so chassis spin is cancelled right away, and a late or missing frame can't make the
 * turret chase an old error. The aim also leads the chassis' rotation by
 * {@link ShootOnMoveConstants#kTurretYawLeadSeconds} so the turret doesn't lag a spinning robot.
 *
 * <p>Until a camera has seen a tag once, the turret just holds where it is.
 */
public class turretAim extends Command {


    private final TurretSubsystem turret;
    // Turret axis to the hub (shoot-on-move virtual hub) on the field (VisionSubsystem::getAimVector).
    private final Supplier<Optional<Translation2d>> aimVectorSupplier;
    // Robot heading on the field, from the camera tag solves (VisionSubsystem::getRobotHeadingFromTurretCamera).
    private final Supplier<Optional<Rotation2d>> robotHeadingSupplier;
    // Chassis yaw rate, rad/s CCW-positive (VisionSubsystem::getYawRateRadPerSec).
    private final DoubleSupplier yawRateSupplier;

    private final LightSubsystem m_lightSubsystem;
    // True when the turret is on the hub (within kAimToleranceDegrees) and the hub is inside
    // the travel range. Only meaningful while this command is running.
    private boolean aimed = false;
    // How far off the hub the turret is (worst of reach and tracking error), degrees.
    // Infinity when there is no turret fix yet.
    private double aimErrorDeg = Double.POSITIVE_INFINITY;

    public turretAim(TurretSubsystem turret, Supplier<Optional<Translation2d>> aimVectorSupplier,
            Supplier<Optional<Rotation2d>> robotHeadingSupplier, DoubleSupplier yawRateSupplier, LightSubsystem m_lightSubsystem) {
        addRequirements(turret);
        this.turret = turret;
        this.aimVectorSupplier = aimVectorSupplier;
        this.robotHeadingSupplier = robotHeadingSupplier;
        this.yawRateSupplier = yawRateSupplier;
        this.m_lightSubsystem = m_lightSubsystem;   
    }

    @Override
    public void initialize() {
        aimed = false;
        aimErrorDeg = Double.POSITIVE_INFINITY;
    }

    /** Whether the indexer may feed: see {@link #aimed}. */
    public boolean ableToShoot() {
        return aimed;
    }

    /** Degrees the turret is off the hub; infinity with no turret fix. */
    public double aimErrorDegrees() {
        return aimErrorDeg;
    }

    @Override
    public void execute() {
        Optional<Translation2d> maybeToHub = aimVectorSupplier.get();
        Optional<Rotation2d> maybeRobotHeading = robotHeadingSupplier.get();
        if (maybeToHub.isEmpty() || maybeRobotHeading.isEmpty()) {
            // No turret fix yet: hold still rather than aim at a guess.
            turret.setAngle(turret.getAngle());
            aimed = false;
            aimErrorDeg = Double.POSITIVE_INFINITY;
            SmartDashboard.putBoolean("TurretDiag/Aimed", false);
            return;
        }

        // Robot-relative heading from the turret axis to the hub. The robot turning CCW swings the
        // hub CW relative to the robot, so aim where it will be kTurretYawLeadSeconds from now.
        Translation2d toHub = maybeToHub.get();
        Rotation2d yawLead = Rotation2d.fromRadians(yawRateSupplier.getAsDouble() * ShootOnMoveConstants.kTurretYawLeadSeconds);
        Rotation2d hubHeading = toHub.getAngle().minus(maybeRobotHeading.get()).minus(yawLead);

        // No wrap-around: a hub behind the travel range just holds at the nearer limit.
        double targetAngle = turret.angleForHeading(hubHeading);
        turret.setAngle(targetAngle);

        // Error between where the turret is told to go and the hub (non-zero only when the
        // hub is out of range), and between where the turret is and where it's told to go.
        double reachErrorDeg = TurretSubsystem.headingForAngle(targetAngle).minus(hubHeading).getDegrees();
        double trackErrorDeg = (targetAngle - turret.getAngle()) * 360.0;
        aimErrorDeg = Math.max(Math.abs(reachErrorDeg), Math.abs(trackErrorDeg));
        aimed = aimErrorDeg <= TurretConstants.kAimToleranceDegrees;

        SmartDashboard.putNumber("TurretDiag/Aim Target (deg)", targetAngle * 360.0);
        SmartDashboard.putNumber("TurretDiag/Aim Hub Heading (deg)", hubHeading.getDegrees());
        SmartDashboard.putNumber("TurretDiag/Aim Hub Distance (m)", toHub.getNorm());
        SmartDashboard.putNumber("TurretDiag/Aim Out Of Range (deg)", reachErrorDeg);
        SmartDashboard.putNumber("TurretDiag/Aim Tracking Error (deg)", trackErrorDeg);
        SmartDashboard.putBoolean("TurretDiag/Aimed", aimed);

        if (aimed){
            m_lightSubsystem.lightOn(); 
        } else {
            m_lightSubsystem.lightOff();
        }
    }

    @Override
    public void end(boolean interrupted) {
        aimed = false;
        aimErrorDeg = Double.POSITIVE_INFINITY;
        SmartDashboard.putBoolean("TurretDiag/Aimed", false);
        turret.stop();
    }
}
