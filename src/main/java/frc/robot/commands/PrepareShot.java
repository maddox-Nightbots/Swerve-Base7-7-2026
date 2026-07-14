package frc.robot.commands;

//import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SelectHub;
import frc.robot.ShooterState;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class PrepareShot extends Command   {

    ShooterSubsystem shooter;
    HoodSubsystem hood;
    private final Supplier<Pose2d> robotPoseSupplier;

    public PrepareShot(ShooterSubsystem shooter, HoodSubsystem hood, Supplier<Pose2d> robotPoseSupplier) {
        addRequirements(shooter, hood);
        this.shooter = shooter;
        this.hood = hood;
        this.robotPoseSupplier = robotPoseSupplier;
    }

    private Distance getDistanceToHub() {
        final Translation2d robotPosition = robotPoseSupplier.get().getTranslation();
            return Meters.of(robotPosition.getDistance(SelectHub.hubPosition()));
    } 


        private ShooterState getShooterState(){
        Distance hubDistance = getDistanceToHub();

        // Distance (meters) -> ShooterState (RPM & Angle)
        final InterpolatingTreeMap<Distance, ShooterState> distanceToShotMap = new InterpolatingTreeMap<>(
        (startValue, endValue, q) -> 
            InverseInterpolator.forDouble()
                .inverseInterpolate(startValue.in(Meters), endValue.in(Meters), q.in(Meters)),
        (startValue, endValue, t) ->
            new ShooterState(
                Interpolator.forDouble()
                    .interpolate(startValue.rpm, endValue.rpm, t),
                Interpolator.forDouble()
                    .interpolate(startValue.hoodPosition, endValue.hoodPosition, t)
            )
    );
        // Add your calibration data
        // Distance, new ShooterState(RPM, PivotPosition)
        distanceToShotMap.put(Inches.of(52.0), new ShooterState(2800, 0.19));
        distanceToShotMap.put(Inches.of(114.4), new ShooterState(3275, 0.40));
        distanceToShotMap.put(Inches.of(165.5), new ShooterState(3650, 0.48));
        // To use it:
        ShooterState currentSetpoints = distanceToShotMap.get(hubDistance);

        return currentSetpoints;
    }

    public boolean isReadyToShoot() {
        return shooter.isVelocityWithinTolerance() && hood.isPositionWithinTolerance();
    }

    @Override
    public void execute() {
        final Distance distanceToHub = getDistanceToHub();
        final ShooterState shot = getShooterState();
        shooter.setShooterRPM(shot.rpm);
        hood.setPosition(shot.hoodPosition);
        SmartDashboard.putNumber("Distance to Hub (inches)", distanceToHub.in(Inches));
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }

}
