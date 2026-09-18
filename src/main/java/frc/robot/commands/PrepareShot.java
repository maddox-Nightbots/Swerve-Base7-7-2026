package frc.robot.commands;

import java.util.List;
import java.util.function.Supplier;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.ShooterState;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
// import org.photonvision.targeting.PhotonTrackedTarget;

public class PrepareShot extends Command   {

    Supplier<List<PhotonTrackedTarget>> targetSupplier;

    ShooterSubsystem shooter;
    HoodSubsystem hood;

    public PrepareShot(ShooterSubsystem shooter, HoodSubsystem hood, Supplier<List<PhotonTrackedTarget>> targetSupplier) {
        addRequirements(shooter, hood);
        this.shooter = shooter;
        this.hood = hood;
        this.targetSupplier = targetSupplier;
    }

    private double getTagDistanceMeters(List<PhotonTrackedTarget> targetstoAim){
        for (var target: targetstoAim){
            if(target.getFiducialId() == TurretConstants.kHubTagId){

                double distance = Math.abs(PhotonUtils.calculateDistanceToTargetMeters(
                VisionConstants.kRobotToCameraRight.getTranslation().getZ(),
                Units.feetToMeters(6),
                VisionConstants.kRobotToCameraRight.getRotation().getY(),
                Units.degreesToRadians(target.getPitch()) // Vertical angle from camera to target
            ));

            SmartDashboard.putNumber("HubDistance", distance);
            // Calculate distance using PhotonUtils

            return distance;
            }
        }
        return 0.0;
    }


        private ShooterState getShooterState(){
        double hubDistance = getTagDistanceMeters(targetSupplier.get());

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
        distanceToShotMap.put(Inches.of(52.0), new ShooterState(-2500, 0.19));
        distanceToShotMap.put(Inches.of(114.4), new ShooterState(-3000, 0.40));
        distanceToShotMap.put(Inches.of(165.5), new ShooterState(-3550, 0.48));
        // To use it:
        ShooterState currentSetpoints = distanceToShotMap.get(Meters.of(hubDistance));

        return currentSetpoints;
    }

    public boolean isReadyToShoot() {
        return shooter.isVelocityWithinTolerance();
    }

    @Override
    public void execute() {
        final double distanceToHub = getTagDistanceMeters(targetSupplier.get());
        final ShooterState shot = getShooterState();
        shooter.setShooterRPM(shot.rpm);
        hood.setPosition(shot.hoodPosition);
        SmartDashboard.putNumber("Distance to Hub (inches)", distanceToHub);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }

}
