package frc.robot.commands;

import java.util.List;
import java.util.function.Supplier;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

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

    // The last shot computed from a real hub distance. Held while the tag flickers out so the
    // shooter doesn't drop to close-range settings; null until the hub tag is first seen.
    private ShooterState lastShot = null;
    // Whether the hub tag was seen this loop, i.e. the current setpoints match a real distance.
    private boolean hasDistance = false;

    public PrepareShot(ShooterSubsystem shooter, HoodSubsystem hood, Supplier<List<PhotonTrackedTarget>> targetSupplier) {
        addRequirements(shooter, hood);
        this.shooter = shooter;
        this.hood = hood;
        this.targetSupplier = targetSupplier;
    }

    private double getTagDistanceMeters(List<PhotonTrackedTarget> targetstoAim){
        for (var target: targetstoAim){
            if(target.getFiducialId() == TurretConstants.kHubTagId){

                double distance = PhotonUtils.calculateDistanceToTargetMeters(
                VisionConstants.kTurretToCamera.getTranslation().getZ(),
                Units.feetToMeters(4.43),
                // PhotonUtils wants pitch UP positive; Rotation3d has pitch up negative.
                -VisionConstants.kTurretToCamera.getRotation().getY(),
                Units.degreesToRadians(target.getPitch()) // Vertical angle from camera to target
            );

            SmartDashboard.putNumber("HubDistance", distance);
            // Calculate distance using PhotonUtils

            return distance;
            }
        }
        return 0.0;
    }


        private ShooterState getShooterState(double hubDistance){

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
        distanceToShotMap.put(Inches.of(41.73), new ShooterState(-2875, 0.15));
        distanceToShotMap.put(Inches.of(84.04), new ShooterState(-3200, 0.40));
        distanceToShotMap.put(Inches.of(102.36), new ShooterState(-3350, 0.45));
        distanceToShotMap.put(Inches.of(120.47), new ShooterState(-3375, 0.47));
        distanceToShotMap.put(Inches.of(138.62), new ShooterState(-3400, 0.48));
        // To use it:
        ShooterState currentSetpoints = distanceToShotMap.get(Meters.of(hubDistance));

        return currentSetpoints;
    }

    /** True only when the setpoints come from a real hub distance AND the shooter is at speed. */
    public boolean isReadyToShoot() {
        return hasDistance && shooter.isVelocityWithinTolerance();
    }

    @Override
    public void initialize() {
        lastShot = null;
        hasDistance = false;
    }

    @Override
    public void execute() {
        final double distanceToHub = getTagDistanceMeters(targetSupplier.get());
        hasDistance = distanceToHub > 0.0;
        if (hasDistance) {
            lastShot = getShooterState(distanceToHub);
        }
        // Before the hub is first seen, pre-spin at the closest table entry. Nothing feeds
        // until hasDistance is true, so this never fires a ball at the wrong settings.
        final ShooterState shot = lastShot != null ? lastShot : getShooterState(0.0);
        shooter.setShooterRPM(shot.rpm);
        hood.setPosition(shot.hoodPosition);
        SmartDashboard.putNumber("Distance to Hub (inches)", distanceToHub);
        SmartDashboard.putBoolean("Shooter/Hub Distance Known", hasDistance);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }

}
