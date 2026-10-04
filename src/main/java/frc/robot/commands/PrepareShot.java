package frc.robot.commands;

import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
    // FPGA time the hub tag was last seen. The tag drops out for single frames all the time,
    // so lastShot counts as a real distance for kDistanceGraceSeconds after that.
    private double lastSeenTime = Double.NEGATIVE_INFINITY;
    private static final double kDistanceGraceSeconds = 1.0;

    private DoubleSupplier hubDistance;

    public PrepareShot(ShooterSubsystem shooter, HoodSubsystem hood, Supplier<List<PhotonTrackedTarget>> targetSupplier, DoubleSupplier hubDistance) {
        addRequirements(shooter, hood);
        this.shooter = shooter;
        this.hood = hood;
        this.targetSupplier = targetSupplier;
        this.hubDistance = hubDistance;
    }

    /** Distance to the nearest visible tag on our hub (any face), or 0.0 if none is in view. */
    private double getTagDistanceMeters(){
        return hubDistance.getAsDouble();
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
        distanceToShotMap.put(Inches.of(29.92), new ShooterState(-2700, 0.05));
        distanceToShotMap.put(Inches.of(71.05), new ShooterState(-3000, 0.20));
        distanceToShotMap.put(Inches.of(107.90), new ShooterState(-3250, 0.35));
        distanceToShotMap.put(Inches.of(131.32), new ShooterState(-3350, 0.42));
        distanceToShotMap.put(Inches.of(157.40), new ShooterState(-3425, 0.52));
        // To use it:
        ShooterState currentSetpoints = distanceToShotMap.get(Meters.of(hubDistance));

        return currentSetpoints;
    }

    /** True when the hub tag was seen within the last kDistanceGraceSeconds. */
    public boolean hasRecentDistance() {
        return lastShot != null && Timer.getFPGATimestamp() - lastSeenTime <= kDistanceGraceSeconds;
    }

    /** True only when the setpoints come from a recent hub distance AND the shooter is at speed. */
    public boolean isReadyToShoot() {
        return hasRecentDistance() && shooter.isVelocityWithinTolerance();
    }

    @Override
    public void initialize() {
        lastShot = null;
        hasDistance = false;
        lastSeenTime = Double.NEGATIVE_INFINITY;
    }

    @Override
    public void execute() {
        final double distanceToHub = getTagDistanceMeters();
        hasDistance = distanceToHub > 0.0;
        if (hasDistance) {
            lastShot = getShooterState(distanceToHub);
            lastSeenTime = Timer.getFPGATimestamp();
        }
        // Before the hub is first seen, pre-spin at the closest table entry. Nothing feeds
        // until hasDistance is true, so this never fires a ball at the wrong settings.
        final ShooterState shot = lastShot != null ? lastShot : getShooterState(0.0);
        shooter.setShooterRPM(shot.rpm);
        hood.setPosition(shot.hoodPosition);
        SmartDashboard.putNumber("Distance to Hub (inches)", distanceToHub);
        SmartDashboard.putBoolean("Shooter/Hub Distance Known", hasDistance);
        SmartDashboard.putBoolean("Shooter/Hub Distance Recent", hasRecentDistance());
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }

}
