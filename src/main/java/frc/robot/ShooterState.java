package frc.robot;

import edu.wpi.first.math.interpolation.Interpolatable;

public class ShooterState implements Interpolatable<ShooterState> {
    public double rpm;
    public double hoodPosition;

    public ShooterState(double rpm, double hoodPosition) {
        this.rpm = rpm;
        this.hoodPosition =hoodPosition;
    }

    @Override
    public ShooterState interpolate(ShooterState endValue, double t) {
        // 't' is a value from 0 to 1 representing how far we are between points
        return new ShooterState(this.rpm + (endValue.rpm - this.rpm) * t, this.hoodPosition + (endValue.hoodPosition - this.hoodPosition) * t);
    }
}
