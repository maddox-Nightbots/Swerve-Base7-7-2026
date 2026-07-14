package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;

import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.MathUtil;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class HoodSubsystem extends SubsystemBase{
    private final Servo actuatorLeft, actuatorRight;

    @SuppressWarnings("FieldMayBeFinal")
    private ShooterConfig ShooterConfig;

    private double currentPosition = 0.5;
    private double targetPosition = 0.5;
    
    private Time lastUpdateTime = Seconds.of(0);

    private static final Distance kServoLength = Millimeters.of(100);
    private static final LinearVelocity kMaxServoSpeed = Millimeters.of(20).per(Second);
    private static final double kMinPosition = 0.01;
    private static final double kMaxPosition = 0.77;
    private static final double kPositionTolerance = 0.01;


    public HoodSubsystem() 
    {
        File directory = new File(Filesystem.getDeployDirectory(), "shooter.json");
        try {
            ShooterConfig = new ObjectMapper().readValue(directory, ShooterConfig.class);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        actuatorLeft = new Servo(ShooterConfig.actuatorLeftServo);
        actuatorRight = new Servo(ShooterConfig.actuatorRightServo);

        actuatorLeft.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        actuatorRight.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        setPosition(currentPosition);
    }

    public void setPosition(double position) {
        final double clampedPosition = MathUtil.clamp(position, kMinPosition, kMaxPosition);
        actuatorLeft.set(clampedPosition);
        actuatorRight.set(clampedPosition);
        targetPosition = clampedPosition;
    }

    /** Expects a position between 0.0 and 1.0 */
    public Command positionCommand(double position) {
        return runOnce(() -> setPosition(position))
            .andThen(Commands.waitUntil(this::isPositionWithinTolerance));
    }

    public boolean isPositionWithinTolerance() {
        return MathUtil.isNear(targetPosition, currentPosition, kPositionTolerance);
    }

    private void updateCurrentPosition() {
        final Time currentTime = Seconds.of(Timer.getFPGATimestamp());
        final Time elapsedTime = currentTime.minus(lastUpdateTime);
        lastUpdateTime = currentTime;

        if (isPositionWithinTolerance()) {
            currentPosition = targetPosition;
            return;
        }

        final Distance maxDistanceTraveled = kMaxServoSpeed.times(elapsedTime);
        final double maxPercentageTraveled = maxDistanceTraveled.div(kServoLength).in(Value);
        currentPosition = targetPosition > currentPosition
            ? Math.min(targetPosition, currentPosition + maxPercentageTraveled)
            : Math.max(targetPosition, currentPosition - maxPercentageTraveled);
    }

    @Override
    public void periodic() {
        updateCurrentPosition();
    }
}
