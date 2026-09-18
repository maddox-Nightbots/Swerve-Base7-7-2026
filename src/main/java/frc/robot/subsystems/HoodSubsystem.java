package frc.robot.subsystems;


import static com.revrobotics.PersistMode.kPersistParameters;
import static com.revrobotics.ResetMode.kResetSafeParameters;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;

public final class HoodSubsystem extends SubsystemBase{
    // null when disabled in Constants, so every use below must check for null
    private final SparkMax hoodMotor;

    @SuppressWarnings("FieldMayBeFinal")

    private double currentPosition = 0;
    private double targetPosition = 0.3;

    private static final double kPositionTolerance = 0.01;

    private final SparkClosedLoopController hoodpidController;

    public HoodSubsystem()
    {
        if (!HoodConstants.kEnableHoodMotor) {
            hoodMotor = null;
            hoodpidController = null;
            return;
        } else{

            hoodMotor = new SparkMax(50, MotorType.kBrushless);
             //config arm motor
        SparkMaxConfig hoodConfig = new SparkMaxConfig();

        hoodConfig.inverted(true);
        hoodConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);

        hoodConfig.smartCurrentLimit(20);
        hoodConfig.closedLoop.p(3).i(0.000001).d(0.000);
        hoodMotor.configure(hoodConfig, kResetSafeParameters, kPersistParameters);
        hoodpidController = hoodMotor.getClosedLoopController();
        }

       
    }

    public void setPosition(double position) {
        final double clampedPosition = MathUtil.clamp(position, HoodConstants.HoodDownPosition, HoodConstants.HoodUpPosition);
        if (hoodpidController != null) {
            hoodpidController.setSetpoint(clampedPosition * Constants.HoodConstants.gearRatio, ControlType.kPosition);
        }
        targetPosition = clampedPosition;
    }

    public boolean isPositionWithinTolerance() {
        return MathUtil.isNear(targetPosition, currentPosition, kPositionTolerance);
    }

    private void updateCurrentPosition() {
        // With the motor disabled, pretend we're always on target so nothing waits on the hood.
        if (hoodMotor == null) {
            currentPosition = targetPosition;
            return;
        }
        currentPosition = hoodMotor.getEncoder().getPosition();
        if (isPositionWithinTolerance()) {
            currentPosition = targetPosition;
            return;
        }
        SmartDashboard.putNumber("Hood Position", currentPosition);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Hood Enabled", hoodMotor != null);
        updateCurrentPosition();
    }
}
