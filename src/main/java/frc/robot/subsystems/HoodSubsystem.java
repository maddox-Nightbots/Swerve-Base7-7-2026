package frc.robot.subsystems;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;

import static com.revrobotics.PersistMode.kPersistParameters;
import static com.revrobotics.ResetMode.kResetSafeParameters;

public final class HoodSubsystem extends SubsystemBase{
    private final SparkMax hoodMotor;

    @SuppressWarnings("FieldMayBeFinal")

    private double currentPosition = 0.5;
    private double targetPosition = 0.5;
    
    private static final double kPositionTolerance = 0.01;

    private final SparkClosedLoopController hoodpidController;

    public HoodSubsystem() 
    {
        hoodMotor = new SparkMax(7, MotorType.kBrushless);
        setPosition(currentPosition);

        //config arm motor
        SparkMaxConfig hoodConfig = new SparkMaxConfig();

        hoodConfig.inverted(true);
        hoodConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);

        hoodConfig.smartCurrentLimit(30);
        hoodConfig.closedLoop.p(0.0002).i(0.000001).d(0.0004);
        hoodMotor.configure(hoodConfig, kResetSafeParameters, kPersistParameters);
        hoodpidController = hoodMotor.getClosedLoopController();
    }

    public void setPosition(double position) {
        final double clampedPosition = MathUtil.clamp(position, HoodConstants.HoodDownPosition, HoodConstants.HoodUpPosition);
        hoodpidController.setSetpoint(clampedPosition * Constants.HoodConstants.gearRatio, ControlType.kPosition);
        targetPosition = clampedPosition;
    }

    public boolean isPositionWithinTolerance() {
        return MathUtil.isNear(targetPosition, currentPosition, kPositionTolerance);
    }

    private void updateCurrentPosition() {
        currentPosition = hoodMotor.getEncoder().getPosition();
        if (isPositionWithinTolerance()) {
            currentPosition = targetPosition;
            return;
        }
        SmartDashboard.putNumber("Hood Position", currentPosition);
    }

    @Override
    public void periodic() {
        updateCurrentPosition();
    }
}
