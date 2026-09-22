package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;



public class ShooterSubsystem extends SubsystemBase { 
    // create motors
    private final SparkFlex ShooterMotorLeft;
    private final SparkClosedLoopController controllerLeft;
    private final RelativeEncoder shooterEncoder;


    @SuppressWarnings("FieldMayBeFinal")


    private double targetSpeed;
    @SuppressWarnings("FieldMayBeFinal")
    public ShooterSubsystem() {
    

        ShooterMotorLeft = new SparkFlex(ShooterConstants.shooterMotorLeftID, MotorType.kBrushless);
        controllerLeft = ShooterMotorLeft.getClosedLoopController();
        shooterEncoder = ShooterMotorLeft.getEncoder();


        

        SparkFlexConfig shooterConfig = new SparkFlexConfig();
        shooterConfig.inverted(false);
        shooterConfig.smartCurrentLimit(40);
        shooterConfig.closedLoop
        .p(0.0015).i(0.00000).d(0.0003).feedForward.kV(0.00018);

        SparkFlexConfig ShooterLeftConfig = shooterConfig;
        ShooterLeftConfig.inverted(false);

        ShooterMotorLeft.configure(ShooterLeftConfig, com.revrobotics.ResetMode.kResetSafeParameters,
                com.revrobotics.PersistMode.kPersistParameters);
        
    }

    
    public void setShooter(double value) {
        ShooterMotorLeft.set(value);
        
    }

    public void setShooterRPM(double rpm) {
        controllerLeft.setSetpoint(-rpm, ControlType.kVelocity);
        targetSpeed = rpm;
    }

    public boolean isVelocityWithinTolerance() {
            // setShooterRPM() commands the motor at -rpm, so the encoder reads negative.
            // Compare against the value actually sent to the motor, not the stored rpm.
            return MathUtil.isNear(-targetSpeed, shooterEncoder.getVelocity(), 400);
    }

    public void stop() {
        ShooterMotorLeft.set(0);
        targetSpeed = 0.0;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/CAN ID", ShooterConstants.shooterMotorLeftID);
        SmartDashboard.putNumber("Shooter/Target RPM", targetSpeed);
        SmartDashboard.putNumber("Shooter/Measured RPM", shooterEncoder.getVelocity());
        SmartDashboard.putBoolean("Shooter/At Target RPM", isVelocityWithinTolerance());
        SmartDashboard.putNumber("Shooter/Applied Output", ShooterMotorLeft.getAppliedOutput());
        SmartDashboard.putNumber("Shooter/Bus Voltage", ShooterMotorLeft.getBusVoltage());
        SmartDashboard.putNumber("Shooter/Output Current Amps", ShooterMotorLeft.getOutputCurrent());
        SmartDashboard.putNumber("Shooter/Motor Temperature C", ShooterMotorLeft.getMotorTemperature());
    }
}
