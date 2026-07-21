package frc.robot.subsystems;

//import com.revrobotics.RelativeEncoder;
//import com.revrobotics.spark.SparkBase;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;



public class ShooterSubsystem extends SubsystemBase { 
    // create motors
    private final SparkFlex ShooterMotorLeft;
    private final SparkClosedLoopController controllerLeft;


    @SuppressWarnings("FieldMayBeFinal")


    private double targetSpeed;
    @SuppressWarnings("FieldMayBeFinal")
    public ShooterSubsystem() {
    

        ShooterMotorLeft = new SparkFlex(ShooterConstants.shooterMotorLeftID, MotorType.kBrushless);
        controllerLeft = ShooterMotorLeft.getClosedLoopController();


        

        SparkFlexConfig shooterConfig = new SparkFlexConfig();
        shooterConfig.inverted(false);
        shooterConfig.smartCurrentLimit(40);
        shooterConfig.closedLoop
        .p(0.01)
        .i(0.0)
        .d(0.0);

        SparkFlexConfig ShooterLeftConfig = shooterConfig;
        ShooterLeftConfig.inverted(false);

        ShooterMotorLeft.configure(ShooterLeftConfig, com.revrobotics.ResetMode.kResetSafeParameters,
                com.revrobotics.PersistMode.kPersistParameters);
        
    }

    
    public void setShooter(double value) {
        ShooterMotorLeft.set(value);
        
    }

    public void setShooterRPM(double rpm) {
        controllerLeft.setSetpoint(rpm, ControlType.kVelocity);
        targetSpeed = rpm;
    }

    public boolean isVelocityWithinTolerance() {
            SparkRelativeEncoder encoder = (SparkRelativeEncoder) ShooterMotorLeft.getEncoder();
            double rpm = encoder.getVelocity();
            return (MathUtil.isNear(targetSpeed, rpm, 100));
    }

    public void stop() {
        ShooterMotorLeft.set(0);
    }
}
