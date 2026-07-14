package frc.robot.subsystems;

//import com.revrobotics.RelativeEncoder;
//import com.revrobotics.spark.SparkBase;
import java.io.File;
import java.io.IOException;
import java.util.List;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase { 
    // create motors
    private final SparkFlex ShooterMotorLeft, ShooterMotorMid,ShooterMotorRight,IndexerMotor,FeederMotor;
    private final SparkClosedLoopController controllerLeft,controllerRight,controllerMid;

    private final List<SparkFlex> motors;

    @SuppressWarnings("FieldMayBeFinal")
    private ShooterConfig ShooterConfig;

    private double targetSpeed;
    @SuppressWarnings("FieldMayBeFinal")
    public ShooterSubsystem() {
    File directory = new File(Filesystem.getDeployDirectory(), "shooter.json");
    try {
      ShooterConfig = new ObjectMapper().readValue(directory, ShooterConfig.class);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }

        ShooterMotorLeft = new SparkFlex(ShooterConfig.shooterMotorLeftID, MotorType.kBrushless);
        ShooterMotorMid = new SparkFlex(ShooterConfig.shooterMotorMidID, MotorType.kBrushless);
        ShooterMotorRight = new SparkFlex(ShooterConfig.shooterMotorRightID, MotorType.kBrushless);
        IndexerMotor = new SparkFlex(ShooterConfig.indexerMotorID, MotorType.kBrushless);
        FeederMotor = new SparkFlex(ShooterConfig.feederMotorID, MotorType.kBrushless);
        controllerLeft = ShooterMotorLeft.getClosedLoopController();
        controllerMid = ShooterMotorMid.getClosedLoopController();
        controllerRight = ShooterMotorRight.getClosedLoopController();

        motors = List.of(ShooterMotorLeft, ShooterMotorMid, ShooterMotorRight);

        SparkFlexConfig shooterConfig = new SparkFlexConfig();
        shooterConfig.inverted(false);
        shooterConfig.smartCurrentLimit(ShooterConfig.shooterCurrentLimit);
        shooterConfig.closedLoop
        .p(ShooterConfig.shooterP)
        .i(ShooterConfig.shooterI)
        .d(ShooterConfig.shooterD)
        .outputRange(ShooterConfig.shooterSpeedMinOutput, ShooterConfig.shooterSpeedMaxOutput);

        SparkFlexConfig ShooterLeftConfig = shooterConfig;
        ShooterLeftConfig.inverted(false);
        SparkFlexConfig ShooterMidConfig = shooterConfig;
        ShooterMidConfig.inverted(false);
        SparkFlexConfig ShooterRightConfig = shooterConfig;
        ShooterRightConfig.inverted(false);
        SparkFlexConfig IndexerConfig = shooterConfig;
        IndexerConfig.inverted(false);
        SparkFlexConfig FeederConfig = shooterConfig;
        FeederConfig.inverted(false);

        ShooterMotorLeft.configure(ShooterLeftConfig, com.revrobotics.ResetMode.kResetSafeParameters,
                com.revrobotics.PersistMode.kPersistParameters);
        ShooterMotorMid.configure(ShooterMidConfig, com.revrobotics.ResetMode.kResetSafeParameters,
                com.revrobotics.PersistMode.kPersistParameters);
        ShooterMotorRight.configure(ShooterRightConfig, com.revrobotics.ResetMode.kResetSafeParameters,
                com.revrobotics.PersistMode.kPersistParameters);
        IndexerMotor.configure(IndexerConfig, com.revrobotics.ResetMode.kResetSafeParameters,
                com.revrobotics.PersistMode.kPersistParameters);
        FeederMotor.configure(FeederConfig, com.revrobotics.ResetMode.kResetSafeParameters,
                com.revrobotics.PersistMode.kPersistParameters);
    }

    public void setFeeder(double value) {
        FeederMotor.set(value);
    }

    public void setIndexer(double value) {
        IndexerMotor.set(value);
    }

    public void setShooter(double value) {
        ShooterMotorLeft.set(value);
        ShooterMotorMid.set(value);
        ShooterMotorRight.set(value);
    }

    public void setShooterRPM(double rpm) {
        controllerLeft.setSetpoint(rpm, ControlType.kVelocity);
        controllerMid.setSetpoint(rpm, ControlType.kVelocity);
        controllerRight.setSetpoint(rpm, ControlType.kVelocity);
        targetSpeed = rpm;
    }

    public boolean isVelocityWithinTolerance() {
        return motors.stream().allMatch(motor -> {
            SparkRelativeEncoder encoder = (SparkRelativeEncoder) motor.getEncoder();
            double rpm = encoder.getVelocity();
            return (Math.abs(rpm - targetSpeed) < ShooterConfig.actuatorVelocityTolerance);
        });
    }

    public void stop() {
        ShooterMotorLeft.set(0);
        ShooterMotorMid.set(0);
        ShooterMotorRight.set(0);
        IndexerMotor.set(0);
        FeederMotor.set(0);
    }
}
