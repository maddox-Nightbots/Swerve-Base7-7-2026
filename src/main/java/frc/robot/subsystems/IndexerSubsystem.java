package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*import frc.robot.LimelightHelpers;*/

public class IndexerSubsystem extends SubsystemBase {

    //create motors
    private final TalonFX IndexerMotor;


    private final VelocityVoltage IndexervelocityRequest = new VelocityVoltage(0);



    public IndexerSubsystem() {

        //config motors
        IndexerMotor = new TalonFX(5);

        TalonFXConfiguration IndexerConfig = new TalonFXConfiguration();
          // Match the 40A baseline to protect your robot battery during high loads
        IndexerConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        IndexerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        IndexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;


        IndexerConfig.Slot0.kP = 0.11;  // Proportional gain (corrects error)
        IndexerConfig.Slot0.kV = 0.12;  // Feedforward gain (scales voltage to expected velocity)
        IndexerConfig.Slot0.kI = 0.0;   // Integral gain (rarely needed for FRC velocity)
        IndexerConfig.Slot0.kD = 0.0;   // Derivative gain (helps reduce overshoot)

        // Apply configs to Kraken
        IndexerMotor.getConfigurator().apply(IndexerConfig);


    }

    /**
     * Drives the Kraken at a specific velocity target.
     * @param targetRPM The desired engine speed in Revolutions Per Minute (Max ~6000 RPM)
     */
    public void setIndexerVelocityRPM(double targetRPM) {
        // Convert RPM to Rotations Per Second (RPS)
        double targetRPS = targetRPM / 60.0;

        // Use the request object to smoothly command the motor
        IndexerMotor.setControl(IndexervelocityRequest.withVelocity(targetRPS));
    }

    public Command SpinIndexer(){
        return this.run(() -> {
            this.setIndexerVelocityRPM(2000);
        });
    }

    public Command unstuckBalls(){
        return this.runOnce(() -> {
            this.setIndexerVelocityRPM(-2000);
        });
    }

    public void Stop(){
        IndexerMotor.stopMotor();
    }
}