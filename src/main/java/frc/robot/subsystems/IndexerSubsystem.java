package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import static com.revrobotics.PersistMode.kPersistParameters;
import com.revrobotics.RelativeEncoder;
import static com.revrobotics.ResetMode.kResetSafeParameters;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

/*import frc.robot.LimelightHelpers;*/

public class IndexerSubsystem extends SubsystemBase {

    //create motors
    private final TalonFX IndexerMotor;
    private final SparkMax FeederMotor;

    private final RelativeEncoder FeederEndcoder;

    private final VelocityVoltage IndexervelocityRequest = new VelocityVoltage(0);

    private final SparkClosedLoopController FeederController;

    public IndexerSubsystem() {

        //config motors
        // null when disabled in Constants, so every use below must check for null
        IndexerMotor = IndexerConstants.kEnableIndexerTalon ? new TalonFX(54) : null;

        if (IndexerMotor != null) {
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

        FeederMotor = new SparkMax(53, MotorType.kBrushless);
        FeederEndcoder = FeederMotor.getEncoder();

        //config arm motor
        SparkMaxConfig FeederConfig = new SparkMaxConfig();

        FeederConfig.inverted(false);
        FeederConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);

        FeederConfig.smartCurrentLimit(30);
        FeederConfig.closedLoop.p(0.00025).i(0.000001).d(0.0004);
        FeederMotor.configure(FeederConfig, kResetSafeParameters, kPersistParameters);

        FeederController = FeederMotor.getClosedLoopController();
    }

    /**
     * Drives the Kraken at a specific velocity target.
     * @param targetRPM The desired engine speed in Revolutions Per Minute (Max ~6000 RPM)
     */
    public void setIndexerVelocityRPM(double targetRPM) {
        // Convert RPM to Rotations Per Second (RPS)
        double targetRPS = targetRPM / 60.0;

        // Use the request object to smoothly command the motor
        if (IndexerMotor != null) {
            IndexerMotor.setControl(IndexervelocityRequest.withVelocity(targetRPS));
        }
        SmartDashboard.putNumber("Feeder Speed", FeederEndcoder.getVelocity());
        FeederController.setSetpoint(targetRPM, ControlType.kVelocity);

    }

    public void setJustIndexerVelocityRPM(double targetRPM) {
        // Convert RPM to Rotations Per Second (RPS)
        double targetRPS = targetRPM / 60.0;

        // Use the request object to smoothly command the motor
        if (IndexerMotor != null) {
            IndexerMotor.setControl(IndexervelocityRequest.withVelocity(targetRPS));
            
        }
    }

    public Command SpinIndexer(){
        return this.run(() -> {
            this.setIndexerVelocityRPM(-2000);
        });
    }

    public Command unstuckBalls(){
        return this.runOnce(() -> {
            this.setIndexerVelocityRPM(2000);
        });
    }

    public void Stop(){
        if (IndexerMotor != null) {
            IndexerMotor.stopMotor();
        }
        FeederMotor.stopMotor();
    }
}