package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class TurretSubsystem extends SubsystemBase{

    private final SparkMax TurnMotor;
    private final SparkClosedLoopController controllerTurn;

    public TurretSubsystem() {
        TurnMotor = new SparkMax(ShooterConstants.TurnMotorID, MotorType.kBrushless);
        controllerTurn = TurnMotor.getClosedLoopController();

        SparkMaxConfig ShooterConstants = new SparkMaxConfig();
        ShooterConstants.inverted(false);
        ShooterConstants.smartCurrentLimit(40);
        ShooterConstants.closedLoop
        .p(0.01)
        .i(0.0)
        .d(0.0);

        SparkMaxConfig TurnConfig = ShooterConstants;
        TurnConfig.inverted(false);

        TurnMotor.configure(TurnConfig, com.revrobotics.ResetMode.kResetSafeParameters,
                com.revrobotics.PersistMode.kPersistParameters);
    }

    public void setAngle(double angle) {
        controllerTurn.setSetpoint(angle * ShooterConstants.gearRatio, ControlType.kPosition);
    }

    public Command TargetHub(){
        return(
            this.run(()->{
                
            })
        );
    }

    public void stop() {
        TurnMotor.set(0.0);
    }
}
