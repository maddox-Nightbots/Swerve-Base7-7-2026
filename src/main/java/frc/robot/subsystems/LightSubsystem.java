package frc.robot.subsystems;


import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LightSubsystem extends SubsystemBase{

    private final PowerDistribution pdh = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);

    // Last state sent to the PDH. turretAim calls lightOn/lightOff every loop, so only
    // send (and print) when the state actually changes.
    private boolean isOn = false;

    public LightSubsystem(){
        pdh.setSwitchableChannel(false); // match isOn
    }

    public void lightOn(){
        if (isOn){
            return;
        }
        pdh.setSwitchableChannel(true);
        isOn = true;
    }


    public void lightOff(){
        if (!isOn){
            return;
        }
        pdh.setSwitchableChannel(false);
        isOn = false;
    }

    public void lightToggle(){
        if (isOn){  //toggle between light on and off
            this.lightOff();
        } else {
            this.lightOn();
        }
    }


    public boolean lightState(){
        return isOn;
    }


}
