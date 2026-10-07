package frc.robot.subsystems;


import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LightSubsystem extends SubsystemBase{

    private final PowerDistribution pdh = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);

    public void lightOn(){
        pdh.setSwitchableChannel(true);
        System.out.println("Light on");
    }

    
    public void lightOff(){
        pdh.setSwitchableChannel(false);
        System.out.println("Light off");
    }

    public void lightToggle(){
        boolean isOn = pdh.getSwitchableChannel();  //toggle between light on and off

        if (isOn){
            this.lightOff();
        } else {
            this.lightOn();
        }
    }

    
    public boolean lightState(){
        return pdh.getSwitchableChannel();
    }


}