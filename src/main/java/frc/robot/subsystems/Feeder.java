package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.FeederConstants;

public class Feeder extends SubsystemBase {

  private TalonFX m_motor = new TalonFX(0); 


  public Feeder() {
    m_motor.getConfigurator().apply(Configs.Feeder.feederConfig);
    }
    public void run(){
      m_motor.set(FeederConstants.kfeederspeed);
    }
    public void stop(){
      m_motor.set(0.0);
    }
    public Command feeder(){
      return startEnd(this::run, this::stop);
    }
}
