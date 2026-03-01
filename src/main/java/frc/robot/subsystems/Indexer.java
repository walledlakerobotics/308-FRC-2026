package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {

  private TalonFX m_motor = new TalonFX(0);

  private DigitalInput m_lightDetectorLeft = new DigitalInput(0);
  private DigitalInput m_lightDetectorRight = new DigitalInput(0);

  public Indexer() {}

  public void run() {
    m_motor.set(0);
  }

  public void stop() {
    m_motor.set(0);
  }

  public boolean leftDetectedBall() {
    return m_lightDetectorLeft.get();
  }

  public boolean rightDetectedBall() {
    return m_lightDetectorRight.get();
  }

  public Command index() {
    return startEnd(this::run, this::stop);
  }

  @Override
  public void periodic() {}
}
