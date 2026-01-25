package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.Configs;

public class Extender {

  private SparkMax m_Motor = new SparkMax(0, MotorType.kBrushless);
  private AbsoluteEncoder m_Encoder = m_Motor.getAbsoluteEncoder();

  public Extender() {
    m_Motor.configure(
        Configs.Extender.extenderConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void setSpeed(double speed) {
    m_Motor.set(speed);
  }

  public double getPosition() {
    return m_Encoder.getPosition();
  }

  public double getVelocity() {
    return m_Encoder.getVelocity();
  }
}
