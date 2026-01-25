package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.Configs;

public class Extender {

  private SparkMax m_motor = new SparkMax(0, MotorType.kBrushless);
  private AbsoluteEncoder m_encoder = m_motor.getAbsoluteEncoder();

  public Extender() {
    m_motor.configure(
        Configs.Extender.extenderConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void setSpeed(double speed) {
    m_motor.set(speed);
  }

  public double getPosition() {
    return m_encoder.getPosition();
  }

  public double getVelocity() {
    return m_encoder.getVelocity();
  }
}
