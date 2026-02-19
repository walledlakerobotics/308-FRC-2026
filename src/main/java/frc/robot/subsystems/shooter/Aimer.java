package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AimerConstants;

public class Aimer extends SubsystemBase {
  private final Spark m_aimerMotor = new Spark(AimerConstants.kAimerPWMChannel);

  private final DutyCycleEncoder m_aimerEncoder =
      new DutyCycleEncoder(AimerConstants.kEncoderDIOChannel);

  private final PIDController m_controller =
      new PIDController(1.0, 0.0, 0.0); // PID values will need to be tuned based on testing.

  public Aimer() {
    m_aimerMotor.setInverted(AimerConstants.kAimerMotorInverted);
    m_aimerEncoder.setInverted(AimerConstants.kAimerEncoderInverted);

    m_aimerEncoder.setDutyCycleRange(
        AimerConstants.kAimerEncoderDutyCycleMin, AimerConstants.kAimerEncoderDutyCycleMax);

    m_aimerEncoder.setAssumedFrequency(AimerConstants.kAimerEncoderFrequencyHz);
  }

  public void setAngle(Angle angle) {
    m_controller.setSetpoint(angle.in(Rotations));
  }

  public Angle getAngle() {
    return Rotation.of(m_aimerEncoder.get());
  }

  @Override
  public void periodic() {
    double nominalVoltage = 12.0;

    double output = m_controller.calculate(getAngle().in(Rotations));
    m_aimerMotor.setVoltage(output * nominalVoltage);
  }
}
