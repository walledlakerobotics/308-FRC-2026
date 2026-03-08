package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.ExtenderConstants;

public class Extender extends SubsystemBase {
  private SparkMax m_motor = new SparkMax(ExtenderConstants.kExtenderCanId, MotorType.kBrushless);
  private AbsoluteEncoder m_encoder = m_motor.getAbsoluteEncoder();

  private ProfiledPIDController m_pidController =
      new ProfiledPIDController(
          ExtenderConstants.kP,
          ExtenderConstants.kI,
          ExtenderConstants.kD,
          new Constraints(ExtenderConstants.kMaxSpeed, ExtenderConstants.kMaxAcceleration));

  private ArmFeedforward m_feedforward =
      new ArmFeedforward(
          ExtenderConstants.kS,
          ExtenderConstants.kG,
          ExtenderConstants.kV,
          ExtenderConstants.kA,
          Constants.kPeriodSeconds);

  public Extender() {
    m_motor.configure(
        Configs.Extender.extenderConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  // gets the position of the encoder.
  public double getPosition() {
    return m_encoder.getPosition();
  }

  // gets the velocity of the encoder.
  public double getVelocity() {
    return m_encoder.getVelocity();
  }
}
