package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ExtenderConstants;

public class Extender extends SubsystemBase {

  private SparkMax m_motor = new SparkMax(ExtenderConstants.kExtenderCanId, MotorType.kBrushless);
  private AbsoluteEncoder m_encoder = m_motor.getAbsoluteEncoder();

  private SparkLimitSwitch m_closedLimitSwitch =
      m_motor.getForwardLimitSwitch(); // I might make this depend on the angle of the motor,
  private SparkLimitSwitch m_openedLimitSwitch =
      m_motor.getReverseLimitSwitch(); // with an absolute encoder.

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

  public boolean isExtendorClosed() {
    return m_closedLimitSwitch.isPressed();
  }

  public boolean isExtendorOpened() {
    return m_openedLimitSwitch.isPressed();
  }

  /**
   * Runs the extendor motor at a set speed until limit switch is triggered.
   *
   * @return Command
   */
  public Command closeExtender() {
    return runOnce(
            () -> {
              m_motor.set(-ExtenderConstants.kExdenterMotorSpeed);
            })
        .andThen(
            () -> {
              Commands.waitUntil(() -> m_closedLimitSwitch.isPressed());
            })
        .finallyDo(
            () -> {
              m_motor.set(0);
            });
  }

  /**
   * Runs the extendor motor at a set speed until limit switch is triggered.
   *
   * @return Command
   */
  public Command openExtender() {

    return runOnce(
            () -> {
              m_motor.set(ExtenderConstants.kExdenterMotorSpeed);
            })
        .andThen(
            () -> {
              Commands.waitUntil(() -> m_openedLimitSwitch.isPressed());
            })
        .finallyDo(
            () -> {
              m_motor.set(0);
            });
  }
}
