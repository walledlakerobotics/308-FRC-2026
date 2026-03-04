package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.IndexerConstants;

public class Indexer extends SubsystemBase {

  private TalonFX m_motor = new TalonFX(IndexerConstants.kMotorCANId);

  private DigitalInput m_lightDetectorLeft =
      new DigitalInput(IndexerConstants.kLeftLightSensorChannel);
  private DigitalInput m_lightDetectorRight =
      new DigitalInput(IndexerConstants.kRightLightSensorChannel);

  public Indexer() {
    m_motor.getConfigurator().apply(Configs.Indexer.indexConfig);
  }

  public void run() {
    m_motor.set(IndexerConstants.kMotorSpeed);
  }

  public void stop() {
    m_motor.set(0);
  }

  /**
   * this checks if one of the sensors are detecting balls.
   *
   * @return bool
   */
  public boolean isBallDetected() {
    return m_lightDetectorLeft.get() || m_lightDetectorRight.get();
  }

  public boolean isBallNotDetected() {
    return !isBallDetected();
  }

  /**
   * checks if the motors are running or not.
   *
   * @return bool
   */
  public boolean isMotorRunning() {
    return m_motor.get() != 0;
  }

  /**
   * This runs the motor until no balls are detected.
   *
   * @return Command
   */
  public Command index() {
    return runOnce(this::run)
        .andThen(Commands.waitUntil(this::isBallNotDetected))
        .finallyDo(this::stop);
  }

  @Override
  public void periodic() {
    // checks if the motor is already running.
    if (!isMotorRunning()) {
      index();
    }
  }
}
