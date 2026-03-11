package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Configs;
import frc.robot.Constants.IndexerConstants;

public class Indexer extends SubsystemBase {
  private TalonFX m_motor = new TalonFX(IndexerConstants.kMotorCANId);

  private VoltageOut m_voltageControl = new VoltageOut(0.0);

  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          new Config(
              null,
              null,
              null,
              state -> {
                SignalLogger.writeString("sysid-test-state-indexer", state.toString());
              }),
          new Mechanism(this::setVoltage, null, this, "indexer"));

  private DigitalInput m_lightDetectorLeft =
      new DigitalInput(IndexerConstants.kLeftLightSensorChannel);
  private DigitalInput m_lightDetectorRight =
      new DigitalInput(IndexerConstants.kRightLightSensorChannel);

  public Indexer() {
    m_motor.getConfigurator().apply(Configs.Indexer.indexConfig);
  }

  public void setVoltage(Voltage volts) {
    m_motor.setControl(m_voltageControl.withOutput(volts));
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

  /**
   * This runs the motor until no balls are detected.
   *
   * @return Command
   */
  public Command indexing() {
    return runOnce(this::run)
        .andThen(Commands.waitUntil(this::isBallDetected))
        .finallyDo(this::stop);
  }

  /***
   * Runs the motor until input is stopped.
   *
   * @return Command
   */
  public Command runIndex() {
    return runEnd(this::run, this::stop);
  }

  /**
   * Generates a SysId command for the shooter subsystem to perform a quasistatic test.
   *
   * @param direction The direction of the quasistatic test (forward or backward).
   * @return The command.
   */
  public Command sysIdQuasistatic(Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  /**
   * Generates a SysId command for the shooter subsystem to perform a dynamic test.
   *
   * @param direction The direction of the dynamic test (forward or backward).
   * @return The command.
   */
  public Command sysIdDynamic(Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}
