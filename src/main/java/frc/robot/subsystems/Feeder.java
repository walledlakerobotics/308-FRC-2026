package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Configs;
import frc.robot.Constants.FeederConstants;

public class Feeder extends SubsystemBase {
  private TalonFX m_motor = new TalonFX(FeederConstants.kFeederCanId);
  private VoltageOut m_voltageControl = new VoltageOut(0.0);
  private VelocityVoltage m_velocityVoltageControl = new VelocityVoltage(0.0);

  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          new Config(
              null,
              null,
              null,
              state -> {
                SignalLogger.writeString("sysid-test-state-feeder", state.toString());
              }),
          new Mechanism(this::setVoltage, null, this, "feeder"));

  public Feeder() {
    m_motor.getConfigurator().apply(Configs.Feeder.feederConfig);
  }

  public void setVoltage(Voltage volts) {
    m_motor.setControl(m_voltageControl.withOutput(volts));
  }

  public void setVelocity(AngularVelocity velocity) {
    m_motor.setControl(m_velocityVoltageControl.withVelocity(velocity));
  }

  public void run() {
    setVelocity(FeederConstants.kFeedingVelocity);
  }

  public void runBack() {
    setVelocity(FeederConstants.kFeedingVelocity);
  }

  public void runIdle() {
    setVelocity(FeederConstants.kIdleVelocity);
  }

  public void stop() {
    setVelocity(RotationsPerSecond.of(0.0));
  }

  public Command feed() {
    return startEnd(this::run, this::stop);
  }

  public Command idlePush() {
    return startEnd(this::runIdle, this::stop);
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
