package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
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

  public void run() {
    m_motor.set(FeederConstants.kfeederspeed);
  }

  public void stop() {
    m_motor.set(0.0);
  }

  public Command feeder() {
    return startEnd(this::run, this::stop);
  }

  /**
   * Generates a SysId command for the shooter subsystem to perform a quasistatic test.
   *
   * @param direction The direction of the quasistatic test (forward or backward).
   * @return The command.
   */
  public Command sysIdQuasiStatic(Direction direction) {
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
