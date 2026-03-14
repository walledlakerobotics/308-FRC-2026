package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.NeutralOut;
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
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private TalonFX m_motor = new TalonFX(IntakeConstants.kIntakeCanId);

  private NeutralOut m_neutralControl = new NeutralOut();
  private VoltageOut m_voltageControl = new VoltageOut(0.0);
  private VelocityVoltage m_velocityVoltageControl = new VelocityVoltage(0.0);

  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          new Config(
              null,
              null,
              null,
              state -> {
                SignalLogger.writeString("sysid-test-state-intake", state.toString());
              }),
          new Mechanism(this::setVoltage, null, this, "intake"));

  public Intake() {
    m_motor.getConfigurator().apply(Configs.Intake.intakeConfig);
  }

  public void setVoltage(Voltage volts) {
    m_motor.setControl(m_voltageControl.withOutput(volts));
  }

  public void setVelocity(AngularVelocity velocity) {
    m_motor.setControl(m_velocityVoltageControl.withVelocity(velocity));
  }

  public void run() {
    setVelocity(IntakeConstants.kIntakeVelocity);
  }

  public void stop() {
    setVelocity(RotationsPerSecond.of(0.0));
  }

  public void coast() {
    m_motor.setControl(m_neutralControl);
  }

  public Command intake() {
    return startEnd(this::run, this::coast);
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

  public void addOrchestra(Orchestra orchestra) {
    orchestra.addInstrument(m_motor);
  }
}
