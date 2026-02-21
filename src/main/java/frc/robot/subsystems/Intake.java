package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

  private TalonFX m_motor = new TalonFX(IntakeConstants.kIntakeCanId);

  public Intake() {

    m_motor.getConfigurator().apply(Configs.Intake.intakeConfig);
  }

  public void run() {
    m_motor.set(IntakeConstants.kIntakeSpeed);
  }

  public void stop() {
    m_motor.set(0.0);
  }

  public Command intake() {
    return startEnd(this::run, this::stop);
  }
}
