package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

  private SparkMax m_motor = new SparkMax(IntakeConstants.kIntakeCanId, MotorType.kBrushless);

  public Intake() {
    m_motor.configure(
        Configs.Intake.intakeConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
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
