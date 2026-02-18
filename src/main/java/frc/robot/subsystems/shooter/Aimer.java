package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.AimerConstants;

public class Aimer extends SubsystemBase {
  private final SparkMax m_aimerMotor =
      new SparkMax(AimerConstants.kAimerCanId, MotorType.kBrushless);

  public Aimer() {
    m_aimerMotor.configure(
        Configs.Aimer.aimerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
}
