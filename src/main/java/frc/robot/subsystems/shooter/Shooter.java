package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  TalonFX m_shooterLeader = new TalonFX(ShooterConstants.kShooterLeaderCanId);
  TalonFX m_shooterFollower = new TalonFX(ShooterConstants.kShooterFollowerCanId);

  private final VelocityVoltage m_velocityVoltageControl = new VelocityVoltage(0.0);

  public Shooter() {
    m_shooterLeader.getConfigurator().apply(Configs.Shooter.shooterConfig);
    m_shooterFollower.getConfigurator().apply(Configs.Shooter.shooterConfig);

    m_shooterFollower.setControl(
        new Follower(m_shooterLeader.getDeviceID(), ShooterConstants.kShooterFollowerAlignment));
  }

  public void setVelocity(double velocity) {
    m_shooterLeader.setControl(m_velocityVoltageControl.withVelocity(velocity));
  }

  public double getVelocity() {
    return m_shooterLeader.getVelocity().getValueAsDouble();
  }
}
