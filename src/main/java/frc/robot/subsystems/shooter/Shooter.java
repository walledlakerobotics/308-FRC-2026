package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  TalonFX m_shooterLeader = new TalonFX(ShooterConstants.kShooterLeaderCanId);
  TalonFX m_shooterFollower = new TalonFX(ShooterConstants.kShooterFollowerCanId);

  public Shooter() {
    m_shooterLeader.getConfigurator().apply(Configs.Shooter.shooterConfig);
    m_shooterFollower.getConfigurator().apply(Configs.Shooter.shooterConfig);

    m_shooterFollower.setControl(
        new Follower(m_shooterLeader.getDeviceID(), ShooterConstants.kShooterFollowerAlignment));
  }
}
