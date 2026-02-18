package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.math.TrajectoryModel;
import frc.robot.subsystems.shooter.math.VirtualTarget;

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

  public double shootAt(Translation2d target, Translation2d robotPose, ChassisSpeeds robotSpeeds) {
    Translation2d virtualTarget =
        VirtualTarget.calculateVirtualTarget(target, robotPose, robotSpeeds);
    double distanceToVirtualTarget = virtualTarget.minus(robotPose).getNorm();
    return TrajectoryModel.shooterSpeedRPS(distanceToVirtualTarget);
  }

  public double shootAt(Translation2d target, Translation2d robotPose) {
    return shootAt(target, robotPose, new ChassisSpeeds());
  }
}
