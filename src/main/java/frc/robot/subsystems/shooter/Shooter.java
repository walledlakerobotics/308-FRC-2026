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

/** Subsystem for controlling the Fuel shooter. */
public class Shooter extends SubsystemBase {
  TalonFX m_shooterLeader = new TalonFX(ShooterConstants.kShooterLeaderCanId);
  TalonFX m_shooterFollower = new TalonFX(ShooterConstants.kShooterFollowerCanId);

  private final VelocityVoltage m_velocityVoltageControl = new VelocityVoltage(0.0);

  /** Creates a new Shooter. */
  public Shooter() {
    m_shooterLeader.getConfigurator().apply(Configs.Shooter.shooterConfig);
    m_shooterFollower.getConfigurator().apply(Configs.Shooter.shooterConfig);

    m_shooterFollower.setControl(
        new Follower(m_shooterLeader.getDeviceID(), ShooterConstants.kShooterFollowerAlignment));
  }

  /**
   * Sets the shooter velocity in rotations per second.
   *
   * @param velocity The desired shooter velocity in rotations per second.
   */
  public void setVelocity(double velocity) {
    m_shooterLeader.setControl(m_velocityVoltageControl.withVelocity(velocity));
  }

  /**
   * Gets the current shooter velocity in rotations per second.
   *
   * @return The current shooter velocity in rotations per second.
   */
  public double getVelocity() {
    return m_shooterLeader.getVelocity().getValueAsDouble();
  }

  /**
   * Shoots at the given target by calculating the appropriate shooter speed to hit the target based
   * on the distance to the target and accounting for the robot's movement during the projectile's
   * time of flight.
   *
   * @param target The target position in field coordinates that the shooter should aim at.
   * @param robotPose The current position of the robot in field coordinates.
   * @param robotSpeeds The current field-relative speeds of the robot in the x and y directions in
   *     meters per second.
   */
  public void shootAt(Translation2d target, Translation2d robotPose, ChassisSpeeds robotSpeeds) {
    Translation2d virtualTarget =
        VirtualTarget.calculateVirtualTarget(target, robotPose, robotSpeeds);
    double distanceToVirtualTarget = virtualTarget.minus(robotPose).getNorm();
    setVelocity(TrajectoryModel.shooterSpeedRPS(distanceToVirtualTarget));
  }

  /**
   * Shoots at the given target by calculating the appropriate shooter speed to hit the target based
   * on the distance to the target. Assumes the robot is stationary.
   *
   * @param target The target position in field coordinates that the shooter should aim at.
   * @param robotPose The current position of the robot in field coordinates.
   */
  public void shootAt(Translation2d target, Translation2d robotPose) {
    shootAt(target, robotPose, new ChassisSpeeds());
  }
}
