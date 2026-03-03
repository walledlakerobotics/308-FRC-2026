package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Configs;
import frc.robot.Constants.FieldConstants.ScoringTarget;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.math.TrajectoryModel;
import frc.robot.subsystems.shooter.math.VirtualTarget;
import java.util.function.Supplier;

/** Subsystem for controlling the Fuel shooter. */
public class Shooter extends SubsystemBase {
  TalonFX m_shooterLeader = new TalonFX(ShooterConstants.kShooterLeaderCanId);
  TalonFX m_shooterFollower = new TalonFX(ShooterConstants.kShooterFollowerCanId);

  private final VelocityVoltage m_velocityVoltageControl = new VelocityVoltage(0.0);

  private Supplier<Pose2d> robotPoseSupplier;

  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          new Config(
              null,
              null,
              null,
              state -> {
                SignalLogger.writeString("sysid-test-state-shooter", state.toString());
              }),
          new Mechanism(this::setVoltage, null, this, "shooter"));

  /** Creates a new Shooter. */
  public Shooter(Supplier<Pose2d> robotPoseSupplier) {
    this.robotPoseSupplier = robotPoseSupplier;

    m_shooterLeader.getConfigurator().apply(Configs.Shooter.shooterConfig);
    m_shooterFollower.getConfigurator().apply(Configs.Shooter.shooterConfig);

    m_shooterFollower.setControl(
        new Follower(m_shooterLeader.getDeviceID(), ShooterConstants.kShooterFollowerAlignment));
  }

  /**
   * Sets the shooter motor voltage.
   *
   * @param voltage The desired voltage to apply to the shooter motors.
   */
  public void setVoltage(Voltage voltage) {
    m_shooterLeader.setVoltage(voltage.in(Volts));
    ;
  }

  /**
   * Sets the shooter motor voltage.
   *
   * @param voltage The desired voltage to apply to the shooter motors.
   */
  public void setVoltage(double voltage) {
    m_shooterLeader.setVoltage(voltage);
  }

  /**
   * Sets the shooter velocity.
   *
   * @param velocity The desired shooter velocity.
   */
  public void setVelocity(AngularVelocity velocity) {
    m_shooterLeader.setControl(m_velocityVoltageControl.withVelocity(velocity));
  }

  /**
   * Gets the current shooter velocity in rotations per second.
   *
   * @return The current shooter velocity in rotations per second.
   */
  public AngularVelocity getVelocity() {
    return m_shooterLeader.getVelocity().getValue();
  }

  /**
   * Shoots at the given target by calculating the appropriate shooter speed to hit the target based
   * on the distance to the target and accounting for the robot's movement during the projectile's
   * time of flight.
   *
   * @param target The target position in field coordinates that the shooter should aim at.
   */
  public void shootAt(ScoringTarget target) {
    Translation2d virtualTarget = VirtualTarget.getInstance().getVirtualTarget(target);
    double distanceToVirtualTarget =
        virtualTarget.minus(robotPoseSupplier.get().getTranslation()).getNorm();
    setVelocity(TrajectoryModel.shooterVelocity(distanceToVirtualTarget));
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
