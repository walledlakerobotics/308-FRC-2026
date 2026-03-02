package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.AimerConstants;
import frc.robot.Constants.FieldConstants.ScoringTarget;
import frc.robot.subsystems.shooter.math.TrajectoryModel;
import frc.robot.subsystems.shooter.math.VirtualTarget;
import java.util.function.Supplier;

/** Subsystem for controlling the adjustable shooter hood. */
public class Aimer extends SubsystemBase {
  private final SparkMax m_aimerLeader =
      new SparkMax(AimerConstants.kAimerLeaderCanId, MotorType.kBrushed);
  private final SparkMax m_aimerFollower =
      new SparkMax(AimerConstants.kAimerFollowerCanId, MotorType.kBrushed);

  private final AbsoluteEncoder m_aimerEncoder = m_aimerLeader.getAbsoluteEncoder();

  private final PIDController m_controller =
      new PIDController(
          AimerConstants.kAimerPIDConstants.kP,
          AimerConstants.kAimerPIDConstants.kI,
          AimerConstants.kAimerPIDConstants.kD);

  private Supplier<Pose2d> robotPoseSupplier;

  /** Creates a new Aimer */
  public Aimer(Supplier<Pose2d> robotPoseSupplier) {
    this.robotPoseSupplier = robotPoseSupplier;

    m_aimerLeader.configure(
        Configs.Aimer.aimerLeaderConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_aimerFollower.configure(
        Configs.Aimer.aimerFollowerConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  /**
   * Sets the hood angle setpoint.
   *
   * @param angle The desired hood angle.
   */
  public void setAngle(Angle angle) {
    m_controller.setSetpoint(angle.in(Rotations));
  }

  /**
   * Gets the current hood angle.
   *
   * @return The current hood angle.
   */
  public Angle getAngle() {
    return Rotations.of(m_aimerEncoder.getPosition());
  }

  /**
   * Calculates the length of the actuator based on the given hood angle using the law of cosines.
   *
   * @param hoodAngle The angle of the hood, which is used to calculate the actuator length.
   * @return The current length of the actuator in meters.
   */
  public double getActuatorLength(Angle hoodAngle) {
    return Math.sqrt(
        Math.pow(AimerConstants.kDistanceToMountPointMeters, 2)
            + Math.pow(AimerConstants.kRotationRadiusMeters, 2)
            - 2
                * AimerConstants.kDistanceToMountPointMeters
                * AimerConstants.kRotationRadiusMeters
                * Math.cos(hoodAngle.plus(AimerConstants.kActuatorAngleOffset).in(Radians)));
  }

  /**
   * Calculates the current length of the actuator based on the current hood angle using the law of
   * cosines.
   *
   * @return The current length of the actuator in meters.
   */
  public double getActuatorLength() {
    return getActuatorLength(getAngle());
  }

  /**
   * Aims at the given target by calculating the appropriate angle to hit the target based on the
   * distance to the target and accounting for the robot's movement during the projectile's time of
   * flight.
   *
   * @param target The target position in field coordinates that the shooter should aim at.
   */
  public void aimAt(ScoringTarget target) {
    Translation2d virtualTarget = VirtualTarget.getInstance().getVirtualTarget(target);
    double distanceToVirtualTarget =
        virtualTarget.minus(robotPoseSupplier.get().getTranslation()).getNorm();
    setAngle(TrajectoryModel.hoodAngle(distanceToVirtualTarget));
  }

  @Override
  public void periodic() {
    double output = m_controller.calculate(getAngle().in(Rotations));
    m_aimerLeader.set(output);
  }
}
