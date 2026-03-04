package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AimerConstants;
import frc.robot.Constants.FieldConstants.ScoringTarget;
import frc.robot.subsystems.shooter.math.TrajectoryModel;
import frc.robot.subsystems.shooter.math.VirtualTarget;
import java.util.function.Supplier;

/** Subsystem for controlling the adjustable shooter hood. */
public class Aimer extends SubsystemBase {
  private final Spark m_aimerMotor = new Spark(AimerConstants.kAimerPWMChannel);

  private final DutyCycleEncoder m_aimerEncoder =
      new DutyCycleEncoder(AimerConstants.kEncoderDIOChannel);

  private final PIDController m_controller =
      new PIDController(AimerConstants.kAimerP, AimerConstants.kAimerI, AimerConstants.kAimerD);

  private Supplier<Pose2d> robotPoseSupplier;

  /** Creates a new Aimer */
  public Aimer(Supplier<Pose2d> robotPoseSupplier) {
    this.robotPoseSupplier = robotPoseSupplier;

    m_aimerMotor.setInverted(AimerConstants.kAimerMotorInverted);
    m_aimerEncoder.setInverted(AimerConstants.kAimerEncoderInverted);

    m_aimerEncoder.setDutyCycleRange(
        AimerConstants.kAimerEncoderDutyCycleMin, AimerConstants.kAimerEncoderDutyCycleMax);

    m_aimerEncoder.setAssumedFrequency(AimerConstants.kAimerEncoderFrequencyHz);
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
    return Rotations.of(m_aimerEncoder.get()).minus(AimerConstants.kAimerEncoderOffset);
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
    m_aimerMotor.setVoltage(output * Constants.kNominalVoltage);
  }
}
