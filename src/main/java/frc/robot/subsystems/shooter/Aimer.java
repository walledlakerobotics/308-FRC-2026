package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AimerConstants;
import frc.robot.subsystems.shooter.math.TrajectoryModel;
import frc.robot.subsystems.shooter.math.VirtualTarget;

/** Subsystem for controlling the adjustable shooter hood. */
public class Aimer extends SubsystemBase {
  private final Spark m_aimerMotor = new Spark(AimerConstants.kAimerPWMChannel);

  private final DutyCycleEncoder m_aimerEncoder =
      new DutyCycleEncoder(AimerConstants.kEncoderDIOChannel);

  private final PIDController m_controller =
      new PIDController(AimerConstants.kAimerP, AimerConstants.kAimerI, AimerConstants.kAimerD);

  /** Creates a new Aimer */
  public Aimer() {
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
   * @param robotPose The current position of the robot in field coordinates.
   * @param robotSpeeds The current field-relative speeds of the robot in the x and y directions in
   *     meters per second.
   */
  public void aimAt(Translation2d target, Translation2d robotPose, ChassisSpeeds robotSpeeds) {
    Translation2d virtualTarget =
        VirtualTarget.calculateVirtualTarget(target, robotPose, robotSpeeds);
    double distanceToVirtualTarget = virtualTarget.minus(robotPose).getNorm();
    setAngle(TrajectoryModel.hoodAngle(distanceToVirtualTarget));
  }

  /**
   * Aims at the given target by calculating the appropriate angle to hit the target based on the
   * distance to the target. Assumes the robot is stationary.
   *
   * @param target The target position in field coordinates that the shooter should aim at.
   * @param robotPose The current position of the robot in field coordinates.
   */
  public void aimAt(Translation2d target, Translation2d robotPose) {
    aimAt(target, robotPose, new ChassisSpeeds());
  }

  @Override
  public void periodic() {
    double nominalVoltage = 12.0;

    double output = m_controller.calculate(getAngle().in(Rotations));
    m_aimerMotor.setVoltage(output * nominalVoltage);
  }
}
