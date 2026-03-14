package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.shooter.math.TrajectoryModel;
import frc.robot.subsystems.shooter.math.VirtualTarget;
import frc.robot.utils.BangBangController;
import frc.robot.utils.Field.Landmark;
import java.util.function.Supplier;

/** Subsystem for controlling the adjustable shooter hood. */
@Logged
public class Hood extends SubsystemBase {
  private final SparkMax m_hoodLeader =
      new SparkMax(HoodConstants.kHoodLeaderCanId, MotorType.kBrushed);
  private final SparkMax m_hoodFollower =
      new SparkMax(HoodConstants.kHoodFollowerCanId, MotorType.kBrushed);

  private final AbsoluteEncoder m_hoodEncoder = m_hoodLeader.getAbsoluteEncoder();

  private final BangBangController m_controller = new BangBangController(0.01);

  private Supplier<Pose2d> robotPoseSupplier;

  /** Creates a new Hood */
  public Hood(Supplier<Pose2d> robotPoseSupplier) {
    this.robotPoseSupplier = robotPoseSupplier;

    m_hoodLeader.configure(
        Configs.Hood.hoodLeaderConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_hoodFollower.configure(
        Configs.Hood.hoodFollowerConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    setAngle(getAngle());
  }

  /**
   * Sets the hood angle setpoint.
   *
   * @param angle The desired hood angle.
   */
  public void setAngle(Angle angle) {
    System.out.println(angle.in(Rotations));
    m_controller.setSetpoint(angle.in(Rotations));
  }

  /**
   * Gets the current hood angle.
   *
   * @return The current hood angle.
   */
  public Angle getAngle() {
    return Rotations.of(m_hoodEncoder.getPosition());
  }

  /**
   * Calculates the length of the actuator based on the given hood angle using the law of cosines.
   *
   * @param hoodAngle The angle of the hood, which is used to calculate the actuator length.
   * @return The current length of the actuator in meters.
   */
  public double getActuatorLength(Angle hoodAngle) {
    return Math.sqrt(
        Math.pow(HoodConstants.kDistanceToMountPointMeters, 2)
            + Math.pow(HoodConstants.kRotationRadiusMeters, 2)
            - 2
                * HoodConstants.kDistanceToMountPointMeters
                * HoodConstants.kRotationRadiusMeters
                * Math.cos(hoodAngle.plus(HoodConstants.kActuatorAngleOffset).in(Radians)));
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
  public void aimAt(Landmark target) {
    Translation2d virtualTarget = VirtualTarget.getInstance().getVirtualTarget(target);
    double distanceToVirtualTarget =
        virtualTarget.minus(robotPoseSupplier.get().getTranslation()).getNorm();
    setAngle(TrajectoryModel.hoodAngle(distanceToVirtualTarget));
  }

  @Override
  public void periodic() {
    double output = m_controller.calculate(getAngle().in(Rotations));
    m_hoodLeader.set(output);
  }
}
