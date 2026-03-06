package frc.robot.subsystems.shooter.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.ShooterConstants;
import frc.robot.utils.Field;
import frc.robot.utils.Field.Landmark;
import java.util.HashMap;

/**
 * Provides methods for calculating a virtual target position that accounts for the robot's movement
 * during the time of flight of the projectile. This is important for accurately hitting targets
 * while the robot is moving, as the projectile will take some time to reach the target and the
 * robot will likely have moved during that time. The virtual target is calculated by iteratively
 * estimating how far the robot will travel during the time of flight of the projectile and
 * adjusting the target position accordingly until it converges on an accurate virtual target.
 */
public class VirtualTarget {
  private static final VirtualTarget instance = new VirtualTarget();

  private final HashMap<Landmark, Translation2d> virtualTargetCache = new HashMap<>();

  private VirtualTarget() {}

  /**
   * Gets the singleton instance of the VirtualTarget class.
   *
   * @return The singleton instance of the VirtualTarget class.
   */
  public static VirtualTarget getInstance() {
    return instance;
  }

  /** Clears the virtual target cache. */
  public void clear() {
    virtualTargetCache.clear();
  }

  /**
   * Updates the virtual target cache for the scoring targets with the robot pose and robot speeds.
   * This should be called periodically (e.g. in the robot's periodic method) to ensure the virtual
   * targets are updated as the robot moves and its speeds change.
   *
   * @param robotPose The current position of the robot in field coordinates.
   * @param robotSpeeds The current field-relative speeds of the robot in the x and y directions in
   *     meters per second.
   */
  public void update(Pose2d robotPose, ChassisSpeeds robotSpeeds) {
    for (Landmark target : Landmark.values()) {
      Translation2d virtualTarget =
          calculateVirtualTarget(
              target.getTranslation(Field.getAlliance()),
              robotPose,
              robotSpeeds,
              ShooterConstants.kVirtualTargetIterations);
      virtualTargetCache.put(target, virtualTarget);
    }
  }

  /**
   * Gets the virtual target position for the given target. Assumes update has been called recently
   * to populate the cache with the latest virtual target positions.
   *
   * @param target The original target position in field coordinates.
   * @return The virtual target position in field coordinates.
   */
  public Translation2d getVirtualTarget(Landmark target) {
    return virtualTargetCache.getOrDefault(target, target.getTranslation(Field.getAlliance()));
  }

  /**
   * Calculates a virtual target position that accounts for the robot's movement during the time of
   * flight of the projectile. This method iteratively calculates the virtual target by estimating
   * how far the robot will travel during the time of flight of the projectile and adjusting the
   * target position accordingly until it converges on an accurate virtual target.
   *
   * @param target The original target position in field coordinates.
   * @param robotPose The current position of the robot in field coordinates.
   * @param robotSpeeds The current field-relative speeds of the robot in the x and y directions in
   *     meters per second.
   * @param maxIterations The maximum number of iterations to perform when calculating the virtual
   *     target. This can be adjusted to balance accuracy and computation time. Defaults to 5.
   * @return The calculated virtual target position in field coordinates that accounts for the
   *     robot's movement during the time of flight of the projectile.
   */
  public static final Translation2d calculateVirtualTarget(
      Translation2d target, Pose2d robotPose, ChassisSpeeds robotSpeeds, int maxIterations) {
    Translation2d robotTranslation = robotPose.getTranslation();

    Translation2d virtualTarget = target; // virtual target to account for robot's speed

    double targetTOF; // seconds, time for projectile to reach target
    double virtualTargetTOF; // seconds, time for projectile to reach virtual target

    // Iterate a few times to converge on an accurate virtual target
    for (int i = 0; i < maxIterations; i++) {
      double distanceToTarget = target.minus(robotTranslation).getNorm();

      // Calculate how far bot will travel during the time of flight of the projectile
      targetTOF = TrajectoryModel.timeOfFlight(distanceToTarget);
      Translation2d robotDisplacement =
          new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond)
              .times(targetTOF);

      // Calculate the virtual target position by subtracting the robot's displacement to the
      // original target
      virtualTarget = target.minus(robotDisplacement);

      // Calculate how far bot will travel during the time of flight to the virtual target
      double distanceToVirtualTarget = virtualTarget.minus(robotTranslation).getNorm();
      virtualTargetTOF = TrajectoryModel.timeOfFlight(distanceToVirtualTarget);

      // If the time of flight to the virtual target is close enough to the time of flight to the
      // previous target, we can assume the virtual target is accurate enough and break out of the
      // loop
      if (Math.abs(virtualTargetTOF - targetTOF) < 0.1) {
        break;
      }
    }

    return virtualTarget;
  }

  /**
   * Calculates a virtual target position that accounts for the robot's movement during the time of
   * flight of the projectile. This method iteratively calculates the virtual target by estimating
   * how far the robot will travel during the time of flight of the projectile and adjusting the
   * target position accordingly until it converges on an accurate virtual target.
   *
   * @param target The original target position in field coordinates.
   * @param robotPose The current position of the robot in field coordinates.
   * @param robotSpeeds The current field-relative speeds of the robot in the x and y directions in
   *     meters per second.
   * @return The calculated virtual target position in field coordinates that accounts for the
   *     robot's movement during the time of flight of the projectile.
   */
  public static final Translation2d calculateVirtualTarget(
      Translation2d target, Pose2d robotPose, ChassisSpeeds robotSpeeds) {
    return calculateVirtualTarget(target, robotPose, robotSpeeds, 5);
  }
}
