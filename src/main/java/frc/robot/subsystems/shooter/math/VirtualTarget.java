package frc.robot.subsystems.shooter.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

  private final HashMap<Translation3d, Translation3d> virtualTargetCache = new HashMap<>();

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
   * Calculates a virtual target position that accounts for the robot's movement during the time of
   * flight of the projectile. This method iteratively calculates the virtual target by estimating
   * how far the robot will travel during the time of flight of the projectile and adjusting the
   * target position accordingly until it converges on an accurate virtual target.
   *
   * @param target The original target position in field coordinates.
   * @param robotPose The current position of the robot in field coordinates.
   * @param robotSpeeds The current field-relative speeds of the robot in the x and y directions in
   *     meters per second.
   * @param iterations The number of iterations to perform when calculating the virtual target. This
   *     can be adjusted to balance accuracy and computation time. Defaults to 5.
   * @return The calculated virtual target position in field coordinates that accounts for the
   *     robot's movement during the time of flight of the projectile.
   */
  public final Translation3d calculateVirtualTarget(
      Translation3d target, Pose2d robotPose, ChassisSpeeds robotSpeeds, int iterations) {
    if (virtualTargetCache.containsKey(target)) {
      return virtualTargetCache.get(target);
    }

    Translation2d robotTranslation = robotPose.getTranslation();

    Translation2d target2d = target.toTranslation2d();
    Translation2d virtualTarget = target2d;

    for (int i = 0; i < iterations; i++) {
      double distance = robotTranslation.getDistance(virtualTarget);
      double timeOfFlight = TrajectoryModel.timeOfFlight(distance);

      double deltaX = robotSpeeds.vxMetersPerSecond * timeOfFlight;
      double deltaY = robotSpeeds.vyMetersPerSecond * timeOfFlight;
      Translation2d robotDelta = new Translation2d(deltaX, deltaY);

      virtualTarget = target2d.minus(robotDelta);
    }

    Translation3d virtualTarget3d =
        new Translation3d(virtualTarget.getX(), virtualTarget.getY(), target.getZ());

    return virtualTarget3d;
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
  public final Translation3d calculateVirtualTarget(
      Translation3d target, Pose2d robotPose, ChassisSpeeds robotSpeeds) {
    return calculateVirtualTarget(target, robotPose, robotSpeeds, 5);
  }
}
