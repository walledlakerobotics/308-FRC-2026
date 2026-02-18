package frc.robot.subsystems.shooter.math;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class VirtualTarget {
  private VirtualTarget() {
    throw new UnsupportedOperationException("This is a utility class!");
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
      Translation2d target, Translation2d robotPose, ChassisSpeeds robotSpeeds, int maxIterations) {
    Translation2d virtualTarget = target; // virtual target to account for robot's speed

    double targetTOF; // seconds, time for projectile to reach target
    double virtualTargetTOF; // seconds, time for projectile to reach virtual target

    // Iterate a few times to converge on an accurate virtual target
    for (int i = 0; i < maxIterations; i++) {
      double distanceToTarget = target.minus(robotPose).getNorm();

      // Calculate how far bot will travel during the time of flight of the projectile
      targetTOF = TrajectoryModel.timeOfFlight(distanceToTarget);
      Translation2d robotDisplacement =
          new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond)
              .times(targetTOF);

      // Calculate the virtual target position by subtracting the robot's displacement to the
      // original target
      virtualTarget = target.minus(robotDisplacement);

      // Calculate how far bot will travel during the time of flight to the virtual target
      double distanceToVirtualTarget = virtualTarget.minus(robotPose).getNorm();
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
      Translation2d target, Translation2d robotPose, ChassisSpeeds robotSpeeds) {
    return calculateVirtualTarget(target, robotPose, robotSpeeds, 5);
  }
}
