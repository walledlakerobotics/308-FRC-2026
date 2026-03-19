package frc.robot.subsystems.shooter.math;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/**
 * Provides methods for calculating shooter speed, angle, and time of flight based on the distance
 * to the target. These methods will use quadratic regression models based on testing data to
 * provide accurate values for hitting targets at various distances. The regression models will be
 * implemented in the methods below once testing data is collected and analyzed.
 */
public class TrajectoryModel {
  private TrajectoryModel() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  /**
   * Calculates the required shooter velocity to hit a target at the specified distance.
   *
   * @param distanceMeters The distance to the target in meters.
   * @param heightMeters The height of the target in meters.
   * @return The required shooter velocity.
   */
  public static final AngularVelocity shooterVelocity(double distanceMeters, double heightMeters) {
    // Quadratic regression of distance vs. speed data from testing.
    throw new UnsupportedOperationException("Not yet implemented");
  }

  /**
   * Calculates the required shooter angle to hit a target at the specified distance.
   *
   * @param distanceMeters The distance to the target in meters.
   * @param heightMeters The height of the target in meters.
   * @return The required shooter angle.
   */
  public static final Angle hoodAngle(double distanceMeters, double heightMeters) {
    // Quadratic regression of distance vs. angle data from testing.
    throw new UnsupportedOperationException("Not yet implemented");
  }

  /**
   * Calculates the time of flight in seconds for a projectile to hit a target at the specified
   * distance.
   *
   * @param distanceMeters The distance to the target in meters.
   * @param heightMeters The height of the target in meters.
   * @return The time of flight in seconds for a projectile to hit a target at the specified
   *     distance.
   */
  public static final double timeOfFlight(double distanceMeters, double heightMeters) {
    // Quadratic regression of distance vs. TOF data from testing.
    throw new UnsupportedOperationException("Not yet implemented");
  }
}
