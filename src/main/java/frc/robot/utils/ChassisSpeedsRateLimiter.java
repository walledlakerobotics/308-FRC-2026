package frc.robot.utils;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ChassisSpeedsRateLimiter {
  private final SlewRateLimiter slewRateLimiterX;
  private final SlewRateLimiter slewRateLimiterY;
  private final SlewRateLimiter slewRateLimiterOmega;

  public ChassisSpeedsRateLimiter(double xRateLimit, double yRateLimit, double omegaRateLimit) {
    slewRateLimiterX = new SlewRateLimiter(xRateLimit);
    slewRateLimiterY = new SlewRateLimiter(yRateLimit);
    slewRateLimiterOmega = new SlewRateLimiter(omegaRateLimit);
  }

  public ChassisSpeeds calculate(ChassisSpeeds input) {
    return new ChassisSpeeds(
        slewRateLimiterX.calculate(input.vxMetersPerSecond),
        slewRateLimiterY.calculate(input.vyMetersPerSecond),
        slewRateLimiterOmega.calculate(input.omegaRadiansPerSecond));
  }
}
