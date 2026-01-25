package frc.robot.utils;

public final class MatchTimer {
  public static enum MatchPeriod {
    kAuto,
    kTransition,
    kShiftActive,
    kShiftInactive,
    kEndgame;

    public boolean isHubActive() {
      return this != kShiftInactive;
    }
  }

  private MatchTimer() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
