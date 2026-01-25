package frc.robot.utils;

public final class MatchTimer {
  private MatchTimer() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

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
}
