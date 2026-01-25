package frc.robot.utils;

public final class MatchTimer {
  public static final MatchTimer instance = new MatchTimer();

  private MatchTimer() {}

  public static MatchTimer getInstance() {
    return instance;
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
