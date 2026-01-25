package frc.robot.utils;

import edu.wpi.first.wpilibj.event.EventLoop;

public final class MatchTimer {
  public static final MatchTimer instance = new MatchTimer();

  private MatchTimer() {}

  public static MatchTimer getInstance() {
    return instance;
  }

  private EventLoop m_timerLoop = new EventLoop();

  public void poll() {
    m_timerLoop.poll();
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
