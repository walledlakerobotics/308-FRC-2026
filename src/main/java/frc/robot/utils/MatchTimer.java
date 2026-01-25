package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.event.EventLoop;

public final class MatchTimer {
  public static final MatchTimer instance = new MatchTimer();

  private MatchTimer() {}

  public static MatchTimer getInstance() {
    return instance;
  }

  private EventLoop m_timerLoop = new EventLoop();

  public TimerDirection getTimerDirection() {
    return DriverStation.getMatchType() == MatchType.None ? TimerDirection.kUp : TimerDirection.kDown;
  }

  public void poll() {
    m_timerLoop.poll();
  }

  public static enum TimerDirection {
    kUp,
    kDown;
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
