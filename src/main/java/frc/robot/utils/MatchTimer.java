package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;

public final class MatchTimer {
  private static final MatchTimer instance = new MatchTimer();

  public static MatchTimer getInstance() {
    return instance;
  }

  private EventLoop m_timerLoop = new EventLoop();

  private int m_currentDSMatchTime = -1;
  private double m_lastTickTimestamp = -1;

  private MatchTimer() {
    BooleanEvent tickEvent =
        new BooleanEvent(
            m_timerLoop,
            () -> {
              return getDSMatchTime() != m_currentDSMatchTime;
            });

    tickEvent.rising().ifHigh(this::tick);
  }

  private void tick() {
    m_currentDSMatchTime = getDSMatchTime();
    m_lastTickTimestamp = Timer.getTimestamp();
  }

  public double getMatchTime() {
    TimerDirection direction = getTimerDirection();
    int sign = direction == TimerDirection.kUp ? 1 : -1;

    double currentTimestamp = Timer.getTimestamp();
    double difference = m_lastTickTimestamp - currentTimestamp;

    return m_currentDSMatchTime + sign * difference;
  }

  private int getDSMatchTime() {
    return (int) DriverStation.getMatchTime();
  }

  public TimerDirection getTimerDirection() {
    return DriverStation.getMatchType() == MatchType.None
        ? TimerDirection.kUp
        : TimerDirection.kDown;
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
