package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import frc.robot.Constants.MatchTimerConstants;
import java.util.Optional;

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
    int sign = direction == TimerDirection.Up ? 1 : -1;

    double currentTimestamp = Timer.getTimestamp();
    double difference = currentTimestamp - m_lastTickTimestamp;

    return m_currentDSMatchTime + sign * difference;
  }

  public Optional<Alliance> getFirstInactiveAlliance() {
    String gameData = DriverStation.getGameSpecificMessage();
    switch (gameData) {
      case "B":
        return Optional.of(Alliance.Blue);
      case "R":
        return Optional.of(Alliance.Red);
      default:
        return Optional.empty();
    }
  }

  private boolean withinPeriod(double t, double start, double length) {
    return t <= start && t > start - length;
  }

  public MatchPeriod getMatchPeriod() {
    Optional<Alliance> currentAllianceOptional = DriverStation.getAlliance();

    if (currentAllianceOptional.isEmpty()) return MatchPeriod.None;

    Alliance currentAlliance = currentAllianceOptional.get();

    if (DriverStation.isAutonomousEnabled()) return MatchPeriod.Auto;
    if (DriverStation.getMatchType() == MatchType.None) return MatchPeriod.None;

    double periodStart = MatchTimerConstants.kTeleopPeriodSeconds;
    double matchTime = getMatchTime();

    if (withinPeriod(matchTime, periodStart, MatchTimerConstants.kTransitionPeriodSeconds)) {
      return MatchPeriod.Transition;
    }

    periodStart -= MatchTimerConstants.kTransitionPeriodSeconds;

    Optional<Alliance> firstInactiveOptional = getFirstInactiveAlliance();

    if (firstInactiveOptional.isEmpty()) return MatchPeriod.None;

    Alliance firstInactive = firstInactiveOptional.get();

    for (int i = 0; i < MatchTimerConstants.kShiftPeriodAmount; i++) {
      if (withinPeriod(matchTime, periodStart, MatchTimerConstants.kShiftPeriodSeconds)) {
        boolean useFirstInactive = i % 2 == 1;

        if (useFirstInactive) {
          return firstInactive == currentAlliance
              ? MatchPeriod.ShiftActive
              : MatchPeriod.ShiftInactive;
        } else {
          return firstInactive == currentAlliance
              ? MatchPeriod.ShiftInactive
              : MatchPeriod.ShiftActive;
        }
      }

      periodStart -= MatchTimerConstants.kShiftPeriodSeconds;
    }

    if (withinPeriod(matchTime, periodStart, MatchTimerConstants.kEndgamePeriodSeconds)) {
      return MatchPeriod.Endgame;
    }

    return MatchPeriod.None;
  }

  private int getDSMatchTime() {
    return (int) DriverStation.getMatchTime();
  }

  public TimerDirection getTimerDirection() {
    return DriverStation.getMatchType() == MatchType.None ? TimerDirection.Up : TimerDirection.Down;
  }

  public void poll() {
    m_timerLoop.poll();
  }

  public static enum TimerDirection {
    Up,
    Down;
  }

  public static enum MatchPeriod {
    Auto,
    Transition,
    ShiftActive,
    ShiftInactive,
    Endgame,
    None;

    public boolean isHubActive() {
      return this != ShiftInactive;
    }
  }
}
