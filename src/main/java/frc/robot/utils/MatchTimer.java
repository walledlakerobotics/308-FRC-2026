package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import frc.robot.Constants.MatchConstants;
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
    if (DriverStation.isDisabled()) return MatchPeriod.Disabled;

    if (DriverStation.isAutonomous()) return MatchPeriod.Autonomous;
    if (DriverStation.getMatchType() == MatchType.None) return MatchPeriod.TeleopUnknown;

    double periodStart = MatchConstants.kTeleopPeriodSeconds;
    double matchTime = getMatchTime();

    if (withinPeriod(matchTime, periodStart, MatchConstants.kTransitionPeriodSeconds)) {
      return MatchPeriod.Transition;
    }

    periodStart -= MatchConstants.kTransitionPeriodSeconds;

    Optional<Alliance> firstInactiveOptional = getFirstInactiveAlliance();

    if (firstInactiveOptional.isEmpty()) return MatchPeriod.TeleopUnknown;

    Alliance firstInactive = firstInactiveOptional.get();

    for (int i = 0; i < MatchConstants.kNumShiftPeriods; i++) {
      if (withinPeriod(matchTime, periodStart, MatchConstants.kShiftPeriodSeconds)) {
        return MatchPeriod.getShiftPeriod(firstInactive, i);
      }

      periodStart -= MatchConstants.kShiftPeriodSeconds;
    }

    if (withinPeriod(matchTime, periodStart, MatchConstants.kEndgamePeriodSeconds)) {
      return MatchPeriod.Endgame;
    }

    return MatchPeriod.TeleopUnknown;
  }

  private int getDSMatchTime() {
    double matchTime = DriverStation.getMatchTime();
    TimerDirection timerDirection = getTimerDirection();

    return timerDirection == TimerDirection.Up
        ? (int) Math.floor(matchTime)
        : (int) Math.ceil(matchTime);
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
    Disabled,
    Autonomous,
    TeleopUnknown,
    Transition,
    FirstBlueShift,
    SecondBlueShift,
    FirstRedShift,
    SecondRedShift,
    Endgame;

    public Optional<HubState> getHubState(Alliance alliance) {
      switch (this) {
        case Autonomous:
          return Optional.of(HubState.Active);
        case TeleopUnknown:
          return Optional.empty();
        case Transition:
          return Optional.of(HubState.Active);
        case FirstBlueShift:
        case SecondBlueShift:
          return Optional.of(alliance == Alliance.Blue ? HubState.Active : HubState.Inactive);
        case FirstRedShift:
        case SecondRedShift:
          return Optional.of(alliance == Alliance.Red ? HubState.Active : HubState.Inactive);
        case Endgame:
          return Optional.of(HubState.Active);
        default:
          return Optional.empty();
      }
    }

    public Optional<HubState> getHubState() {
      Optional<Alliance> alliance = DriverStation.getAlliance();

      if (alliance.isPresent()) {
        return getHubState(alliance.get());
      }

      return Optional.empty();
    }

    private static MatchPeriod getShiftPeriod(Alliance firstInactive, int i) {
      Alliance other = firstInactive == Alliance.Blue ? Alliance.Red : Alliance.Blue;
      Alliance activeAlliance = i % 2 == 1 ? firstInactive : other;

      // 0 -> 1, 1 -> 1, 2 -> 2, 3 -> 2
      int periodNum = i / 2 + 1;

      switch (activeAlliance) {
        case Blue:
          return periodNum == 1 ? FirstBlueShift : SecondBlueShift;
        case Red:
          return periodNum == 1 ? FirstRedShift : SecondRedShift;
        default:
          return TeleopUnknown;
      }
    }
  }

  public static enum HubState {
    Active,
    Inactive;
  }
}
