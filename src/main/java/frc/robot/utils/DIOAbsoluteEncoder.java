package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;

public class DIOAbsoluteEncoder {
  private DutyCycleEncoder m_dutyCycleEncoder;

  private double m_positionConversionFactor = 1.0;
  private double m_velocityConversionFactor = 1.0;
  private double m_zeroOffset = 0.0;
  private double m_discontinuityPoint = 1.0;

  private volatile double m_lastPosition = 0.0;
  private volatile double m_lastTimestamp = 0.0;

  private Notifier m_notifier;

  public DIOAbsoluteEncoder(DigitalSource source) {
    m_dutyCycleEncoder = new DutyCycleEncoder(source);
    init();
  }

  public DIOAbsoluteEncoder(int channel) {
    m_dutyCycleEncoder = new DutyCycleEncoder(channel);
    init();
  }

  private void init() {
    m_notifier = new Notifier(this::periodic);
    m_notifier.startPeriodic(0.01); // Run every 10 ms
  }

  private void periodic() {
    m_lastPosition = m_dutyCycleEncoder.get();
    m_lastTimestamp = Timer.getFPGATimestamp() / 60.0; // Convert to minutes
  }

  public double getPosition() {
    double zeroedPosition = m_dutyCycleEncoder.get() - m_zeroOffset;
    double wrappedPosition =
        MathUtil.inputModulus(zeroedPosition, m_discontinuityPoint - 1.0, m_discontinuityPoint);

    return wrappedPosition * m_positionConversionFactor;
  }

  public double getVelocity() {
    double currentPosition = m_dutyCycleEncoder.get();
    double currentTime = Timer.getFPGATimestamp() / 60.0; // Convert to minutes

    double positionDelta = currentPosition - m_lastPosition;
    positionDelta = Math.min(positionDelta, 1.0 - positionDelta); // Handle wrap-around

    double timeDelta = currentTime - m_lastTimestamp;

    double unscaledVelocity = positionDelta / timeDelta;

    return unscaledVelocity * m_velocityConversionFactor;
  }

  public int getFrequency() {
    return m_dutyCycleEncoder.getFrequency();
  }

  public int getFPGAIndex() {
    return m_dutyCycleEncoder.getFPGAIndex();
  }

  public int getSourceChannel() {
    return m_dutyCycleEncoder.getSourceChannel();
  }

  public void configure(DIOAbsoluteEncoderConfig config, boolean reset) {
    if (reset) {
      m_dutyCycleEncoder.setInverted(false);
      m_dutyCycleEncoder.setDutyCycleRange(0.0, 1.0);
      m_dutyCycleEncoder.setAssumedFrequency(0.0);

      m_positionConversionFactor = 1.0;
      m_velocityConversionFactor = 1.0;
      m_zeroOffset = 0.0;
      m_discontinuityPoint = 1.0;
    }

    if (config.inverted != null) {
      m_dutyCycleEncoder.setInverted(config.inverted);
    }

    if (config.outputPeriodUs != null && config.minPulseUs != null && config.maxPulseUs != null) {
      m_dutyCycleEncoder.setDutyCycleRange(
          config.minPulseUs / config.outputPeriodUs, config.maxPulseUs / config.outputPeriodUs);
    }

    if (config.assumedFrequency != null) {
      m_dutyCycleEncoder.setAssumedFrequency(config.assumedFrequency);
    }

    m_positionConversionFactor =
        config.positionConversionFactor == null
            ? m_positionConversionFactor
            : config.positionConversionFactor;

    m_velocityConversionFactor =
        config.velocityConversionFactor == null
            ? m_velocityConversionFactor
            : config.velocityConversionFactor;

    if (config.zeroCentered != null) {
      m_discontinuityPoint = config.zeroCentered ? 0.5 : 1.0;
    }

    m_zeroOffset = config.zeroOffset == null ? m_zeroOffset : config.zeroOffset;
  }

  public static class DIOAbsoluteEncoderConfig {
    public Boolean inverted;
    public Double positionConversionFactor;
    public Double velocityConversionFactor;
    public Boolean zeroCentered;
    public Double zeroOffset;
    public Double outputPeriodUs;
    public Double minPulseUs;
    public Double maxPulseUs;
    public Double assumedFrequency;

    public DIOAbsoluteEncoderConfig inverted(boolean inverted) {
      this.inverted = inverted;
      return this;
    }

    public DIOAbsoluteEncoderConfig positionConversionFactor(double positionConversionFactor) {
      this.positionConversionFactor = positionConversionFactor;
      return this;
    }

    public DIOAbsoluteEncoderConfig velocityConversionFactor(double velocityConversionFactor) {
      this.velocityConversionFactor = velocityConversionFactor;
      return this;
    }

    public DIOAbsoluteEncoderConfig zeroCentered(boolean zeroCentered) {
      this.zeroCentered = zeroCentered;
      return this;
    }

    public DIOAbsoluteEncoderConfig zeroOffset(double zeroOffset) {
      this.zeroOffset = zeroOffset;
      return this;
    }

    public DIOAbsoluteEncoderConfig outputPeriodUs(double outputPeriodUs) {
      this.outputPeriodUs = outputPeriodUs;
      return this;
    }

    public DIOAbsoluteEncoderConfig minPulseUs(double minPulseUs) {
      this.minPulseUs = minPulseUs;
      return this;
    }

    public DIOAbsoluteEncoderConfig maxPulseUs(double maxPulseUs) {
      this.maxPulseUs = maxPulseUs;
      return this;
    }

    public DIOAbsoluteEncoderConfig dutyCycleRange(double min, double max) {
      this.minPulseUs = min;
      this.maxPulseUs = max;
      this.outputPeriodUs = 1.0;
      return this;
    }

    public DIOAbsoluteEncoderConfig assumedFrequency(double assumedFrequency) {
      this.assumedFrequency = assumedFrequency;
      return this;
    }

    public DIOAbsoluteEncoderConfig apply(DIOAbsoluteEncoderConfig config) {
      inverted = config.inverted;
      positionConversionFactor = config.positionConversionFactor;
      velocityConversionFactor = config.velocityConversionFactor;
      zeroCentered = config.zeroCentered;
      zeroOffset = config.zeroOffset;
      outputPeriodUs = config.outputPeriodUs;
      minPulseUs = config.minPulseUs;
      maxPulseUs = config.maxPulseUs;
      assumedFrequency = config.assumedFrequency;
      return this;
    }

    public static class Presets {
      private Presets() {
        throw new UnsupportedOperationException("This is a utility class!");
      }

      public static DIOAbsoluteEncoderConfig REV_ThroughBoreEncoder =
          new DIOAbsoluteEncoderConfig()
              .outputPeriodUs(1025.0)
              .minPulseUs(1.0)
              .maxPulseUs(1024.0)
              .assumedFrequency(975.6);

      public static DIOAbsoluteEncoderConfig REV_ThroughBoreEncoderV2 =
          new DIOAbsoluteEncoderConfig()
              .outputPeriodUs(1000.0)
              .minPulseUs(3.884)
              .maxPulseUs(998.06)
              .assumedFrequency(1000.0);

      public static DIOAbsoluteEncoderConfig REV_SplineEncoder =
          new DIOAbsoluteEncoderConfig()
              .outputPeriodUs(1000.0)
              .minPulseUs(3.884)
              .maxPulseUs(998.06)
              .assumedFrequency(1000.0);
    }
  }
}
