package frc.robot;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ExtenderConstants;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ShooterConstants;

/** Contains configuration objects for various robot components. */
public final class Configs {
  public static final class SwerveModule {
    public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

    public static final MagnetSensorConfigs turningEncoderConfig = new MagnetSensorConfigs();

    static {
      // Use module constants to calculate conversion factors and feed forward gain.
      double drivingFactor =
          ModuleConstants.kWheelCircumferenceMeters / ModuleConstants.kDrivingMotorReduction;
      double turningFactor = 1.0 / ModuleConstants.kTurningMotorReduction;

      double drivingVelocityFeedForward =
          1.0
              / (ModuleConstants.kDrivingMotor.KvRadPerSecPerVolt
                  * ModuleConstants.kWheelRadiusMeters);

      drivingConfig
          .inverted(ModuleConstants.kDrivingMotorsInverted)
          .idleMode(ModuleConstants.kDrivingMotorIdleMode)
          .smartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit)
          .voltageCompensation(ModuleConstants.kDrivingMotor.nominalVoltageVolts);

      drivingConfig
          .encoder
          .positionConversionFactor(drivingFactor) // meters
          .velocityConversionFactor(drivingFactor / 60.0); // meters per second

      drivingConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(0.2, 0, 0)
          .outputRange(-1.0, 1.0)
          .feedForward
          .sva(0.2, drivingVelocityFeedForward, 0.0);

      turningConfig
          .inverted(ModuleConstants.kTurningMotorsInverted)
          .idleMode(ModuleConstants.kTurningMotorIdleMode)
          .smartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit)
          .voltageCompensation(ModuleConstants.kTurningMotor.nominalVoltageVolts);

      turningConfig
          .encoder
          .positionConversionFactor(turningFactor) // rotations
          .velocityConversionFactor(turningFactor / 60.0); // rotations per second

      turningConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(2.0, 0, 0)
          .outputRange(-1.0, 1.0)
          .positionWrappingEnabled(true)
          .positionWrappingInputRange(-0.5, 0.5);

      turningEncoderConfig
          .withSensorDirection(ModuleConstants.kTurningEncoderDirection)
          .withAbsoluteSensorDiscontinuityPoint(0.5);
    }
  }

  public static final class Extender {
    public static final TalonFXConfiguration extenderConfig = new TalonFXConfiguration();

    static {
      double extenderVelocityFeedForward =
          1.0 / Units.radiansToRotations(ExtenderConstants.kExtenderMotor.KvRadPerSecPerVolt);

      extenderConfig
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(ExtenderConstants.kExtenderMotorInverted)
                  .withNeutralMode(ExtenderConstants.kExtenderMotorNeutralMode))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(ExtenderConstants.kExtenderMotorStatorCurrentLimit)
                  .withSupplyCurrentLimit(ExtenderConstants.kExtenderMotorSupplyCurrentLimit))
          .withFeedback(
              new FeedbackConfigs()
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                  .withRotorToSensorRatio(1.0)
                  .withSensorToMechanismRatio(ExtenderConstants.kExtenderMotorReduction))
          .withSlot0(
              new Slot0Configs()
                  .withKP(0.1)
                  .withKI(0.0)
                  .withKD(0.0)
                  .withKS(0.0)
                  .withKV(extenderVelocityFeedForward)
                  .withKA(0.0))
          .withAudio(new AudioConfigs().withAllowMusicDurDisable(true));
    }
  }

  public static final class Intake {
    public static final TalonFXConfiguration intakeConfig = new TalonFXConfiguration();

    static {
      double intakeVelocityFeedForward =
          1.0 / Units.radiansToRotations(IntakeConstants.kIntakeMotor.KvRadPerSecPerVolt);

      intakeConfig
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(IntakeConstants.kIntakeMotorInverted)
                  .withNeutralMode(IntakeConstants.kIntakeMotorNeutralMode))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(IntakeConstants.kIntakeMotorStatorCurrentLimit)
                  .withSupplyCurrentLimit(IntakeConstants.kIntakeMotorSupplyCurrentLimit))
          .withFeedback(
              new FeedbackConfigs()
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                  .withRotorToSensorRatio(1.0)
                  .withSensorToMechanismRatio(IntakeConstants.kIntakeMotorReduction))
          .withSlot0(
              new Slot0Configs()
                  .withKP(1.2)
                  .withKI(0.0)
                  .withKD(0.0)
                  .withKS(0.0)
                  .withKV(intakeVelocityFeedForward)
                  .withKA(0.0))
          .withAudio(new AudioConfigs().withAllowMusicDurDisable(true));
    }
  }

  public static final class Feeder {
    public static final TalonFXConfiguration feederConfig = new TalonFXConfiguration();

    static {
      double feederVelocityFeedForward =
          1.0 / Units.radiansToRotations(FeederConstants.kFeederMotor.KvRadPerSecPerVolt);

      feederConfig
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(FeederConstants.kFeederInverted)
                  .withNeutralMode(FeederConstants.kFeederIntakeNeutralMode))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(FeederConstants.kFeederMotorStatorCurrentLimit)
                  .withSupplyCurrentLimit(FeederConstants.kFeederMotorSupplyCurrentLimit))
          .withFeedback(
              new FeedbackConfigs()
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                  .withRotorToSensorRatio(0)
                  .withSensorToMechanismRatio(FeederConstants.kFeederMotorReduction))
          .withSlot0(
              new Slot0Configs()
                  .withKP(0.1)
                  .withKI(0.0)
                  .withKD(0.0)
                  .withKS(0.0)
                  .withKV(feederVelocityFeedForward)
                  .withKA(0.0))
          .withAudio(new AudioConfigs().withAllowMusicDurDisable(true));
    }
  }

  public static final class Indexer {
    public static final TalonFXConfiguration indexConfig = new TalonFXConfiguration();

    static {
      double indexerVelocityFeedForward =
          1.0 / Units.radiansToRotations(IndexerConstants.kIndexerMotor.KvRadPerSecPerVolt);

      indexConfig
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(IndexerConstants.kMotorInvertedValue)
                  .withNeutralMode(IndexerConstants.kMotorNeutralMode))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(IndexerConstants.kIndexerMotorStatorCurrentLimit)
                  .withSupplyCurrentLimit(IndexerConstants.kIndexerMotorSupplyCurrentLimit))
          .withFeedback(
              new FeedbackConfigs()
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                  .withRotorToSensorRatio(1.0)
                  .withSensorToMechanismRatio(IndexerConstants.kIndexerMotorReduction))
          .withSlot0(
              new Slot0Configs()
                  .withKP(0.1)
                  .withKI(0.0)
                  .withKD(0.0)
                  .withKS(0.0)
                  .withKV(indexerVelocityFeedForward)
                  .withKA(0.0))
          .withAudio(new AudioConfigs().withAllowMusicDurDisable(true));
    }
  }

  public static final class Shooter {
    public static final TalonFXConfiguration shooterConfig = new TalonFXConfiguration();

    static {
      shooterConfig
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(ShooterConstants.kShooterLeaderInverted)
                  .withNeutralMode(ShooterConstants.kShooterMotorNeutralMode))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(ShooterConstants.kShooterMotorStatorCurrentLimit)
                  .withSupplyCurrentLimit(ShooterConstants.kShooterMotorSupplyCurrentLimit))
          .withFeedback(
              new FeedbackConfigs()
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                  .withRotorToSensorRatio(1.0)
                  .withSensorToMechanismRatio(ShooterConstants.kShooterMotorReduction))
          .withSlot0(
              new Slot0Configs()
                  .withKP(0.1)
                  .withKI(0.0)
                  .withKD(0.0)
                  .withKS(0.095995)
                  .withKV(0.11533)
                  .withKA(0.0067228))
          .withAudio(new AudioConfigs().withAllowMusicDurDisable(true));
    }
  }

  public static final class Hood {
    public static final SparkMaxConfig hoodLeaderConfig = new SparkMaxConfig();
    public static final SparkMaxConfig hoodFollowerConfig = new SparkMaxConfig();

    static {
      double hoodFactor = 1.0 / HoodConstants.kHoodEncoderReduction;

      hoodLeaderConfig
          .inverted(HoodConstants.kHoodLeaderInverted)
          .idleMode(HoodConstants.kHoodMotorIdleMode)
          .smartCurrentLimit(HoodConstants.kHoodMotorCurrentLimit)
          .voltageCompensation(Constants.kNominalVoltage);

      hoodLeaderConfig
          .absoluteEncoder
          .positionConversionFactor(hoodFactor) // rotations
          .velocityConversionFactor(hoodFactor / 60.0) // rotations per second
          .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoder);

      hoodFollowerConfig
          .apply(hoodLeaderConfig)
          .follow(HoodConstants.kHoodLeaderCanId)
          .inverted(HoodConstants.kHoodFollowerInverted);
    }
  }
}
