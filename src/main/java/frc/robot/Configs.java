package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.ExtenderConstants;
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
      double nominalVoltage = 12.0;
      double drivingVelocityFeedForward =
          nominalVoltage / ModuleConstants.kDriveFreeSpeedMetersPerSecond;

      drivingConfig
          .inverted(ModuleConstants.kDrivingMotorsInverted)
          .idleMode(ModuleConstants.kDrivingMotorIdleMode)
          .smartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
      drivingConfig
          .encoder
          .positionConversionFactor(drivingFactor) // meters
          .velocityConversionFactor(drivingFactor / 60.0); // meters per second
      drivingConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(0.04, 0, 0)
          .outputRange(-1.0, 1.0)
          .feedForward
          .sva(0.0, drivingVelocityFeedForward, 0.0);

      turningConfig
          .inverted(ModuleConstants.kTurningMotorsInverted)
          .idleMode(ModuleConstants.kTurningMotorIdleMode)
          .smartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);
      turningConfig
          .encoder
          .positionConversionFactor(turningFactor) // rotations
          .velocityConversionFactor(turningFactor / 60.0); // rotations per second
      turningConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(1.0, 0, 0)
          .outputRange(-1.0, 1.0)
          .positionWrappingEnabled(true)
          .positionWrappingInputRange(-0.5, 0.5);

      turningEncoderConfig
          .withSensorDirection(
              ModuleConstants.kTurningEncoderInverted
                  ? SensorDirectionValue.Clockwise_Positive
                  : SensorDirectionValue.CounterClockwise_Positive)
          .withAbsoluteSensorDiscontinuityPoint(0.5);
    }
  }

  public static final class Extender {
    public static final SparkMaxConfig extenderConfig = new SparkMaxConfig();

    static {
      double extenderFactor = 1.0 / ExtenderConstants.kExtenderMotorReduction;

      extenderConfig
          .inverted(ExtenderConstants.kExtenderMotorInverted)
          .idleMode(ExtenderConstants.kExtenderMotorIdleMode)
          .smartCurrentLimit(ExtenderConstants.kExtenderMotorCurrentLimit);
      extenderConfig
          .absoluteEncoder
          .positionConversionFactor(extenderFactor) // rotations
          .velocityConversionFactor(extenderFactor / 60.0); // rotations per second
      extenderConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          .pid(0.1, 0, 0)
          .outputRange(-1.0, 1.0);
    }
  }

  public static final class Intake {
    public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();

    static {
      double intakeFactor = 1.0 / IntakeConstants.kIntakeMotorReduction;

      intakeConfig
          .inverted(IntakeConstants.kIntakeMotorInverted)
          .idleMode(IntakeConstants.kIntakeMotorIdleMode)
          .smartCurrentLimit(IntakeConstants.kIntakeMotorCurrentLimit)
          .voltageCompensation(12.0);
      intakeConfig
          .encoder
          .positionConversionFactor(intakeFactor) // rotations
          .velocityConversionFactor(intakeFactor / 60.0); // rotations per second
      intakeConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(0.1, 0, 0)
          .outputRange(-1.0, 1.0);
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
                  .withKS(0.0)
                  .withKV(0.0)
                  .withKA(0.0));
    }
  }
}
