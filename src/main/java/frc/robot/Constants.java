// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.utils.CANIDs;
import java.util.List;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double kPeriodSeconds = TimedRobot.kDefaultPeriod;
  public static final double kNominalVoltage = 12.0;

  public static final RobotConfig kRobotConfig;

  static {
    try {
      kRobotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      throw new RuntimeException("Failed to load robot configuration", e);
    }
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds
    // of the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.47;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Odometry measurement standard deviations
    public static final Matrix<N4, N1> kOdometryStdDevs =
        VecBuilder.fill(0.001, 0.001, 0.001, 0.01);

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(23);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(23.125);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = CANIDs.frontLeftDrive();
    public static final int kRearLeftDrivingCanId = CANIDs.rearLeftDrive();
    public static final int kFrontRightDrivingCanId = CANIDs.frontRightDrive();
    public static final int kRearRightDrivingCanId = CANIDs.rearRightDrive();

    public static final int kFrontLeftTurningCanId = CANIDs.frontLeftTurning();
    public static final int kRearLeftTurningCanId = CANIDs.rearLeftTurning();
    public static final int kFrontRightTurningCanId = CANIDs.frontRightTurning();
    public static final int kRearRightTurningCanId = CANIDs.rearRightTurning();

    // Encoder CAN IDs
    public static final int kFrontLeftTurningEncoderId = CANIDs.frontLeftEncoder();
    public static final int kRearLeftTurningEncoderId = CANIDs.rearLeftEncoder();
    public static final int kFrontRightTurningEncoderId = CANIDs.frontRightEncoder();
    public static final int kRearRightTurningEncoderId = CANIDs.rearRightEncoder();

    public static final SwerveSetpointGenerator kSetpointGenerator =
        new SwerveSetpointGenerator(kRobotConfig, ModuleConstants.kMaxSteerSpeedRadPerSec);
  }

  public static final class ModuleConstants {
    // Calculations required for driving motor conversion factors and feed forward
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kWheelRadiusMeters = kWheelDiameterMeters / 2;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

    public static final DCMotor kDrivingMotor = DCMotor.getNEO(1);

    public static final double kDrivingMotorReduction = 6.75;
    public static final double kDriveFreeSpeedMetersPerSecond =
        (kDrivingMotor.freeSpeedRadPerSec * kWheelRadiusMeters) / kDrivingMotorReduction;

    public static final double kTurningMotorReduction = 150.0 / 7;

    public static final double kMaxSteerSpeedRadPerSec =
        0.9 * kDrivingMotor.freeSpeedRadPerSec / kTurningMotorReduction;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 40; // amps
    public static final int kTurningMotorCurrentLimit = 35; // amps

    public static final boolean kDrivingMotorsInverted = true;
    public static final boolean kTurningMotorsInverted = true;
    public static final boolean kTurningEncoderInverted = false;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final PIDConstants kTranslationConstants = new PIDConstants(1, 0, 0);
    public static final PIDConstants kRotationConstants = new PIDConstants(1, 0, 0);

    public static final PathFollowingController kPathFollowingController =
        new PPHolonomicDriveController(kTranslationConstants, kRotationConstants);
  }

  public static final class FieldConstants {
    public static final AprilTagFieldLayout kAprilTagFieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    public static final double kFieldLengthMeters = kAprilTagFieldLayout.getFieldLength();
    public static final double kFieldWidthMeters = kAprilTagFieldLayout.getFieldWidth();

    public static final double kHubLengthMeters = Units.inchesToMeters(47.0);
    public static final double kBumpLengthMeters = Units.inchesToMeters(47.0);

    public static final double kAllianceZoneLengthMeters = Units.inchesToMeters(158.61);

    public static enum ScoringTarget {
      Hub(
          new Translation2d(
              kAllianceZoneLengthMeters + kHubLengthMeters / 2, kFieldWidthMeters / 2));

      private final Translation2d pose;

      ScoringTarget(Translation2d pose) {
        this.pose = pose;
      }

      public Translation2d getTranslation() {
        return pose;
      }
    }
  }

  public static final class VisionConstants {
    public static final String[] kCameraNames = {
      "Arducam OV9281 #1", "Arducam OV9281 #2", "Arducam OV9281 #3", "Arducam OV9281 #4"
    };

    public static final Transform3d[] kRobotToCameraTransforms = {
      Transform3d.kZero, Transform3d.kZero, Transform3d.kZero, Transform3d.kZero
    };

    public static final List<Matrix<N4, N1>> kVisionMeasurementStdDevs =
        List.of(
            VecBuilder.fill(0.02, 0.02, 0.02, 3.0),
            VecBuilder.fill(0.02, 0.02, 0.02, 3.0),
            VecBuilder.fill(0.02, 0.02, 0.02, 3.0),
            VecBuilder.fill(0.02, 0.02, 0.02, 3.0));
  }

  public static final class MatchConstants {
    public static final double kAutoPeriodSeconds = 20.0;
    public static final double kTeleopPeriodSeconds = 140.0;

    public static final double kTransitionPeriodSeconds = 10.0;

    public static final int kNumShiftPeriods = 4;
    public static final double kShiftPeriodSeconds = 25.0;

    public static final double kEndgamePeriodSeconds = 30.0;
  }

  public static final class ExtenderConstants {
    public static final int kExtenderCanId = CANIDs.secondaryMotor(0);

    public static final double kExtenderMotorReduction = 1.0;
    public static final IdleMode kExtenderMotorIdleMode = IdleMode.kBrake;
    public static final int kExtenderMotorCurrentLimit = 30; // amps
    public static final boolean kExtenderMotorInverted = false;

    public static final double kExdenterMotorSpeed = 0.5;
  }

  public static final class IntakeConstants {
    public static final int kIntakeCanId = CANIDs.secondaryMotor(1);

    public static final double kIntakeMotorReduction = 1.0;
    public static final IdleMode kIntakeMotorIdleMode = IdleMode.kBrake;
    public static final int kIntakeMotorCurrentLimit = 30; // amps
    public static final boolean kIntakeMotorInverted = false;

    public static final double kIntakeSpeed = 0.5;
  }

  public static final class ShooterConstants {
    public static final int kShooterLeaderCanId = CANIDs.secondaryMotor(2);
    public static final int kShooterFollowerCanId = CANIDs.secondaryMotor(3);

    public static final DCMotor kShooterMotor = DCMotor.getKrakenX60(2);

    public static final double kShooterMotorReduction = 1.0;
    public static final NeutralModeValue kShooterMotorNeutralMode = NeutralModeValue.Coast;
    public static final int kShooterMotorStatorCurrentLimit = 120; // amps
    public static final int kShooterMotorSupplyCurrentLimit = 70; // amps
    public static final InvertedValue kShooterLeaderInverted =
        InvertedValue.CounterClockwise_Positive;
    public static final MotorAlignmentValue kShooterFollowerAlignment = MotorAlignmentValue.Opposed;

    public static final int kVirtualTargetIterations = 5;
  }

  public static final class AimerConstants {
    public static final int kAimerPWMChannel = 0;
    public static final int kEncoderDIOChannel = 0;

    public static final boolean kAimerMotorInverted = false;
    public static final boolean kAimerEncoderInverted = false;

    public static final double kAimerEncoderDutyCycleMin = 1.0 / 1025.0;
    public static final double kAimerEncoderDutyCycleMax = 1024.0 / 1025.0;

    public static final double kAimerEncoderFrequencyHz = 975.6;

    public static final Angle kAimerEncoderOffset = Rotations.of(0.0);

    public static final double kAimerP = 1.0;
    public static final double kAimerI = 0.0;
    public static final double kAimerD = 0.0;
  }
}
