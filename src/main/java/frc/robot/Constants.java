// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.utils.CANIDs;
import java.util.List;
import org.apache.commons.numbers.core.Precision;
import org.apache.commons.numbers.core.Precision.DoubleEquivalence;

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
    public static final double kMaxSpeedMetersPerSecond = 6.0;
    public static final double kMaxAngularSpeed = 4 * Math.PI; // radians per second

    // Odometry measurement standard deviations
    public static final Matrix<N4, N1> kOdometryStdDevs =
        VecBuilder.fill(0.001, 0.001, 0.001, 0.01);

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(21.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(21.5);
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

    public static final double kDrivingMotorReduction = 6.75;
    public static final double kTurningMotorReduction = 150.0 / 7;

    public static final DCMotor kDrivingMotor =
        DCMotor.getNEO(1).withReduction(kDrivingMotorReduction);

    public static final DCMotor kTurningMotor =
        DCMotor.getNEO(1).withReduction(kTurningMotorReduction);

    public static final double kMaxSteerSpeedRadPerSec = 0.9 * kDrivingMotor.freeSpeedRadPerSec;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 40; // amps
    public static final int kTurningMotorCurrentLimit = 35; // amps

    public static final boolean kDrivingMotorsInverted = false;
    public static final boolean kTurningMotorsInverted = true;
    public static final SensorDirectionValue kTurningEncoderDirection =
        SensorDirectionValue.CounterClockwise_Positive;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kCoDriverControllerPort = 1;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final PIDConstants kTranslationConstants = new PIDConstants(1.5, 0.0, 0.0);
    public static final PIDConstants kRotationConstants = new PIDConstants(2.5, 0.0, 0.0);

    public static final PathFollowingController kPathFollowingController =
        new PPHolonomicDriveController(kTranslationConstants, kRotationConstants);
  }

  public static final class FieldConstants {
    public static final AprilTagFieldLayout kAprilTagFieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    public static final double kFieldLengthMeters = kAprilTagFieldLayout.getFieldLength();
    public static final double kFieldWidthMeters = kAprilTagFieldLayout.getFieldWidth();

    public static final double kHubLengthMeters = Units.inchesToMeters(47.0);
    public static final double kHubWidthMeters = Units.inchesToMeters(47.0);
    public static final double kHubHeightMeters = Units.inchesToMeters(72.0);

    public static final double kBumpLengthMeters = Units.inchesToMeters(44.4);
    public static final double kBumpWidthMeters = Units.inchesToMeters(73.0);

    public static final double kAllianceZoneLengthMeters = Units.inchesToMeters(158.61);

    public static final DoubleEquivalence kRegionPrecision =
        Precision.doubleEquivalenceOfEpsilon(0.01);
  }

  public static final class VisionConstants {
    public static final String[] kCameraNames = {"Arducam OV9281 #1", "Arducam OV9281 #2"};

    public static final Transform3d[] kRobotToCameraTransforms = {
      new Transform3d(
          new Pose3d(
              Units.inchesToMeters(24.0),
              0.0,
              Units.inchesToMeters(-44.25),
              new Rotation3d(Rotation2d.k180deg)),
          new Pose3d(
              0.6128275955053124,
              0.09668390599339825,
              -0.5964829934833575,
              new Rotation3d(
                  new Quaternion(
                      -0.1299234066976117,
                      0.2125947453635515,
                      -0.011083582808246062,
                      0.9683981701943858)))),
      new Transform3d(
          new Pose3d(
              Units.inchesToMeters(24.0),
              0.0,
              Units.inchesToMeters(-44.25),
              new Rotation3d(Rotation2d.k180deg)),
          new Pose3d(
              0.5942146854401791,
              -0.23197565220133776,
              -0.5973754694329283,
              new Rotation3d(
                  new Quaternion(
                      0.10571767646617448,
                      0.1980934342547792,
                      0.03918154332501794,
                      0.9736769537300156)))),
      Transform3d.kZero,
      Transform3d.kZero
    };

    public static final List<Matrix<N4, N1>> kVisionMeasurementStdDevs =
        List.of(
            VecBuilder.fill(0.005, 0.005, 0.005, 3.0),
            VecBuilder.fill(0.005, 0.005, 0.005, 3.0),
            VecBuilder.fill(0.005, 0.005, 0.005, 3.0),
            VecBuilder.fill(0.005, 0.005, 0.005, 3.0));
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

    public static final double kExtenderWinchRadiusMeters = Units.inchesToMeters(0.5);
    public static final double kExtenderWinchDiamaterMeters = 2 * kExtenderWinchRadiusMeters;
    public static final double kExtenderWinchCircumferenceMeters =
        kExtenderWinchDiamaterMeters * Math.PI;

    public static final double kExtenderMotorReduction = 1.0;

    public static final DCMotor kExtenderMotor =
        DCMotor.getKrakenX60(1).withReduction(kExtenderMotorReduction);

    public static final NeutralModeValue kExtenderMotorNeutralMode = NeutralModeValue.Brake;
    public static final int kExtenderMotorStatorCurrentLimit = 40; // amps
    public static final int kExtenderMotorSupplyCurrentLimit = 20; // amps
    public static final InvertedValue kExtenderMotorInverted =
        InvertedValue.CounterClockwise_Positive;

    public static final double kExtendedPosition = 10.0;
  }

  public static final class IntakeConstants {
    public static final int kIntakeCanId = CANIDs.secondaryMotor(1);

    public static final double kIntakeMotorReduction = 1.0;
    public static final NeutralModeValue kIntakeMotorNeutralMode = NeutralModeValue.Brake;
    public static final int kIntakeMotorStatorCurrentLimit = 120; // amps
    public static final int kIntakeMotorSupplyCurrentLimit = 70; // amps
    public static final InvertedValue kIntakeMotorInverted = InvertedValue.Clockwise_Positive;

    public static final DCMotor kIntakeMotor =
        DCMotor.getKrakenX60(1).withReduction(kIntakeMotorReduction);

    public static final AngularVelocity kIntakeVelocity = RotationsPerSecond.of(100.0);
  }

  public static final class ShooterConstants {
    public static final int kShooterLeaderCanId = CANIDs.secondaryMotor(2);
    public static final int kShooterFollowerCanId = CANIDs.secondaryMotor(3);

    public static final double kShooterMotorReduction = 1.0;
    public static final NeutralModeValue kShooterMotorNeutralMode = NeutralModeValue.Coast;
    public static final int kShooterMotorStatorCurrentLimit = 120; // amps
    public static final int kShooterMotorSupplyCurrentLimit = 70; // amps
    public static final InvertedValue kShooterLeaderInverted =
        InvertedValue.CounterClockwise_Positive;
    public static final MotorAlignmentValue kShooterFollowerAlignment = MotorAlignmentValue.Opposed;

    public static final DCMotor kShooterMotor =
        DCMotor.getKrakenX60(1).withReduction(kShooterMotorReduction);

    public static final int kVirtualTargetIterations = 5;
  }

  public static final class HoodConstants {
    public static final int kHoodLeaderCanId = CANIDs.secondaryMotor(4);
    public static final int kHoodFollowerCanId = CANIDs.secondaryMotor(5);

    public static final IdleMode kHoodMotorIdleMode = IdleMode.kCoast;
    public static final int kHoodMotorCurrentLimit = 40; // amps

    public static final boolean kHoodLeaderInverted = true;
    public static final boolean kHoodFollowerInverted = true;

    // angle between horizontal line and the line between the axle of hood and mount point of the
    // actuator
    public static final Angle kActuatorAngleOffset = Degrees.of(39);

    public static final double kHoodEncoderReduction = 19.08 / 7;

    public static final double kDistanceToMountPointMeters = 0.2;
    public static final double kRotationRadiusMeters = 0.075;

    public static final PIDConstants kHoodPIDConstants = new PIDConstants(50.0, 100.0, 0.0);
  }

  public static final class IndexerConstants {

    public static final int kMotorCANId = CANIDs.secondaryMotor(6);
    public static final double kIndexerMotorReduction = 1.0;

    public static final NeutralModeValue kMotorNeutralMode = NeutralModeValue.Brake;
    public static final InvertedValue kMotorInvertedValue = InvertedValue.CounterClockwise_Positive;

    public static final DCMotor kIndexerMotor =
        DCMotor.getKrakenX44(1).withReduction(kIndexerMotorReduction);

    public static final int kIndexerMotorStatorCurrentLimit = 120; // amps
    public static final int kIndexerMotorSupplyCurrentLimit = 70; // amps

    public static final int kLeftLightSensorChannel = 0;
    public static final int kRightLightSensorChannel = 0;

    public static final AngularVelocity kIndexerVelocity = RotationsPerSecond.of(3000.0);
  }

  public static final class FeederConstants {
    public static final int kFeederCanId = CANIDs.secondaryMotor(7);
    public static final double kFeederMotorReduction = 30.0 / 18.0;
    public static final NeutralModeValue kFeederIntakeNeutralMode = NeutralModeValue.Brake;
    public static final int kFeederMotorStatorCurrentLimit = 120;
    public static final int kFeederMotorSupplyCurrentLimit = 70;
    public static final InvertedValue kFeederInverted = InvertedValue.CounterClockwise_Positive;

    public static final DCMotor kFeederMotor =
        DCMotor.getKrakenX44(1).withReduction(kFeederMotorReduction);

    public static final AngularVelocity kIdleVelocity = RotationsPerSecond.of(5.0);
    public static final AngularVelocity kFeedingVelocity = RotationsPerSecond.of(30.0);
  }
}
