package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ExtenderConstants;

// import frc.robot.utils.DIOAbsoluteEncoder;

public class Extender extends SubsystemBase {
  private TalonFX m_motor = new TalonFX(ExtenderConstants.kExtenderCanId);
  // private DIOAbsoluteEncoder m_encoder =
  //     new DIOAbsoluteEncoder(ExtenderConstants.kExtenderEncoderChannel);

  PositionVoltage m_positonVoltageControl = new PositionVoltage(0.0);

  // private ProfiledPIDController m_pidController =
  //     new ProfiledPIDController(
  //         ExtenderConstants.kP,
  //         ExtenderConstants.kI,
  //         ExtenderConstants.kD,
  //         new Constraints(ExtenderConstants.kMaxSpeed, ExtenderConstants.kMaxAcceleration));

  // private ArmFeedforward m_feedforward =
  //     new ArmFeedforward(
  //         ExtenderConstants.kS,
  //         ExtenderConstants.kG,
  //         ExtenderConstants.kV,
  //         ExtenderConstants.kA,
  //         Constants.kPeriodSeconds);

  public Extender() {
    m_motor.getConfigurator().apply(Configs.Extender.extenderConfig);

    // RobotModeTriggers.disabled()
    //     .onFalse(
    //         runOnce(
    //             () -> {
    //               m_motor.setPosition(0.0);
    //               setPosition(ExtenderConstants.kExtendedPosition);
    //             }));
  }

  /**
   * Gets the current angle of the extender based on the encoder position.
   *
   * @return The current angle of the extender as an Angle object.
   */
  // public Angle getAngle() {
  //   return Rotations.of(m_encoder.getPosition());
  // }

  // // gets the velocity of the encoder.
  // public double getVelocity() {
  //   return m_encoder.getVelocity();
  // }

  /**
   * Calculates the length of the winch string based on the given extender angle using the law of
   * cosines and pythagorean theorem.
   *
   * @param extenderAngle The angle of the extender, which is used to calculate the winch string
   *     length.
   * @return The length of the winch string in meters.
   */
  // public double getWinchLength(Angle extenderAngle) {
  //   double a = ExtenderConstants.kDistanceToMotorMeters;
  //   double b = ExtenderConstants.kRotationRadiusMeters;
  //   // law of cosines to find the distance from the motor to the winch attachment point
  //   double cSquared =
  //       Math.sqrt(
  //           Math.pow(a, 2) + Math.pow(b, 2) - 2 * a * b * Math.cos(extenderAngle.in(Radians)));

  //   // pythagorean theorem to find the length of the winch string
  //   return Math.sqrt(cSquared - Math.pow(ExtenderConstants.kExtenderWinchRadiusMeters, 2));
  // }

  /**
   * Calculates the length of the winch string based on the current extender angle using the law of
   * cosines and pythagorean theorem.
   *
   * @param extenderAngle The angle of the extender, which is used to calculate the winch string
   *     length.
   * @return The length of the winch string in meters.
   */
  // public double getWinchLength() {
  //   return getWinchLength(getAngle());
  // }

  public void setPosition(double position) {
    m_motor.setControl(m_positonVoltageControl.withPosition(position));
  }

  public Command run() {
    return startEnd(() -> m_motor.setVoltage(-3.0), () -> m_motor.setVoltage(0.0));
  }

  public Command runDown() {
    return startEnd(() -> m_motor.setVoltage(3.0), () -> m_motor.setVoltage(0.0));
  }

  public void addOrchestra(Orchestra orchestra) {
    orchestra.addInstrument(m_motor);
  }
}
