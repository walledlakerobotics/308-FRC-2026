package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.FieldConstants;

public class Field {
  private Field() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  /**
   * Flips a pose across the field. This is useful for converting between the red and blue sides of
   * the field, as the field is symmetrical.
   *
   * @param pose The pose to flip, in field coordinates.
   * @return The flipped pose, in field coordinates.
   */
  public static Pose2d flip(Pose2d pose) {
    return new Pose2d(
        FieldConstants.kFieldLengthMeters - pose.getX(),
        FieldConstants.kFieldWidthMeters - pose.getY(),
        pose.getRotation().plus(Rotation2d.k180deg));
  }
}
