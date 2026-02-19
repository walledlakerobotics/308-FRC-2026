package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants.FieldConstants;

public class Field {
  private Field() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  /**
   * Flips a {@link Pose2d} across the field. This is useful for converting between the red and blue
   * sides of the field, as the field is symmetrical.
   *
   * @param pose The pose to flip, in field coordinates.
   * @return The flipped pose, in field coordinates.
   */
  public static Pose2d flip(Pose2d pose) {
    return new Pose2d(flip(pose.getTranslation()), flip(pose.getRotation()));
  }

  /**
   * Flips a {@link Pose3d} across the field. This is useful for converting between the red and blue
   * sides of the field, as the field is symmetrical.
   *
   * @param pose The pose to flip, in field coordinates.
   * @return The flipped pose, in field coordinates.
   */
  public static Pose3d flip(Pose3d pose) {
    return new Pose3d(flip(pose.getTranslation()), flip(pose.getRotation()));
  }

  /**
   * Flips a {@link Translation2d} across the field. This is useful for converting between the red
   * and blue sides of the field, as the field is symmetrical.
   *
   * @param translation The translation to flip, in field coordinates.
   * @return The flipped translation, in field coordinates.
   */
  public static Translation2d flip(Translation2d translation) {
    return new Translation2d(
        FieldConstants.kFieldLengthMeters - translation.getX(),
        FieldConstants.kFieldWidthMeters - translation.getY());
  }

  /**
   * Flips a {@link Translation3d} across the field. This is useful for converting between the red
   * and blue sides of the field, as the field is symmetrical.
   *
   * @param translation The translation to flip, in field coordinates.
   * @return The flipped translation, in field coordinates.
   */
  public static Translation3d flip(Translation3d translation) {
    return new Translation3d(
        FieldConstants.kFieldLengthMeters - translation.getX(),
        FieldConstants.kFieldWidthMeters - translation.getY(),
        translation.getZ());
  }

  /**
   * Flips a {@link Rotation2d} across the field. This is useful for converting between the red and
   * blue sides of the field, as the field is symmetrical.
   *
   * @param rotation The rotation to flip, in field coordinates.
   * @return The flipped rotation, in field coordinates.
   */
  public static Rotation2d flip(Rotation2d rotation) {
    return rotation.plus(Rotation2d.k180deg);
  }

  /**
   * Flips a {@link Rotation3d} across the field. This is useful for converting between the red and
   * blue sides of the field, as the field is symmetrical.
   *
   * @param rotation The rotation to flip, in field coordinates.
   * @return The flipped rotation, in field coordinates.
   */
  public static Rotation3d flip(Rotation3d rotation) {
    return rotation.plus(new Rotation3d(Rotation2d.k180deg));
  }
}
