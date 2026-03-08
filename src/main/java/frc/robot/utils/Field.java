package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import java.util.EnumSet;
import org.apache.commons.geometry.euclidean.twod.AffineTransformMatrix2D;
import org.apache.commons.geometry.euclidean.twod.ConvexArea;
import org.apache.commons.geometry.euclidean.twod.Vector2D;
import org.apache.commons.geometry.euclidean.twod.shape.Parallelogram;

/** Utility class for working with the field. */
public class Field {
  private Field() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  /**
   * Gets the current alliance from the driver station.
   *
   * @return The current alliance, or blue if the alliance cannot be determined.
   */
  public static Alliance getAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue);
  }

  /**
   * Gets the alliance corresponding to the given translation on the field.
   *
   * @param pose The translation to get the alliance for, in field coordinates.
   * @return The alliance for the given translation.
   */
  public static Alliance getAllianceFor(Translation2d pose) {
    if (pose.getX() > FieldConstants.kFieldLengthMeters / 2) {
      return Alliance.Red;
    } else {
      return Alliance.Blue;
    }
  }

  /**
   * Gets the alliance corresponding to the given translation on the field.
   *
   * @param pose The translation to get the alliance for, in field coordinates.
   * @return The alliance for the given translation.
   */
  public static Alliance getAllianceFor(Translation3d pose) {
    return getAllianceFor(pose.toTranslation2d());
  }

  /**
   * Gets the alliance corresponding to the given pose on the field.
   *
   * @param pose The pose to get the alliance for, in field coordinates.
   * @return The alliance for the given pose.
   */
  public static Alliance getAllianceFor(Pose2d pose) {
    return getAllianceFor(pose.getTranslation());
  }

  /**
   * Gets the alliance corresponding to the given pose on the field.
   *
   * @param pose The pose to get the alliance for, in field coordinates.
   * @return The alliance for the given pose.
   */
  public static Alliance getAllianceFor(Pose3d pose) {
    return getAllianceFor(pose.getTranslation());
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

  /**
   * Flips a {@link Pose2d} to the given alliance. This is useful for converting between the red and
   * blue sides of the field, as the field is symmetrical.
   *
   * @param pose The pose to flip, in field coordinates.
   * @param alliance The alliance to flip to.
   * @return The flipped pose, in field coordinates.
   */
  public static Pose2d flipTo(Pose2d pose, Alliance alliance) {
    return alliance == getAllianceFor(pose) ? pose : flip(pose);
  }

  /**
   * Flips a {@link Pose3d} to the given alliance. This is useful for converting between the red and
   * blue sides of the field, as the field is symmetrical.
   *
   * @param pose The pose to flip, in field coordinates.
   * @param alliance The alliance to flip to.
   * @return The flipped pose, in field coordinates.
   */
  public static Pose3d flipTo(Pose3d pose, Alliance alliance) {
    return alliance == getAllianceFor(pose) ? pose : flip(pose);
  }

  /**
   * Flips a {@link Translation2d} to the given alliance. This is useful for converting between the
   * red and blue sides of the field, as the field is symmetrical.
   *
   * @param translation The translation to flip, in field coordinates.
   * @param alliance The alliance to flip to.
   * @return The flipped translation, in field coordinates.
   */
  public static Translation2d flipTo(Translation2d translation, Alliance alliance) {
    return alliance == getAllianceFor(translation) ? translation : flip(translation);
  }

  /**
   * Flips a {@link Translation3d} to the given alliance. This is useful for converting between the
   * red and blue sides of the field, as the field is symmetrical.
   *
   * @param translation The translation to flip, in field coordinates.
   * @return The flipped translation, in field coordinates.
   */
  public static Translation3d flipTo(Translation3d translation, Alliance alliance) {
    return alliance == getAllianceFor(translation) ? translation : flip(translation);
  }

  /**
   * Gets the zones that the given translation is within for the alliance corresponding to the
   * translation's position on the field.
   *
   * @param translation The translation to check, in field coordinates.
   * @return The zones that the translation is within.
   */
  public EnumSet<Zone> getZones(Translation2d translation) {
    EnumSet<Zone> zones = EnumSet.noneOf(Zone.class);
    for (Zone zone : Zone.values()) {
      if (zone.contains(translation)) {
        zones.add(zone);
      }
    }
    return zones;
  }

  /**
   * Gets the zones that the given translation is within for the alliance corresponding to the
   * translation's position on the field.
   *
   * @param translation The translation to check, in field coordinates.
   * @return The zones that the translation is within.
   */
  public EnumSet<Zone> getZones(Translation3d translation) {
    return getZones(translation.toTranslation2d());
  }

  /**
   * Gets the zones that the given pose is within for the alliance corresponding to the pose's
   * position on the field.
   *
   * @param pose The pose to check, in field coordinates.
   * @return The zones that the pose is within.
   */
  public EnumSet<Zone> getZones(Pose2d pose) {
    return getZones(pose.getTranslation());
  }

  /**
   * Gets the zones that the given pose is within for the alliance corresponding to the pose's
   * position on the field.
   *
   * @param pose The pose to check, in field coordinates.
   * @return The zones that the pose is within.
   */
  public EnumSet<Zone> getZones(Pose3d pose) {
    return getZones(pose.getTranslation());
  }

  /** Represents a specific landmark or element of the field. */
  public static enum Landmark {
    Hub(
        new Translation3d(
            FieldConstants.kAllianceZoneLengthMeters + FieldConstants.kHubLengthMeters / 2,
            FieldConstants.kFieldWidthMeters / 2,
            FieldConstants.kHubHeightMeters));

    private final Translation3d m_translation;

    Landmark(Translation3d translation) {
      m_translation = translation;
    }

    /** Gets the translation of this landmark for the given alliance. */
    public Translation3d getTranslation(Alliance alliance) {
      return flipTo(m_translation, alliance);
    }
  }

  /** Represents a zone on the field. */
  public static enum Zone {
    Alliance(
        Parallelogram.axisAligned(
            Vector2D.ZERO,
            Vector2D.of(FieldConstants.kAllianceZoneLengthMeters, FieldConstants.kFieldWidthMeters),
            FieldConstants.kRegionPrecision)),
    Neutral(
        Parallelogram.axisAligned(
            Vector2D.of(FieldConstants.kAllianceZoneLengthMeters, 0.0),
            Vector2D.of(FieldConstants.kFieldLengthMeters / 2, FieldConstants.kFieldWidthMeters),
            FieldConstants.kRegionPrecision)),
    LeftBump(
        Parallelogram.axisAligned(
            Vector2D.of(
                FieldConstants.kAllianceZoneLengthMeters
                    + (FieldConstants.kHubLengthMeters - FieldConstants.kBumpLengthMeters) / 2,
                (FieldConstants.kFieldWidthMeters + FieldConstants.kHubWidthMeters) / 2
                    + FieldConstants.kBumpWidthMeters),
            Vector2D.of(
                FieldConstants.kAllianceZoneLengthMeters
                    + (FieldConstants.kHubLengthMeters + FieldConstants.kBumpLengthMeters) / 2,
                (FieldConstants.kFieldWidthMeters + FieldConstants.kHubWidthMeters) / 2),
            FieldConstants.kRegionPrecision)),
    RightBump(
        Parallelogram.axisAligned(
            Vector2D.of(
                FieldConstants.kAllianceZoneLengthMeters
                    + (FieldConstants.kHubLengthMeters - FieldConstants.kBumpLengthMeters) / 2,
                (FieldConstants.kFieldWidthMeters - FieldConstants.kHubWidthMeters) / 2
                    - FieldConstants.kBumpWidthMeters),
            Vector2D.of(
                FieldConstants.kAllianceZoneLengthMeters
                    + (FieldConstants.kHubLengthMeters + FieldConstants.kBumpLengthMeters) / 2,
                (FieldConstants.kFieldWidthMeters - FieldConstants.kHubWidthMeters) / 2),
            FieldConstants.kRegionPrecision));

    private final ConvexArea m_region;

    private Zone(ConvexArea region) {
      m_region = region;
    }

    /**
     * Gets the region of the field this zone covers, flipping to the given alliance.
     *
     * @param alliance The alliance to get the region for.
     * @return The region of the field this zone covers for the given alliance.
     */
    public ConvexArea getRegion(Alliance alliance) {
      if (alliance == DriverStation.Alliance.Red) {
        AffineTransformMatrix2D transform =
            AffineTransformMatrix2D.createScale(-1.0)
                .translate(FieldConstants.kFieldLengthMeters, FieldConstants.kFieldWidthMeters);

        return m_region.transform(transform);
      }

      return m_region;
    }

    /**
     * Checks if the given translation is within this zone for the given alliance.
     *
     * @param alliance The alliance to check for.
     * @param translation The translation to check.
     * @return True if the translation is within this zone for the given alliance, false otherwise.
     */
    public boolean contains(Alliance alliance, Translation2d translation) {
      return getRegion(alliance).contains(Vector2D.of(translation.getX(), translation.getY()));
    }

    /**
     * Checks if the given translation is within this zone for the given alliance.
     *
     * @param alliance The alliance to check for.
     * @param translation The translation to check.
     * @return True if the translation is within this zone for the given alliance, false otherwise.
     */
    public boolean contains(Alliance alliance, Translation3d translation) {
      return contains(alliance, translation.toTranslation2d());
    }

    /**
     * Checks if the given pose is within this zone for the given alliance.
     *
     * @param alliance The alliance to check for.
     * @param pose The pose to check.
     * @return True if the pose is within this zone for the given alliance, false otherwise.
     */
    public boolean contains(Alliance alliance, Pose2d pose) {
      return contains(alliance, pose.getTranslation());
    }

    /**
     * Checks if the given pose is within this zone for the given alliance.
     *
     * @param alliance The alliance to check for.
     * @param pose The pose to check.
     * @return True if the pose is within this zone for the given alliance, false otherwise.
     */
    public boolean contains(Alliance alliance, Pose3d pose) {
      return contains(alliance, pose.getTranslation());
    }

    /**
     * Checks if the given translation is within this zone for the alliance corresponding to the
     * translation's position on the field.
     *
     * @param translation The translation to check.
     * @return True if the translation is within this zone, false otherwise.
     */
    public boolean contains(Translation2d translation) {
      return contains(getAllianceFor(translation), translation);
    }

    /**
     * Checks if the given translation is within this zone for the alliance corresponding to the
     * translation's position on the field.
     *
     * @param translation The translation to check.
     * @return True if the translation is within this zone, false otherwise.
     */
    public boolean contains(Translation3d translation) {
      return contains(translation.toTranslation2d());
    }

    /**
     * Checks if the given pose is within this zone for the alliance corresponding to the pose's
     * position on the field.
     *
     * @param pose The pose to check.
     * @return True if the pose is within this zone, false otherwise.
     */
    public boolean contains(Pose2d pose) {
      return contains(getAllianceFor(pose), pose);
    }

    /**
     * Checks if the given pose is within this zone for the alliance corresponding to the pose's
     * position on the field.
     *
     * @param pose The pose to check.
     * @return True if the pose is within this zone, false otherwise.
     */
    public boolean contains(Pose3d pose) {
      return contains(pose.getTranslation());
    }
  }
}
