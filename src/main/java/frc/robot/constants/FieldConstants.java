package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.EnumMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

public class FieldConstants {
  public static final AprilTagFieldLayout FIELD_LAYOUT;

  static {
    FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    FIELD_LAYOUT.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
  }

  public static final double FIELD_HEIGHT = 8.0518;
  public static final double FIELD_LENGTH = 17.548249;

  public static Pose3d getTagPose(int id) {
    if (id < 1 || id > 22) {
      throw new IllegalArgumentException("id must be between 1 and 22");
    }

    return FIELD_LAYOUT
        .getTagPose(id)
        .orElseThrow(
            () -> {
              final String message = String.format("getTagPose called for unexpected tag %d", id);
              return new RuntimeException(message);
            });
  }

  public static final class Reef {
    // Injected alliance supplier so replay can control alliance without DriverStation.
    // Default uses DriverStation; override via setAllianceSupplier(...) in robotInit or replay
    // init.
    private static Supplier<Alliance> allianceSupplier =
        () -> DriverStation.getAlliance().orElse(Alliance.Blue);

    public static void setAllianceSupplier(Supplier<Alliance> supplier) {
      if (supplier != null) allianceSupplier = supplier;
    }

    public static final Translation2d REEF_CENTER_BLUE =
        new Translation2d(4.48936, FIELD_HEIGHT / 2);

    public static final Supplier<Translation2d> REEF_CENTER =
        () -> {
          var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
          boolean isRed =
              alliance.isPresent()
                  && alliance.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Red;
          return isRed
              ? new Translation2d(FIELD_LENGTH - REEF_CENTER_BLUE.getX(), REEF_CENTER_BLUE.getY())
              : REEF_CENTER_BLUE;
        };

    public static Translation2d getReefCenter() {
      return REEF_CENTER.get();
    }

    // reef face sides (organized by which tags are on each face)
    public enum ReefFaces {
      AB,
      CD,
      EF,
      GH,
      IJ,
      KL
    }

    public enum PipeSide {
      LEFT,
      RIGHT,
      CENTER
    }

    // coral scoring levels and standoffs
    public enum CoralScoreLevel {
      L1,
      L2,
      L3,
      L4
    }

    private static final Map<CoralScoreLevel, Double> CORALSTANDOFF =
        new EnumMap<>(CoralScoreLevel.class);

    static {
      CORALSTANDOFF.put(CoralScoreLevel.L1, 0.40);
      CORALSTANDOFF.put(CoralScoreLevel.L2, 0.45);
      CORALSTANDOFF.put(CoralScoreLevel.L3, 0.45);
      CORALSTANDOFF.put(CoralScoreLevel.L4, 0.40);
    }

    // reef algae levels and standoffs
    public enum ReefAlgaeLevel {
      L2GRABALGAE,
      L2SCALGAE,
      L3GRABALGAE,
      L3SCALGAE
    }

    private static final Map<ReefAlgaeLevel, Double> ALGAESTANDOFF =
        new EnumMap<>(ReefAlgaeLevel.class);

    static {
      ALGAESTANDOFF.put(ReefAlgaeLevel.L2GRABALGAE, 0.40);
      ALGAESTANDOFF.put(ReefAlgaeLevel.L3GRABALGAE, 0.40);
      ALGAESTANDOFF.put(ReefAlgaeLevel.L2SCALGAE, 0.40); // SC = supercycling
      ALGAESTANDOFF.put(ReefAlgaeLevel.L3SCALGAE, 0.40);
    }

    // left/right offset for reef pipes
    public static final double PIPE_HALF_SPACING = 0.3286 / 2;

    public static final EnumMap<ReefFaces, Integer> TAG_IDS_BLUE = new EnumMap<>(ReefFaces.class);

    static {
      // clockwise around the reef face
      TAG_IDS_BLUE.put(ReefFaces.AB, 18);
      TAG_IDS_BLUE.put(ReefFaces.CD, 19);
      TAG_IDS_BLUE.put(ReefFaces.EF, 20);
      TAG_IDS_BLUE.put(ReefFaces.GH, 21);
      TAG_IDS_BLUE.put(ReefFaces.IJ, 22);
      TAG_IDS_BLUE.put(ReefFaces.KL, 17);
    }

    private static final Pose2d[] FACE_POSE_BLUE = new Pose2d[6];
    private static final Pose2d[] FACE_POSE_RED = new Pose2d[6];

    // Look up each reef face tag ID from TAG_IDS_BLUE ans store each as a Pose2d
    public static final int initFromTags() {
      AprilTagFieldLayout layout = AprilTagFields.k2025ReefscapeWelded.loadAprilTagLayoutField();
      int count = 0;
      for (Map.Entry<ReefFaces, Integer> e : TAG_IDS_BLUE.entrySet()) {
        ReefFaces face = e.getKey();
        int id = e.getValue();
        Optional<Pose3d> opt = layout.getTagPose(id);
        if (opt.isEmpty()) continue;
        Pose2d p = opt.get().toPose2d();
        FACE_POSE_BLUE[face.ordinal()] = p;
        FACE_POSE_RED[face.ordinal()] = allianceFlip(p);
        count++;
      }
      return count;
    }

    private static Pose2d allianceFlip(Pose2d bluePose) {
      // Mirror across the field X-length centerline (x -> L - x) and reflect heading.
      // For a reflection across the vertical centerline, the heading transforms as (pi - theta).
      double x = (FIELD_LENGTH) - bluePose.getX();
      double y = bluePose.getY();
      Rotation2d rot = Rotation2d.fromRadians(Math.PI).minus(bluePose.getRotation());
      return new Pose2d(x, y, rot);
    }

    // Face poses
    private static Pose2d facePoseBlue(ReefFaces face) {
      Pose2d p = FACE_POSE_BLUE[face.ordinal()];
      return p;
    }

    public static Pose2d facePose(ReefFaces face, Alliance alliance) {
      int i = face.ordinal();
      return alliance == Alliance.Red ? FACE_POSE_RED[i] : FACE_POSE_BLUE[i];
    }

    public static Pose2d inward(ReefFaces face) {
      return facePose(face, allianceSupplier.get());
    }

    // coral and algae poses
    private static Pose2d poseAt(
        ReefFaces face, double standOff, PipeSide side, Alliance alliance) {
      // Use the alliance-specific center pose, then apply offsets so LEFT/RIGHT remain consistent
      // across alliances.
      Pose2d base = facePose(face, alliance);

      // Robot scores from the back; match robot rotation to the face rotation (plus 180° if
      // required by mechanism convention).
      Rotation2d inward = base.getRotation().plus(Rotation2d.fromDegrees(180));

      // Translate the standoffs (forward along the face pose rotation)
      Translation2d forward = new Translation2d(standOff, 0).rotateBy(base.getRotation());

      double lateralOffset =
          switch (side) {
            case LEFT -> PIPE_HALF_SPACING;
            case CENTER -> 0;
            case RIGHT -> -PIPE_HALF_SPACING;
          };

      Rotation2d leftDir = inward.plus(Rotation2d.fromDegrees(90));

      Translation2d lateral =
          new Translation2d(Math.abs(lateralOffset), 0)
              .rotateBy(lateralOffset < 0 ? leftDir.plus(Rotation2d.fromDegrees(180)) : leftDir);

      return new Pose2d(base.getTranslation().plus(forward).plus(lateral), inward);
    }

    // Public CORAL / ALGAE APIs
    private static double coralStandoff(CoralScoreLevel level) {
      Double v = CORALSTANDOFF.get(level);
      if (v == null) throw new IllegalStateException("Coral standoff missing for " + level);
      return v;
    }

    public static Pose2d coralPose(
        ReefFaces face, CoralScoreLevel level, PipeSide side, Alliance alliance) {
      return poseAt(face, coralStandoff(level), side, alliance);
    }

    public static Pose2d coralPose(ReefFaces face, CoralScoreLevel level, PipeSide side) {
      return coralPose(face, level, side, allianceSupplier.get());
    }

    private static double algaeStandoff(ReefAlgaeLevel level) {
      Double v = ALGAESTANDOFF.get(level);
      if (v == null) throw new IllegalStateException("Algae standoff missing for " + level);
      return v;
    }

    public static Pose2d algaePose(
        ReefFaces face, ReefAlgaeLevel level, PipeSide side, Alliance alliance) {
      return poseAt(face, algaeStandoff(level), side, alliance);
    }

    public static Pose2d algaePose(ReefFaces face, ReefAlgaeLevel level, PipeSide side) {
      return algaePose(face, level, side, allianceSupplier.get());
    }

    // returns the nearest reef face to a given robot pose
    public static ReefFaces nearestFace(Translation2d point, Alliance alliance) {
      ReefFaces best = ReefFaces.AB;
      double bestD2 = Double.POSITIVE_INFINITY;
      for (ReefFaces f : ReefFaces.values()) {
        Translation2d c = facePose(f, alliance).getTranslation();
        double dx = c.getX() - point.getX();
        double dy = c.getY() - point.getY();
        double d2 = dx * dx + dy * dy;
        if (d2 < bestD2) {
          bestD2 = d2;
          best = f;
        }
      }
      return best;
    }
  }
}
