// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;
  private final Supplier<Pose2d> robotPoseSupplier;
  // Latest accepted estimator pose per camera (field frame). Null if none accepted yet.
  private final Pose2d[] latestEstimatorPoses;
  private final double[] latestEstimatorTimestamps;

  public Vision(VisionConsumer consumer, Supplier<Pose2d> robotPoseSupplier, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;
    this.robotPoseSupplier = robotPoseSupplier;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }

    // Initialize latest estimator poses
    latestEstimatorPoses = new Pose2d[io.length];
    latestEstimatorTimestamps = new double[io.length];
    for (int i = 0; i < latestEstimatorTimestamps.length; i++) {
      latestEstimatorTimestamps[i] = Double.NaN;
    }
  }

  public Vision(VisionConsumer consumer, VisionIO... io) {
    this(consumer, null, io);
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  public Rotation2d getTartgetY(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.ty();
  }

  public boolean hasTarget(int cameraIndex) {
    return inputs[cameraIndex].tagIds.length > 0;
  }

  private volatile int servoingTagID = 18;

  public void setservoingTagID(int id) {
    servoingTagID = id;
  }

  public int getservoingTagID() {
    return servoingTagID;
  }

  public boolean hasTag(int cameraIndex, int tagId) {
    for (int id : inputs[cameraIndex].tagIds) {
      if (id == tagId) return true;
    }
    return false;
  }

  /**
   * Estimates horizontal distance from the camera to the specified AprilTag using vertical angle.
   * Uses camera pitch from `robotToCamera*` and the tag's Z from the field layout. Returns NaN if
   * the tag is not currently detected by that camera.
   */
  public double getDistanceToTagMeters(int cameraIndex, int tagId) {
    // Require that this camera currently sees the tag
    if (!hasTag(cameraIndex, tagId)) {
      return Double.NaN;
    }

    // Get target height (Z in meters) from AprilTag layout
    var tagPoseOpt = aprilTagLayout.getTagPose(tagId);
    if (tagPoseOpt.isEmpty()) {
      return Double.NaN;
    }
    double targetZ = tagPoseOpt.get().getZ();

    // Select camera transform based on index (supports 0 and 1)
    Transform3d robotToCamera = getRobotToCamera(cameraIndex);

    // Camera mounting parameters
    double cameraZ = robotToCamera.getZ(); // meters
    double cameraPitch = robotToCamera.getRotation().getY(); // radians (pitch)

    // Measured vertical angle to target from this camera (radians)
    double ty = inputs[cameraIndex].latestTargetObservation.ty().getRadians();

    // Total vertical angle above/below horizontal
    double totalAngle = cameraPitch + ty;

    // Prevent divide-by-zero; if too shallow, return infinity
    double tan = Math.tan(totalAngle);
    if (Math.abs(tan) < 1e-6) {
      return Double.POSITIVE_INFINITY;
    }

    // Horizontal distance on the floor plane to the tag center projection
    double dz = targetZ - cameraZ;
    double distance = dz / tan;
    return Math.abs(distance);
  }

  /** Convenience: distance to the current servoingTagID from the given camera. */
  public double getDistanceToServoingTagMeters(int cameraIndex) {
    return getDistanceToTagMeters(cameraIndex, servoingTagID);
  }

  /** Returns the latest accepted estimator pose for a camera, or null if none yet. */
  public Pose2d getLatestEstimatorPose2d(int cameraIndex) {
    if (cameraIndex < 0 || cameraIndex >= latestEstimatorPoses.length) return null;
    return latestEstimatorPoses[cameraIndex];
  }

  /**
   * Returns the latest estimator pose if the camera is connected and the pose is newer than
   * maxAgeSec. Otherwise returns null.
   */
  public Pose2d getLatestEstimatorPose2dFresh(int cameraIndex, double maxAgeSec) {
    if (cameraIndex < 0 || cameraIndex >= latestEstimatorPoses.length) return null;
    if (!isCameraConnected(cameraIndex)) return null;
    double ts = latestEstimatorTimestamps[cameraIndex];
    if (!Double.isFinite(ts)) return null;
    if ((Timer.getFPGATimestamp() - ts) > maxAgeSec) return null;
    return latestEstimatorPoses[cameraIndex];
  }

  /** Returns true if the specified camera is currently considered connected/fresh. */
  public boolean isCameraConnected(int cameraIndex) {
    if (cameraIndex < 0 || cameraIndex >= inputs.length) return false;
    return inputs[cameraIndex].connected;
  }

  /**
   * Distance from the specified camera to a field point using latest estimator pose. Returns NaN if
   * no pose has been accepted yet for that camera.
   */
  public double getDistanceToFieldPointMeters(
      int cameraIndex, edu.wpi.first.math.geometry.Translation2d fieldPoint) {
    Pose2d camFieldPose = getLatestEstimatorPose2d(cameraIndex);
    if (camFieldPose == null) return Double.NaN;
    return camFieldPose.getTranslation().getDistance(fieldPoint);
  }

  @Override
  public void periodic() {
    Pose2d robotFieldPose = robotPoseSupplier != null ? robotPoseSupplier.get() : null;

    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
    }

    // Publish camera transform(s) for AdvantageScope visualization
    for (int cameraIndex = 0; cameraIndex < Math.min(io.length, 2); cameraIndex++) {
      Transform3d robotToCamera = getRobotToCamera(cameraIndex);
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotToCamera", robotToCamera);
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotToCameraPose",
          getRobotToCameraPose(cameraIndex));
      if (robotFieldPose != null) {
        Logger.recordOutput(
            "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotToCameraFieldPose",
            new Pose3d(robotFieldPose).transformBy(robotToCamera));
      }
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                || Math.abs(observation.pose().getZ())
                    > maxZError // Must have realistic Z coordinate

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > aprilTagLayout.getFieldWidth();

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= angularStdDevMegatag2Factor;
        }
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        // Update latest estimator pose and timestamp for this camera
        latestEstimatorPoses[cameraIndex] = observation.pose().toPose2d();
        latestEstimatorTimestamps[cameraIndex] = observation.timestamp();

        // Build a ray from the camera pose to this observation pose (for visualization/debug)
        Transform3d robotToCamera = getRobotToCamera(cameraIndex);
        // Send vision observation
        if (consumer != null) {
          consumer.accept(
              observation.pose().toPose2d(),
              observation.timestamp(),
              VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
        }
      }

      // Log camera datadata
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
          tagPoses.toArray(new Pose3d[tagPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
          robotPoses.toArray(new Pose3d[robotPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    Logger.recordOutput(
        "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted",
        allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected",
        allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
