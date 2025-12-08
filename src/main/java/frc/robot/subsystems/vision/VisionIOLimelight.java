// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.BiConsumer;
import java.util.function.Supplier;
import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;
import limelight.results.RawFiducial;

/** IO implementation for real Limelight hardware. */
public class VisionIOLimelight implements VisionIO {
  private final Supplier<Rotation2d> rotationSupplier;
  private final Limelight camera;
  private final LimelightPoseEstimator estimatorMegatag1;
  private final LimelightPoseEstimator estimatorMegatag2;
  private final boolean enablePoseEstimation;
  private final Supplier<Transform3d> robotToCameraSupplier;
  // Freshness tracking (FPGA timeline) and last NT pose timestamp seen
  private double lastDataFpgaTime = Double.NEGATIVE_INFINITY;
  private double lastSeenNtPoseTimestamp = Double.NEGATIVE_INFINITY;
  // Track last-used estimator timestamps to avoid duplicates per type
  // Avoid resending identical pose samples per estimator type
  private double lastUsedMegatag1Ts = Double.NEGATIVE_INFINITY;
  private double lastUsedMegatag2Ts = Double.NEGATIVE_INFINITY;
  // Heartbeat subscriber to mimic template resiliency (uses NT 'tl' latency as a frequently-updated
  // entry)
  private final DoubleSubscriber heartbeatLatencySub;

  /**
   * Creates a new VisionIOLimelight with pose estimation enabled.
   *
   * @param name The configured name of the Limelight.
   * @param rotationSupplier Supplier for the current estimated rotation, used for MegaTag 2.
   */
  public VisionIOLimelight(String name, Supplier<Rotation2d> rotationSupplier) {
    this(name, rotationSupplier, true, null);
  }

  /**
   * Creates a new VisionIOLimelight with pose estimation enabled and a dynamic camera transform.
   *
   * @param name The configured name of the Limelight.
   * @param rotationSupplier Supplier for the current estimated rotation, used for MegaTag 2.
   * @param robotToCameraSupplier Supplier for the robot-to-camera transform (for moving mounts).
   */
  public VisionIOLimelight(
      String name,
      Supplier<Rotation2d> rotationSupplier,
      Supplier<Transform3d> robotToCameraSupplier) {
    this(name, rotationSupplier, true, robotToCameraSupplier);
  }

  /**
   * Creates a new VisionIOLimelight.
   *
   * @param name The configured name of the Limelight.
   * @param rotationSupplier Supplier for the current estimated rotation, used for MegaTag 2.
   * @param enablePoseEstimation If false, only tx/ty and tag IDs are populated (no pose samples).
   */
  public VisionIOLimelight(
      String name, Supplier<Rotation2d> rotationSupplier, boolean enablePoseEstimation) {
    this(name, rotationSupplier, enablePoseEstimation, null);
  }

  /**
   * Creates a new VisionIOLimelight.
   *
   * @param name The configured name of the Limelight.
   * @param rotationSupplier Supplier for the current estimated rotation, used for MegaTag 2.
   * @param enablePoseEstimation If false, only tx/ty and tag IDs are populated (no pose samples).
   * @param robotToCameraSupplier Supplier for the robot-to-camera transform (for moving mounts).
   */
  public VisionIOLimelight(
      String name,
      Supplier<Rotation2d> rotationSupplier,
      boolean enablePoseEstimation,
      Supplier<Transform3d> robotToCameraSupplier) {
    this.rotationSupplier = rotationSupplier;
    this.enablePoseEstimation = enablePoseEstimation;
    this.robotToCameraSupplier = robotToCameraSupplier;
    camera = new Limelight(name);
    estimatorMegatag1 = camera.getPoseEstimator(false);
    estimatorMegatag2 = camera.getPoseEstimator(true);
    heartbeatLatencySub = camera.getNTTable().getDoubleTopic("tl").subscribe(0.0);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Current FPGA time in seconds
    double nowFpga = Timer.getFPGATimestamp();
    // Update latest target observation
    var data = camera.getData();
    var targetData = data.targetData;
    boolean hasTarget = targetData.getTargetStatus();
    double txDeg = hasTarget ? targetData.getHorizontalOffset() : 0.0;
    double tyDeg = hasTarget ? targetData.getVerticalOffset() : 0.0;
    inputs.latestTargetObservation =
        new TargetObservation(Rotation2d.fromDegrees(txDeg), Rotation2d.fromDegrees(tyDeg));
    // Treat valid 2D target info as a fresh update for connectivity purposes
    if (hasTarget) {
      lastDataFpgaTime = Math.max(lastDataFpgaTime, nowFpga);
    }

    // Update orientation for MegaTag 2 using YALL settings API (only if estimating poses)
    if (enablePoseEstimation) {
      var yaw = rotationSupplier.get().getRadians();
      var orientation =
          new Orientation3d(
              new Rotation3d(0.0, 0.0, yaw),
              new AngularVelocity3d(
                  edu.wpi.first.units.Units.RadiansPerSecond.of(0.0),
                  edu.wpi.first.units.Units.RadiansPerSecond.of(0.0),
                  edu.wpi.first.units.Units.RadiansPerSecond.of(0.0)));
      var settings = camera.getSettings().withRobotOrientation(orientation);
      if (robotToCameraSupplier != null) {
        settings.withCameraOffset(new Pose3d().transformBy(robotToCameraSupplier.get()));
      }
      camera.flush();
    }

    // Read new observations
    Set<Integer> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();

    // Add tag IDs from raw fiducials (available even when not estimating)
    RawFiducial[] fiducials = data.getRawFiducials();
    if (fiducials != null) {
      for (var f : fiducials) {
        tagIds.add(f.id);
      }
    }

    // Helper to convert a PoseEstimate into our PoseObservation and collect tag IDs
    BiConsumer<PoseEstimate, PoseObservationType> addObservation =
        (estimate, type) -> {
          if (!estimate.hasData) return;

          // Skip stale/duplicate samples from the same estimator type
          if (type == PoseObservationType.MEGATAG_1) {
            if (estimate.timestampSeconds <= lastUsedMegatag1Ts) return;
            lastUsedMegatag1Ts = estimate.timestampSeconds;
          } else if (type == PoseObservationType.MEGATAG_2) {
            if (estimate.timestampSeconds <= lastUsedMegatag2Ts) return;
            lastUsedMegatag2Ts = estimate.timestampSeconds;
          }

          // Collect tag IDs
          if (estimate.rawFiducials != null) {
            for (var fid : estimate.rawFiducials) {
              tagIds.add(fid.id);
            }
          }

          // Build and add observation: template behavior (NT timestamp - latency)
          double timestamp = estimate.timestampSeconds - estimate.latency;
          Pose3d pose = estimate.pose;
          int count = estimate.tagCount;
          double avgDist = estimate.avgTagDist;
          double ambiguity = 0.0;
          if (type == PoseObservationType.MEGATAG_1) {
            // Use average ambiguity for single-tag observations; 0 if multitag
            ambiguity = (count == 1) ? estimate.getAvgTagAmbiguity() : 0.0;
          } else {
            // MEGATAG_2 is already disambiguated
            ambiguity = 0.0;
          }
          poseObservations.add(
              new PoseObservation(timestamp, pose, ambiguity, count, avgDist, type));

          // Update freshness only when NT pose timestamp advances
          if (estimate.timestampSeconds > lastSeenNtPoseTimestamp) {
            lastSeenNtPoseTimestamp = estimate.timestampSeconds;
            // Use receipt time for freshness check
            lastDataFpgaTime = Math.max(lastDataFpgaTime, nowFpga);
          }
        };

    if (enablePoseEstimation) {
      estimatorMegatag1
          .getPoseEstimate()
          .ifPresent(pe -> addObservation.accept(pe, PoseObservationType.MEGATAG_1));
      estimatorMegatag2
          .getPoseEstimate()
          .ifPresent(pe -> addObservation.accept(pe, PoseObservationType.MEGATAG_2));
    }

    // Hybrid connection check: table present AND (fresh heartbeat OR fresh usable data)
    boolean tableAvailable = Limelight.isAvailable(camera.limelightName);
    boolean freshHeartbeat =
        ((RobotController.getFPGATime() - heartbeatLatencySub.getLastChange()) / 1000.0) <= 250.0;
    boolean freshData = (nowFpga - lastDataFpgaTime) <= 0.25;
    inputs.connected = tableAvailable && (freshHeartbeat || freshData);

    // Save pose observations to inputs object
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }
}
