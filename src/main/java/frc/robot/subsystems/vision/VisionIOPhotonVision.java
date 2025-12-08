// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision implements VisionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCameraStatic;
  protected final Supplier<Transform3d> robotToCameraSupplier;
  protected final boolean enablePoseEstimation;

  /**
   * Creates a new VisionIOPhotonVision.
   *
   * @param name The configured name of the camera.
   * @param rotationSupplier The 3D position of the camera relative to the robot.
   */
  public VisionIOPhotonVision(String name, Transform3d robotToCamera) {
    this(name, robotToCamera, true);
  }

  /**
   * Creates a new VisionIOPhotonVision with optional pose estimation.
   *
   * @param name The configured name of the camera.
   * @param robotToCamera The 3D position of the camera relative to the robot.
   * @param enablePoseEstimation If false, only tx/ty and tag IDs are populated (no pose samples).
   */
  public VisionIOPhotonVision(
      String name, Transform3d robotToCamera, boolean enablePoseEstimation) {
    camera = new PhotonCamera(name);
    this.robotToCameraStatic = robotToCamera;
    this.robotToCameraSupplier = null;
    this.enablePoseEstimation = enablePoseEstimation;
  }

  /**
   * Creates a new VisionIOPhotonVision with optional pose estimation and a dynamic camera
   * transform.
   *
   * @param name The configured name of the camera.
   * @param robotToCameraSupplier Supplier for the 3D position of the camera relative to the robot.
   * @param enablePoseEstimation If false, only tx/ty and tag IDs are populated (no pose samples).
   */
  public VisionIOPhotonVision(
      String name, Supplier<Transform3d> robotToCameraSupplier, boolean enablePoseEstimation) {
    camera = new PhotonCamera(name);
    this.robotToCameraStatic = null;
    this.robotToCameraSupplier = robotToCameraSupplier;
    this.enablePoseEstimation = enablePoseEstimation;
  }

  /** Returns the current robot-to-camera transform (dynamic if a supplier was provided). */
  protected Transform3d getRobotToCamera() {
    if (robotToCameraSupplier != null) {
      return robotToCameraSupplier.get();
    }
    return robotToCameraStatic;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();

    // Read new camera observations
    Set<Short> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    for (var result : camera.getAllUnreadResults()) {
      // Update latest target observation
      if (result.hasTargets()) {
        inputs.latestTargetObservation =
            new TargetObservation(
                Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
      } else {
        inputs.latestTargetObservation = new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);
      }

      // Add observations and tag IDs
      if (result.multitagResult.isPresent()) { // Multitag result
        var multitagResult = result.multitagResult.get();

        // Add tag IDs regardless of pose mode
        tagIds.addAll(multitagResult.fiducialIDsUsed);

        if (enablePoseEstimation) {
          Transform3d robotToCamera = getRobotToCamera();
          // Calculate robot pose
          Transform3d fieldToCamera = multitagResult.estimatedPose.best;
          Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
          Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

          // Calculate average tag distance
          double totalTagDistance = 0.0;
          for (var target : result.targets) {
            totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
          }

          // Add observation
          poseObservations.add(
              new PoseObservation(
                  result.getTimestampSeconds(), // Timestamp
                  robotPose, // 3D pose estimate
                  multitagResult.estimatedPose.ambiguity, // Ambiguity
                  multitagResult.fiducialIDsUsed.size(), // Tag count
                  totalTagDistance / Math.max(1, result.targets.size()), // Avg tag distance
                  PoseObservationType.PHOTONVISION)); // Observation type
        }

      } else if (!result.targets.isEmpty()) { // Single tag result
        var target = result.targets.get(0);

        // Add tag ID regardless of pose mode
        tagIds.add((short) target.fiducialId);

        if (enablePoseEstimation) {
          Transform3d robotToCamera = getRobotToCamera();
          // Calculate robot pose
          var tagPose = aprilTagLayout.getTagPose(target.fiducialId);
          if (tagPose.isPresent()) {
            Transform3d fieldToTarget =
                new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
            Transform3d cameraToTarget = target.bestCameraToTarget;
            Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
            Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
            Pose3d robotPose =
                new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

            // Add observation
            poseObservations.add(
                new PoseObservation(
                    result.getTimestampSeconds(), // Timestamp
                    robotPose, // 3D pose estimate
                    target.poseAmbiguity, // Ambiguity
                    1, // Tag count
                    cameraToTarget.getTranslation().getNorm(), // Average tag distance
                    PoseObservationType.PHOTONVISION)); // Observation type
          }
        }
      }
    }

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
