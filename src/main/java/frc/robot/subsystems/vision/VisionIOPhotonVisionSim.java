// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
  private static VisionSystemSim visionSim;

  private final Supplier<Pose2d> poseSupplier;
  private final PhotonCameraSim cameraSim;

  /**
   * Creates a new VisionIOPhotonVisionSim.
   *
   * @param name The name of the camera.
   * @param poseSupplier Supplier for the robot pose to use in simulation.
   */
  public VisionIOPhotonVisionSim(
      String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
    this(name, robotToCamera, poseSupplier, true);
  }

  /**
   * Creates a new VisionIOPhotonVisionSim with optional pose estimation.
   *
   * @param name The name of the camera.
   * @param robotToCamera The 3D position of the camera relative to the robot.
   * @param poseSupplier Supplier for the robot pose to use in simulation.
   * @param enablePoseEstimation If false, only tx/ty and tag IDs are populated (no pose samples).
   */
  public VisionIOPhotonVisionSim(
      String name,
      Transform3d robotToCamera,
      Supplier<Pose2d> poseSupplier,
      boolean enablePoseEstimation) {
    this(
        name,
        robotToCamera,
        poseSupplier,
        enablePoseEstimation,
        (java.util.function.Supplier<Transform3d>) null);
  }

  /**
   * Creates a new VisionIOPhotonVisionSim with optional pose estimation and a dynamic camera
   * transform supplier. If {@code robotToCameraDynamicSupplier} is non-null, the camera extrinsics
   * are updated each cycle to reflect camera motion in simulation.
   */
  public VisionIOPhotonVisionSim(
      String name,
      Transform3d robotToCamera,
      Supplier<Pose2d> poseSupplier,
      boolean enablePoseEstimation,
      Supplier<Transform3d> robotToCameraDynamicSupplier) {
    super(
        name,
        robotToCameraDynamicSupplier != null ? robotToCameraDynamicSupplier : () -> robotToCamera,
        enablePoseEstimation);
    this.poseSupplier = poseSupplier;

    // Initialize vision sim
    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(aprilTagLayout);
    }

    // Add sim camera
    var cameraProperties = new SimCameraProperties();
    cameraSim = new PhotonCameraSim(camera, cameraProperties, aprilTagLayout);
    Transform3d initialRobotToCamera =
        robotToCameraDynamicSupplier != null ? robotToCameraDynamicSupplier.get() : robotToCamera;
    visionSim.addCamera(cameraSim, initialRobotToCamera);
    cameraSim.enableRawStream(true);
    cameraSim.enableProcessedStream(true);
    cameraSim.enableDrawWireframe(true);
  }

  /**
   * Convenience overload: provide a yaw supplier (radians) to rotate the base transform's yaw while
   * keeping its translation fixed. This is a simple approximation suitable for rotating camera
   * mounts where the camera is near the rotation axis.
   */
  public VisionIOPhotonVisionSim(
      String name,
      Transform3d baseRobotToCamera,
      Supplier<Pose2d> poseSupplier,
      boolean enablePoseEstimation,
      DoubleSupplier yawRadSupplier) {
    this(
        name,
        baseRobotToCamera,
        poseSupplier,
        enablePoseEstimation,
        () -> composeYaw(baseRobotToCamera, yawRadSupplier.getAsDouble()));
  }

  private static Transform3d composeYaw(Transform3d base, double extraYawRad) {
    Rotation3d r = base.getRotation();
    Rotation3d r2 = new Rotation3d(r.getX(), r.getY(), r.getZ() + extraYawRad);
    return new Transform3d(base.getTranslation(), r2);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // If a dynamic camera transform supplier was provided, update the camera extrinsics to simulate
    // camera rotation or other motion in simulation.
    Transform3d currentRobotToCamera = getRobotToCamera();
    if (currentRobotToCamera != null) {
      // PhotonVision 2025 uses adjustCamera to update a camera's transform.
      visionSim.adjustCamera(cameraSim, currentRobotToCamera);
    }
    visionSim.update(poseSupplier.get());
    super.updateInputs(inputs);
  }
}
