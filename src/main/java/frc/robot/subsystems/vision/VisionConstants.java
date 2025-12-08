// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "camera_0";
  public static String camera1Name = "camera_1";

  // Robot to camera transforms
  // (Configured on Limelight via withCameraOffset, PhotonVision via sim; keep base values for viz)
  public static Transform3d robotToCamera0 =
      new Transform3d(0.327481, 0.0, 0.181211, new Rotation3d(0.0, -0.349, 0.0));
  // Second camera transform (update to match your physical mount if used)
  public static Transform3d robotToCamera1 =
      new Transform3d(0.0, 0.0, 0.181211, new Rotation3d(0.0, -0.349, Math.PI));

  /** Convenience accessor for the configured robot-to-camera transform by index. */
  public static Transform3d getRobotToCamera(int cameraIndex) {
    return cameraIndex == 0 ? robotToCamera0 : robotToCamera1;
  }

  /** Pose form of {@link #getRobotToCamera(int)} for logging. */
  public static Pose3d getRobotToCameraPose(int cameraIndex) {
    return new Pose3d().transformBy(getRobotToCamera(cameraIndex));
  }

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
