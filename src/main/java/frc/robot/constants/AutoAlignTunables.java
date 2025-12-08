package frc.robot.constants;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/** Tunable gains and limits for auto-align controllers. */
public final class AutoAlignTunables {
  private AutoAlignTunables() {}

  public static final LoggedNetworkNumber XY_KP =
      new LoggedNetworkNumber("/Tuning/AutoAlign/XYkP", 5.0);
  public static final LoggedNetworkNumber XY_MAX_VEL_MPS =
      new LoggedNetworkNumber("/Tuning/AutoAlign/XYMaxVelMps", 3.0);
  public static final LoggedNetworkNumber XY_MAX_ACC_MPS2 =
      new LoggedNetworkNumber("/Tuning/AutoAlign/XYMaxAccMps2", 3.0);

  public static final LoggedNetworkNumber THETA_KP =
      new LoggedNetworkNumber("/Tuning/AutoAlign/ThetaKp", 5.0);
  public static final LoggedNetworkNumber THETA_MAX_VEL_RAD_PER_SEC =
      new LoggedNetworkNumber("/Tuning/AutoAlign/ThetaMaxVelRadPerSec", 8.0);
  public static final LoggedNetworkNumber THETA_MAX_ACC_RAD_PER_SEC2 =
      new LoggedNetworkNumber("/Tuning/AutoAlign/ThetaMaxAccRadPerSec2", 20.0);

  // Target pose (field coordinates, meters / degrees)
  public static final LoggedNetworkNumber TARGET_X_METERS =
      new LoggedNetworkNumber("/Tuning/AutoAlign/TargetPoseX", 0.0);
  public static final LoggedNetworkNumber TARGET_Y_METERS =
      new LoggedNetworkNumber("/Tuning/AutoAlign/TargetPoseY", 0.0);
  public static final LoggedNetworkNumber TARGET_THETA_DEG =
      new LoggedNetworkNumber("/Tuning/AutoAlign/TargetPoseThetaDeg", 0.0);

  // Trigger to drive to the tunable pose (set true in Elastic to start)
  public static final LoggedNetworkBoolean DRIVE_TO_TUNABLE_TRIGGER =
      new LoggedNetworkBoolean("/Tuning/AutoAlign/DriveToTunablePose", false);
}
