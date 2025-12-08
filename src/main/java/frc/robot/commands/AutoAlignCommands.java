package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.AutoAlignTunables;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.Reef.CoralScoreLevel;
import frc.robot.constants.FieldConstants.Reef.PipeSide;
import frc.robot.constants.FieldConstants.Reef.ReefFaces;
import frc.robot.subsystems.drive.Drive;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Factory for on-demand auto alignment commands that drive to the nearest reef post. */
public final class AutoAlignCommands {
  private AutoAlignCommands() {}

  /** Drives to a tunable target pose using a profiled PID translation/heading controller */
  public static Command driveToTunablePoseProfiled(Drive drive) {
    double translationKp = AutoAlignTunables.XY_KP.get();
    double translationMaxVel =
        Math.min(drive.getMaxLinearSpeedMetersPerSec(), AutoAlignTunables.XY_MAX_VEL_MPS.get());
    double translationMaxAcc = AutoAlignTunables.XY_MAX_ACC_MPS2.get();
    TrapezoidProfile.Constraints translationConstraints =
        new TrapezoidProfile.Constraints(translationMaxVel, translationMaxAcc);
    ProfiledPIDController translationController =
        new ProfiledPIDController(translationKp, 0.0, 0.0, translationConstraints);

    double thetaKp = AutoAlignTunables.THETA_KP.get();
    double thetaMaxVel =
        Math.min(
            drive.getMaxAngularSpeedRadPerSec(), AutoAlignTunables.THETA_MAX_VEL_RAD_PER_SEC.get());
    double thetaMaxAcc = AutoAlignTunables.THETA_MAX_ACC_RAD_PER_SEC2.get();
    ProfiledPIDController thetaController =
        new ProfiledPIDController(
            thetaKp, 0.0, 0.0, new TrapezoidProfile.Constraints(thetaMaxVel, thetaMaxAcc));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    final boolean[] translationInitialized = new boolean[1];
    final Pose2d[] lastTarget = new Pose2d[1];

    return Commands.run(
            () -> {
              Pose2d desired =
                  new Pose2d(
                      AutoAlignTunables.TARGET_X_METERS.get(),
                      AutoAlignTunables.TARGET_Y_METERS.get(),
                      Rotation2d.fromDegrees(AutoAlignTunables.TARGET_THETA_DEG.get()));

              // If the target changed, re-seed the translation controller
              if (lastTarget[0] == null || !desired.equals(lastTarget[0])) {
                translationInitialized[0] = false;
                lastTarget[0] = desired;

                Logger.recordOutput("AutoAlign/Active", true);
                Logger.recordOutput("AutoAlign/TargetPose", desired);
                Logger.recordOutput("AutoAlign/Tunables/XYkP", translationKp);
                Logger.recordOutput("AutoAlign/Tunables/XYMaxVel", translationMaxVel);
                Logger.recordOutput("AutoAlign/Tunables/XYMaxAcc", translationMaxAcc);
                Logger.recordOutput("AutoAlign/Tunables/ThetaKp", thetaKp);
                Logger.recordOutput("AutoAlign/Tunables/ThetaMaxVel", thetaMaxVel);
                Logger.recordOutput("AutoAlign/Tunables/ThetaMaxAcc", thetaMaxAcc);
              }

              Pose2d current = drive.getPose();
              double errorX = desired.getX() - current.getX();
              double errorY = desired.getY() - current.getY();
              double distance = Math.hypot(errorX, errorY);

              if (!translationInitialized[0]) {
                translationController.reset(-distance);
                translationInitialized[0] = true;
              }

              double dirX = distance > 1e-4 ? errorX / distance : 0.0;
              double dirY = distance > 1e-4 ? errorY / distance : 0.0;
              double speed =
                  translationController.calculate(-distance, 0.0)
                      + translationController.getSetpoint().velocity;

              double vx = speed * dirX;
              double vy = speed * dirY;
              double omega =
                  thetaController.calculate(
                          current.getRotation().getRadians(), desired.getRotation().getRadians())
                      + thetaController.getSetpoint().velocity;

              double maxLin = translationMaxVel;
              double maxOmega = thetaMaxVel;
              vx = Math.max(-maxLin, Math.min(maxLin, vx));
              vy = Math.max(-maxLin, Math.min(maxLin, vy));
              omega = Math.max(-maxOmega, Math.min(maxOmega, omega));

              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, current.getRotation()));
            },
            drive)
        .beforeStarting(
            () -> {
              translationController.reset(0.0);
              translationInitialized[0] = false;
              thetaController.reset(drive.getRotation().getRadians());
            })
        .finallyDo(
            () -> {
              Logger.recordOutput("AutoAlign/Active", false);
              Logger.recordOutput(
                  "AutoAlign/TargetPose",
                  new Pose2d(Double.NaN, Double.NaN, Rotation2d.fromRadians(Double.NaN)));
            })
        .until(
            () ->
                translationController.atGoal()
                    && thetaController.atGoal()) // ends when within tolerances
        .withName("AutoAlign PID (tunable pose)");
  }
  /**
   * Aligns to the nearest reef post using a single profiled PID controller for translation
   * (magnitude limited) and a separate profiled PID for theta.
   */
  public static Command alignToNearestReefPost(Drive drive, PipeSide side, CoralScoreLevel level) {

    double translationKp = AutoAlignTunables.XY_KP.get();
    double translationMaxVel =
        Math.min(drive.getMaxLinearSpeedMetersPerSec(), AutoAlignTunables.XY_MAX_VEL_MPS.get());
    double translationMaxAcc = AutoAlignTunables.XY_MAX_ACC_MPS2.get();
    TrapezoidProfile.Constraints translationConstraints =
        new TrapezoidProfile.Constraints(translationMaxVel, translationMaxAcc);
    ProfiledPIDController translationController =
        new ProfiledPIDController(translationKp, 0.0, 0.0, translationConstraints);
    translationController.setTolerance(0.02); // 2 cm

    double thetaKp = AutoAlignTunables.THETA_KP.get();
    double thetaMaxVel =
        Math.min(
            drive.getMaxAngularSpeedRadPerSec(), AutoAlignTunables.THETA_MAX_VEL_RAD_PER_SEC.get());
    double thetaMaxAcc = AutoAlignTunables.THETA_MAX_ACC_RAD_PER_SEC2.get();
    TrapezoidProfile.Constraints thetaConstraints =
        new TrapezoidProfile.Constraints(thetaMaxVel, thetaMaxAcc);
    ProfiledPIDController thetaController =
        new ProfiledPIDController(thetaKp, 0.0, 0.0, thetaConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(Math.toRadians(1));

    final Pose2d[] goalPose = new Pose2d[1];
    final boolean[] translationInitialized = new boolean[1];

    return Commands.run(
            () -> {
              if (goalPose[0] == null) {
                Pose2d robotPose = drive.getPose();
                Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
                ReefFaces face =
                    FieldConstants.Reef.nearestFace(robotPose.getTranslation(), alliance);
                Pose2d goal = FieldConstants.Reef.coralPose(face, level, side, alliance);
                goalPose[0] = goal;

                Logger.recordOutput("AutoAlign/Active", true);
                Logger.recordOutput("AutoAlign/TargetPose", goal);
                Logger.recordOutput("AutoAlign/Tunables/XYkP", translationKp);
                Logger.recordOutput("AutoAlign/Tunables/XYMaxVel", translationMaxVel);
                Logger.recordOutput("AutoAlign/Tunables/XYMaxAcc", translationMaxAcc);
                Logger.recordOutput("AutoAlign/Tunables/ThetaKp", thetaKp);
                Logger.recordOutput("AutoAlign/Tunables/ThetaMaxVel", thetaMaxVel);
                Logger.recordOutput("AutoAlign/Tunables/ThetaMaxAcc", thetaMaxAcc);
              }

              Pose2d current = drive.getPose();
              Pose2d desired = goalPose[0];

              double errorX = desired.getX() - current.getX();
              double errorY = desired.getY() - current.getY();
              double distance = Math.hypot(errorX, errorY);

              if (!translationInitialized[0]) {
                // Start the profiled controller at the current distance with zero velocity
                translationController.reset(-distance);
                translationInitialized[0] = true;
              }

              double dirX = distance > 1e-4 ? errorX / distance : 0.0;
              double dirY = distance > 1e-4 ? errorY / distance : 0.0;
              double speed =
                  translationController.calculate(-distance, 0.0)
                      + translationController.getSetpoint().velocity;

              double vx = speed * dirX;
              double vy = speed * dirY;
              double omega =
                  thetaController.calculate(
                          current.getRotation().getRadians(), desired.getRotation().getRadians())
                      + thetaController.getSetpoint().velocity;

              double maxLin = 6;
              double maxOmega = thetaMaxVel;
              vx = Math.max(-maxLin, Math.min(maxLin, vx));
              vy = Math.max(-maxLin, Math.min(maxLin, vy));
              omega = Math.max(-maxOmega, Math.min(maxOmega, omega));

              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, current.getRotation()));
            },
            drive)
        .beforeStarting(
            () -> {
              Pose2d current = drive.getPose();
              translationController.reset(0.0);
              translationInitialized[0] = false;
              thetaController.reset(current.getRotation().getRadians());
            })
        .finallyDo(
            () -> {
              Logger.recordOutput("AutoAlign/Active", false);
              Logger.recordOutput(
                  "AutoAlign/TargetPose",
                  new Pose2d(Double.NaN, Double.NaN, Rotation2d.fromRadians(Double.NaN)));
            })
        .withName("AutoAlign PID " + side.name());
  }

  /** Aligns to nearest post using profiled PID with dynamic level supplier. */
  public static Command alignToNearestReefPost(
      Drive drive, PipeSide side, Supplier<CoralScoreLevel> levelSupplier) {
    return Commands.defer(
            () -> alignToNearestReefPost(drive, side, levelSupplier.get()), Set.of(drive))
        .withName("AutoAlign PID " + side.name() + " (dynamic level)");
  }
}
