// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AutoAlignCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.constants.AutoAlignTunables;
import frc.robot.constants.FieldConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Mode;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.led.LedIO;
import frc.robot.subsystems.led.LedIOCANdle;
import frc.robot.subsystems.led.LedIOSim;
import frc.robot.subsystems.led.Leds;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import java.util.Set;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Superstructure superstructure;
  private final Leds leds;
  private final Vision vision;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private final LoggedDashboardChooser<FieldConstants.Reef.CoralScoreLevel> coralLevelChooser;

  // Selected coral scoring level (defaults to L4, overrideable in AdvantageScope)
  private FieldConstants.Reef.CoralScoreLevel coralLevel = FieldConstants.Reef.CoralScoreLevel.L4;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    try {
      FieldConstants.Reef.initFromTags();
    } catch (Exception e) {
      DriverStation.reportError("Failed to init reef constants from AprilTags.", e.getStackTrace());
    }

    LedIO ledIO;

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(camera0Name, drive::getRotation));
        ledIO = new LedIOCANdle();
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose));
        ledIO = new LedIOSim();
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {});
        ledIO = new LedIO() {};
        break;
    }

    superstructure = new Superstructure();
    leds = new Leds(ledIO, () -> superstructure.getMode());

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    coralLevelChooser = new LoggedDashboardChooser<>("Coral Level");
    coralLevelChooser.addDefaultOption("L4", FieldConstants.Reef.CoralScoreLevel.L4);
    coralLevelChooser.addOption("L3", FieldConstants.Reef.CoralScoreLevel.L3);
    coralLevelChooser.addOption("L2", FieldConstants.Reef.CoralScoreLevel.L2);
    coralLevelChooser.addOption("L1", FieldConstants.Reef.CoralScoreLevel.L1);
    coralLevelChooser.periodic();
    FieldConstants.Reef.CoralScoreLevel initialCoralLevel =
        coralLevelChooser.get() == null
            ? FieldConstants.Reef.CoralScoreLevel.L4
            : coralLevelChooser.get();
    setCoralLevel(initialCoralLevel);

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new Trigger(
            () -> {
              FieldConstants.Reef.CoralScoreLevel selected = coralLevelChooser.get();
              return selected != null && selected != coralLevel;
            })
        .onTrue(Commands.runOnce(() -> setCoralLevel(coralLevelChooser.get())));

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    new Trigger(() -> AutoAlignTunables.DRIVE_TO_TUNABLE_TRIGGER.get())
        .onTrue(
            wrapWithMode(
                    Commands.defer(
                        () -> AutoAlignCommands.driveToTunablePoseProfiled(drive), Set.of(drive)),
                    Mode.TUNABLEAUTOALIGN)
                .andThen(
                    Commands.runOnce(() -> AutoAlignTunables.DRIVE_TO_TUNABLE_TRIGGER.set(false))));

    // Auto-align to nearest reef post with bumpers (LEFT/RIGHT)
    controller
        .leftBumper()
        .whileTrue(
            Commands.defer(
                () -> {
                  return wrapWithMode(
                      AutoAlignCommands.alignToNearestReefPost(
                          drive, FieldConstants.Reef.PipeSide.LEFT, () -> coralLevel),
                      Mode.AUTOALIGN);
                },
                Set.of(drive)));
    controller
        .rightBumper()
        .whileTrue(
            Commands.defer(
                () -> {
                  return wrapWithMode(
                      AutoAlignCommands.alignToNearestReefPost(
                          drive, FieldConstants.Reef.PipeSide.RIGHT, () -> coralLevel),
                      Mode.AUTOALIGN);
                },
                Set.of(drive)));
  }

  private void setCoralLevel(FieldConstants.Reef.CoralScoreLevel level) {
    if (level == null) {
      return;
    }
    this.coralLevel = level;
    org.littletonrobotics.junction.Logger.recordOutput("AutoAlign/CoralLevel", level.name());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /** Wnter a mode for the duration of a command, then return to idle. */
  private Command wrapWithMode(Command command, Mode mode) {
    return command
        .beforeStarting(() -> superstructure.requestMode(mode))
        .finallyDo(interrupted -> superstructure.requestMode(Mode.IDLE));
  }
}
