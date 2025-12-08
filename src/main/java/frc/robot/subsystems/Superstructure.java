package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * High-level subsystem coordination and state machine owner. Tracks the current robot intent (idle,
 * auto-align, etc.) so other subsystems (LEDs, future mechanisms) can react.
 */
public class Superstructure extends SubsystemBase {
  public enum Mode {
    IDLE,
    TUNABLEAUTOALIGN,
    AUTOALIGN
  }

  private Mode requestedMode = Mode.IDLE;
  private Mode activeMode = Mode.IDLE;

  public Superstructure() {}

  /** Update the requested mode (e.g., from driver commands or autonomous logic). */
  public void requestMode(Mode newMode) {
    requestedMode = newMode == null ? Mode.IDLE : newMode;
  }

  /** Returns the currently active mode used by other subsystems (e.g., LEDs). */
  public Mode getMode() {
    return activeMode;
  }

  @Override
  public void periodic() {
    // If disabled, force idle. Otherwise, follow the requested mode.
    Mode nextMode = DriverStation.isDisabled() ? Mode.IDLE : requestedMode;

    if (nextMode != activeMode) {
      activeMode = nextMode;
      Logger.recordOutput("Superstructure/Mode", activeMode.name());
    }
  }
}
