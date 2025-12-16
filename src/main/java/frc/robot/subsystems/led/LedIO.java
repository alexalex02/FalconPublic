package frc.robot.subsystems.led;

import org.littletonrobotics.junction.AutoLog;

/** IO interface for LED hardware so real and sim paths share a common API. */
public interface LedIO {
  @AutoLog
  public static class LedIOInputs {
    public boolean connected = false;
    public int red = 0;
    public int green = 0;
    public int blue = 0;
    public String activeMode = "UNKNOWN";
    public String implementation = "";
  }

  /** Populate inputs for logging/replay. */
  public default void updateInputs(LedIOInputs inputs) {}

  /** Command a solid RGB pattern to the LEDs. */
  public default void setPattern(LedPattern pattern) {}

  /** Run a red Larson animation */
  public default void setLarsonRed(double brightness) {}

  /** Stop any running animations so solid patterns fully apply. */
  public default void clearAnimations() {}

  /** Simple RGB pattern container for the LED subsystem. */
  public static record LedPattern(int red, int green, int blue) {
    public LedPattern {
      red = clamp(red);
      green = clamp(green);
      blue = clamp(blue);
    }

    public static LedPattern off() {
      return new LedPattern(0, 0, 0);
    }

    private static int clamp(int value) {
      return Math.max(0, Math.min(255, value));
    }
  }
}
