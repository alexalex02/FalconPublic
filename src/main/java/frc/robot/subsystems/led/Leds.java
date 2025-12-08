package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Superstructure.Mode;
import frc.robot.subsystems.led.LedIO.LedPattern;
import java.util.EnumMap;
import java.util.Map;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Leds extends SubsystemBase {
  private final LedIO io;
  private final LedIOInputsAutoLogged inputs = new LedIOInputsAutoLogged();
  private final Supplier<Mode> modeSupplier;
  private final Map<Mode, LedEffect> modeEffects = new EnumMap<>(Mode.class);

  private LedEffect activeEffect = LedEffect.solid(0, 0, 0);
  private double lastBlinkToggleTime = Timer.getFPGATimestamp();
  private boolean blinkOn = true;

  public Leds(LedIO io, Supplier<Mode> modeSupplier) {
    this.io = io;
    this.modeSupplier = modeSupplier;
    populateDefaultEffects();
  }

  private void populateDefaultEffects() {
    modeEffects.put(Mode.IDLE, LedEffect.solid(255, 0, 0));
    modeEffects.put(Mode.AUTOALIGN, LedEffect.blink(0, 255, 0, 0.25));
    modeEffects.put(Mode.TUNABLEAUTOALIGN, LedEffect.blinkBetween(0, 255, 0, 0, 0, 255, 0.25));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("LEDs", inputs);

    Mode currentMode = modeSupplier.get();
    LedEffect desiredEffect =
        DriverStation.isDisabled()
            ? LedEffect.larsonRed(0.4)
            : modeEffects.getOrDefault(currentMode, LedEffect.solid(0, 0, 0));

    boolean effectChanged = !desiredEffect.equals(activeEffect);
    applyEffect(desiredEffect, effectChanged);
    activeEffect = desiredEffect;
  }

  private void applyEffect(LedEffect effect, boolean effectChanged) {
    switch (effect.type()) {
      case SOLID -> {
        if (effectChanged) {
          io.setPattern(effect.pattern());
        }
      }
      case LARSON_RED -> {
        io.setLarsonRed(effect.brightness());
      }
      case BLINK -> updateBlink(effect, effectChanged);
    }
    Logger.recordOutput("LEDs/ActiveEffect", effect.type().name());
  }

  private void updateBlink(LedEffect effect, boolean effectChanged) {
    double now = Timer.getFPGATimestamp();

    if (effectChanged) {
      blinkOn = true;
      lastBlinkToggleTime = now;
      io.setPattern(effect.pattern());
      return;
    }

    double halfPeriod = Math.max(0.05, effect.periodSec()) / 2.0;
    if (now - lastBlinkToggleTime >= halfPeriod) {
      blinkOn = !blinkOn;
      lastBlinkToggleTime = now;
      io.setPattern(blinkOn ? effect.pattern() : effect.altPattern());
    }
  }

  /**
   * Lightweight descriptor for LED output. Encodes intent (solid color vs animation) without
   * storing runnable animation objects.
   */
  public static record LedEffect(
      Type type, LedPattern pattern, LedPattern altPattern, double brightness, double periodSec) {
    public enum Type {
      SOLID,
      LARSON_RED,
      BLINK
    }

    public LedEffect {
      brightness = clampBrightness(brightness);
      pattern = pattern == null ? LedPattern.off() : pattern;
      altPattern = altPattern == null ? LedPattern.off() : altPattern;
      periodSec = Math.max(0.0, periodSec);
    }

    /** Convenience constructor for solid colors. */
    public static LedEffect solid(int red, int green, int blue) {
      return new LedEffect(
          Type.SOLID, new LedPattern(red, green, blue), LedPattern.off(), 0.0, 0.0);
    }

    /** Convenience constructor for solid colors. */
    public static LedEffect solid(LedPattern pattern) {
      return new LedEffect(Type.SOLID, pattern, LedPattern.off(), 0.0, 0.0);
    }

    /** Larson/cylon animation with bounded brightness (0-1). */
    public static LedEffect larsonRed(double brightness) {
      return new LedEffect(Type.LARSON_RED, LedPattern.off(), LedPattern.off(), brightness, 0.0);
    }

    /** Simple on/off blink with the given period in seconds. */
    public static LedEffect blink(int red, int green, int blue, double periodSec) {
      return new LedEffect(
          Type.BLINK, new LedPattern(red, green, blue), LedPattern.off(), 0.0, periodSec);
    }

    /** Simple on/off blink with the given period in seconds. */
    public static LedEffect blink(LedPattern pattern, double periodSec) {
      return new LedEffect(Type.BLINK, pattern, LedPattern.off(), 0.0, periodSec);
    }

    /** Blink between two colors with the given period in seconds. */
    public static LedEffect blinkBetween(
        LedPattern patternA, LedPattern patternB, double periodSec) {
      return new LedEffect(Type.BLINK, patternA, patternB, 0.0, periodSec);
    }

    /** Blink between two colors with the given period in seconds. */
    public static LedEffect blinkBetween(
        int redA, int greenA, int blueA, int redB, int greenB, int blueB, double periodSec) {
      return new LedEffect(
          Type.BLINK,
          new LedPattern(redA, greenA, blueA),
          new LedPattern(redB, greenB, blueB),
          0.0,
          periodSec);
    }

    private static double clampBrightness(double value) {
      return Math.max(0.0, Math.min(1.0, value));
    }
  }
}
