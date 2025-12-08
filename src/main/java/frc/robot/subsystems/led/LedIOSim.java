package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.SubsystemConstants;
import frc.robot.subsystems.led.LedIO.LedPattern;

/** Simple simulated LED strip using WPILib's AddressableLED. */
public class LedIOSim implements LedIO {
  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;
  private LedPattern lastPattern = LedPattern.off();
  private double lastLarsonToggle = Timer.getFPGATimestamp();
  private int larsonIndex = 0;
  private int larsonDirection = 1;

  public LedIOSim() {
    led = new AddressableLED(0); // PWM port 0; adjust if needed
    buffer = new AddressableLEDBuffer(SubsystemConstants.LED_COUNT);
    led.setLength(buffer.getLength());
    led.start();
  }

  @Override
  public void updateInputs(LedIOInputs inputs) {
    inputs.connected = true;
    inputs.red = lastPattern.red();
    inputs.green = lastPattern.green();
    inputs.blue = lastPattern.blue();
    inputs.activeMode = "SIM";
    inputs.implementation = "AddressableLED";
  }

  @Override
  public void setPattern(LedPattern pattern) {
    if (pattern.equals(lastPattern)) {
      return;
    }
    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setRGB(i, pattern.red(), pattern.green(), pattern.blue());
    }
    led.setData(buffer);
    lastPattern = pattern;
  }

  @Override
  public void setLarsonRed(double brightness) {
    double now = Timer.getFPGATimestamp();
    double stepPeriod = 0.05; // seconds between steps
    int intensity = (int) Math.round(Math.max(0.0, Math.min(1.0, brightness)) * 255.0);

    if (now - lastLarsonToggle >= stepPeriod) {
      lastLarsonToggle = now;
      larsonIndex += larsonDirection;
      if (larsonIndex >= buffer.getLength() - 1 || larsonIndex <= 0) {
        larsonDirection *= -1;
      }
    }

    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setRGB(i, 0, 0, 0);
    }
    buffer.setRGB(larsonIndex, intensity, 0, 0);
    led.setData(buffer);
    lastPattern = new LedPattern(intensity, 0, 0);
  }
}
