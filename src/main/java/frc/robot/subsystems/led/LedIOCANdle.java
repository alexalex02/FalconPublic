package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.signals.VBatOutputModeValue;
import frc.robot.constants.SubsystemConstants;
import frc.robot.subsystems.led.LedIO.LedPattern;

/** Real-hardware LED IO */
public class LedIOCANdle implements LedIO {
  private final CANdle candle;
  private final SolidColor solidColorRequest;
  private final int ledCount = SubsystemConstants.LED_COUNT;
  private LedPattern lastPattern = LedPattern.off();

  public LedIOCANdle() {
    candle = new CANdle(SubsystemConstants.CANDLE_ID, SubsystemConstants.RIOCANBUS);
    solidColorRequest = new SolidColor(0, Math.max(0, SubsystemConstants.LED_COUNT - 1));

    CANdleConfiguration config = new CANdleConfiguration();
    config.LED.StripType = StripTypeValue.GRB;
    config.LED.BrightnessScalar = 1.0;
    config.CANdleFeatures.VBatOutputMode = VBatOutputModeValue.Modulated;
    candle.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(LedIOInputs inputs) {
    double busVoltage = candle.getSupplyVoltage().refresh().getValue().in(Volts);
    inputs.connected = busVoltage > 1.0;
    inputs.red = lastPattern.red();
    inputs.green = lastPattern.green();
    inputs.blue = lastPattern.blue();
    inputs.activeMode = "CANdle";
    inputs.implementation = "CANdle";
  }

  @Override
  public void setPattern(LedPattern pattern) {
    if (pattern == null) {
      pattern = LedPattern.off();
    }
    lastPattern = pattern;
    solidColorRequest.Color = new RGBWColor(pattern.red(), pattern.green(), pattern.blue());
    solidColorRequest.LEDEndIndex = Math.max(0, ledCount - 1);
    candle.setControl(solidColorRequest);
  }

  @Override
  public void setLarsonRed(double brightness) {
    double clampedBrightness = Math.max(0.0, Math.min(1.0, brightness));
    int endIndex = Math.max(0, ledCount - 1);
    var anim = new LarsonAnimation(0, endIndex);
    anim.Slot = 0;
    anim.Color = new RGBWColor((int) Math.round(255 * clampedBrightness), 0, 0);
    anim.Size = 6;
    anim.BounceMode = LarsonBounceValue.Front;
    anim.FrameRate = 20; // Hz
    candle.setControl(anim);
  }
}
