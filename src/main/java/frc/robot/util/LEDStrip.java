package frc.robot.util;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;

public class LEDStrip {
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;

  public LEDStrip(int pwmPort, int numPixels) {
    m_led = new AddressableLED(pwmPort);
    m_ledBuffer = new AddressableLEDBuffer(numPixels);
    m_led.setLength(m_ledBuffer.getLength());
    m_led.start();
    m_led.setData(m_ledBuffer);
  }

  public void updateLEDs(boolean holdingCoral, double ledRatio) {
    LEDPattern pattern;
    if (holdingCoral) {
      pattern = setSolid(Color.kGreen);
    } else {
      pattern = setSolid(Color.kYellow);
      pattern = progressMask(pattern, ledRatio);
    }

    pattern.applyTo(m_ledBuffer);

    m_led.setData(m_ledBuffer);
  }

  public LEDPattern progressMask(LEDPattern pattern, double ledRatio) {
    // The strip is split in half between the robot's sides
    // Mirrors the effect on both sides
    double splitRatio = ledRatio * 0.5;
    LEDPattern mask1 = LEDPattern.progressMaskLayer(() -> splitRatio);
    LEDPattern masked1 = pattern.mask(mask1);
    LEDPattern mask2 = LEDPattern.progressMaskLayer(() -> splitRatio).reversed();
    LEDPattern masked2 = pattern.mask(mask2);
    return masked1.overlayOn(masked2);
  }

  public LEDPattern setGradient(Color bottom, Color top) {
    return LEDPattern.gradient(GradientType.kDiscontinuous, bottom, top);
  }

  public LEDPattern setSolid(Color color) {
    return LEDPattern.solid(color);
  }
}
