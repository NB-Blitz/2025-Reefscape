package frc.robot.util;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;

public class LEDStrip {
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  private AddressableLEDBufferView m_leftBuffer;
  private AddressableLEDBufferView m_rightBuffer;

  private int blinkCount;
  private boolean rainbowOverride = false;

  public LEDStrip(int pwmPort, int numPixels) {
    m_led = new AddressableLED(pwmPort);
    m_ledBuffer = new AddressableLEDBuffer(numPixels);
    m_leftBuffer = m_ledBuffer.createView(0, numPixels / 2 - 1);
    m_rightBuffer = m_ledBuffer.createView(numPixels / 2, numPixels - 1).reversed();

    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();

    blinkCount = 0;
  }

  public void updateLEDs(int atIntake, boolean holdingCoral, double ledRatio) {
    LEDPattern pattern;
    if (rainbowOverride) {
      pattern = setRainbow();
    } else if (atIntake == 1 && !holdingCoral) {
      pattern = setSolid(Color.kRed);
      blinkCount = 0;
    } else if (holdingCoral && blinkCount < 5) {
      pattern = setBlinking(Color.kGreen);
      blinkCount++;
    } else {
      pattern = setSolid(Color.kGold);
      pattern = progressMask(pattern, ledRatio);
    }

    pattern.applyTo(m_leftBuffer);
    pattern.applyTo(m_rightBuffer);

    m_led.setData(m_ledBuffer);
  }

  public LEDPattern progressMask(LEDPattern pattern, double ledRatio) {
    LEDPattern mask = LEDPattern.progressMaskLayer(() -> ledRatio);
    return pattern.mask(mask);
  }

  public LEDPattern setGradient(Color bottom, Color top) {
    return LEDPattern.gradient(GradientType.kDiscontinuous, bottom, top);
  }

  public LEDPattern setSolid(Color color) {
    return LEDPattern.solid(color);
  }

  public LEDPattern setBlinking(Color color) {
    LEDPattern base = LEDPattern.solid(Color.kGreen);
    return base.blink(Seconds.of(0.3)).atBrightness(Percent.of(50));
  }

  public LEDPattern setRainbow() {
    return LEDPattern.rainbow(255, 128).scrollAtRelativeSpeed(Percent.per(Second).of(25));
  }

  public void setRainbowOverride(boolean override) {
    rainbowOverride = override;
  }
}
