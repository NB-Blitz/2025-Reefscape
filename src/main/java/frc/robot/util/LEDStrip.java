package frc.robot.util;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;

public class LEDStrip {
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  private LEDPattern m_colorBase;
  private LEDPattern m_Pattern;

  public LEDStrip(int pwmPort, int ledLength) {
    m_led = new AddressableLED(pwmPort);
    m_ledBuffer = new AddressableLEDBuffer(ledLength);
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
  }

  public void updateLEDs() {
    LEDPattern LEDStuff = m_colorBase.mask(m_Pattern);

    LEDStuff.applyTo(m_ledBuffer);

    m_led.setData(m_ledBuffer);
  }

  public void progressMask(double ledRatio) {
    m_Pattern = LEDPattern.progressMaskLayer(() -> ledRatio);
  }

  public void setGradient(Color bottom, Color top) {
    m_colorBase = LEDPattern.gradient(GradientType.kDiscontinuous, bottom, top);
  }
}
