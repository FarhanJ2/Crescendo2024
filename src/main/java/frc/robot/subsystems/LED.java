// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LED extends SubsystemBase {

  private AddressableLED LEDStrip;
  private AddressableLEDBuffer LEDBuffer;
  private static int rainbowStart = 0;
  private double lastChange;
  private boolean isOn;
  private static int waveIndex = 0;
  private static final int waveLength = 25; //15
  private static LEDColor currentColor = LEDColor.OFF;

  public enum LEDColor {
    PURPLE(70, 2, 115),
    YELLOW(150, 131, 2),
    RED(255, 0, 0),
    BLUE(0, 0, 255),
    GREEN(0, 255, 0),
    WHITE(255, 255, 255),
    CYAN(0, 255, 255),
    ORANGE(252, 144, 3),
    OFF(0, 0, 0); 

    public int r;
    public int g;
    public int b;

    private LEDColor(int r, int g, int b) {
        this.r = r;
        this.g = g;
        this.b = b;
    }
}

public enum LEDMode {
    STATIC,
    WAVE,
    RAINBOW,
    PULSE;

    private LEDMode() {}
}

  public LED() {
    LEDStrip = new AddressableLED(Constants.Led.port);
    LEDBuffer = new AddressableLEDBuffer(Constants.Led.length);

    LEDStrip.setLength(LEDBuffer.getLength());
   
    LEDStrip.setData(LEDBuffer);
    LEDStrip.start();
  }

  public void setColor(LEDColor color) {
    for(int i = 0; i < LEDBuffer.getLength(); i++){
      LEDBuffer.setRGB(i, color.r, color.g, color.b);
    }

    currentColor = color;
    LEDStrip.setData(LEDBuffer);
  }

  public void pulse(LEDColor color, double interval) {
    double timestamp = Timer.getFPGATimestamp();

    if (timestamp - lastChange > interval) {
      lastChange = timestamp;
      isOn = !isOn;
    }

    if (isOn) {
      stop();
    }
    else {
      setColor(color);
    }
  }

  public void wave(LEDColor color) {
    if (waveIndex == 0) {
        for (byte i = 0; i <= waveLength; i++) {
            this.LEDBuffer.setRGB(i, color.r, color.g, color.b);
        }
    } else {
        for (byte i = 0; i < this.LEDBuffer.getLength() - 2; i++) {
            if (waveIndex > 0) this.LEDBuffer.setRGB(waveIndex - 1, 0, 0, 0);

            if (waveIndex + waveLength < LEDBuffer.getLength() - 1) {
                this.LEDBuffer.setRGB(waveIndex + waveLength, color.r, color.g, color.b);
            } else {
                this.LEDBuffer.setRGB(waveIndex + waveLength - (LEDBuffer.getLength() - 1), color.r, color.g, color.b);
            }

            if (waveIndex > LEDBuffer.getLength() - 1) waveIndex = -1;
        }
    }
    waveIndex += 1;

    currentColor = LEDColor.OFF;
    this.LEDStrip.setData(this.LEDBuffer);
}

  public void rainbow() {
    for (int i = 0; i < LEDBuffer.getLength(); i++) {
      i %= LEDBuffer.getLength();

      final var hue = (rainbowStart + (i * 180 / LEDBuffer.getLength())) % 180;
      LEDBuffer.setHSV(i, hue, 255, 128);
    }

    currentColor = LEDColor.OFF;
    LEDStrip.setData(LEDBuffer);

    rainbowStart += 3;
    rainbowStart %= 180;
  }

  public LEDColor getCurrentColor() {
    return currentColor;
  }

  public void stop() {
    setColor(LEDColor.OFF);
  }

  @Override
  public void periodic() {}
}