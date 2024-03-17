package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class LED extends SubsystemBase {

  private AddressableLED LEDStrip;
  private AddressableLEDBuffer LEDBuffer;

  private double lastChange;
  private boolean isOn;
  private int waveIndex = 0;
  private int rainbowStart = 0;
  private int bounceWaveIndex = 0;
  private BounceWaveDirection bounceWaveDirection = BounceWaveDirection.FORWARD;

  private LEDColor currentColor = LEDColor.OFF;

  private static final int waveLength = 20;
  private static final int bounceWaveLength = 7;

  private double fadeMultiplier = 0;
  private FadeDirection fadeDirection = FadeDirection.IN;

  // private int strip2Start;
  // private int stripLength;

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
  }

  public enum FadeDirection {
    IN,
    OUT;
  }

  public enum BounceWaveDirection {
    FORWARD,
    BACKWARD;
  }

  public LED(int port, int length) {
    LEDStrip = new AddressableLED(port);
    LEDBuffer = new AddressableLEDBuffer(length);

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
    for (int i = 0; i < LEDBuffer.getLength(); i++) {
      if ((i >= waveIndex && i < waveIndex + waveLength)
      || (waveIndex + waveLength > LEDBuffer.getLength() && i < (waveIndex + waveLength) % LEDBuffer.getLength())) {
        this.LEDBuffer.setRGB(i, color.r, color.g, color.b);
      } else {
        this.LEDBuffer.setRGB(i, 0, 0, 0);
      }
    }

    waveIndex++;
    waveIndex %= LEDBuffer.getLength();

    currentColor = LEDColor.OFF;
    this.LEDStrip.setData(this.LEDBuffer);
  }

  public void bounceWave(LEDColor color) {
    for (int i = 0; i < LEDBuffer.getLength(); i++) {
      if (i >= bounceWaveIndex && i < bounceWaveIndex + bounceWaveLength) {
        this.LEDBuffer.setRGB(i, color.r, color.g, color.b);
      } else {
        this.LEDBuffer.setRGB(i, 0, 0, 0);
      }
    }

    if (bounceWaveIndex == 0) {
      bounceWaveDirection = BounceWaveDirection.FORWARD;
    } else if (bounceWaveIndex == LEDBuffer.getLength() - bounceWaveLength) {
      bounceWaveDirection = BounceWaveDirection.BACKWARD;
    }

    if (bounceWaveDirection == BounceWaveDirection.FORWARD) {
      bounceWaveIndex++;
    } else {
      bounceWaveIndex--;
    }

    currentColor = LEDColor.OFF;
    this.LEDStrip.setData(this.LEDBuffer);
  }

  public void rainbow() {
    for (int i = 0; i < LEDBuffer.getLength(); i++) {
      i %= LEDBuffer.getLength();

      final var hue = (rainbowStart + (i * 180 / LEDBuffer.getLength())) % 180;
      LEDBuffer.setHSV(i, hue, 255, 128); // Strip 1
    }

    currentColor = LEDColor.OFF;
    LEDStrip.setData(LEDBuffer);

    rainbowStart += 1;
    rainbowStart %= 180;
  }

  public void fade(LEDColor color) {
    for(int i = 0; i < LEDBuffer.getLength(); i++){
      LEDBuffer.setRGB(i, (int) (color.r * fadeMultiplier), (int) (color.g * fadeMultiplier), (int) (color.b * fadeMultiplier));
    }

    if (fadeMultiplier <= 0.02) {
      fadeDirection = FadeDirection.IN;
    } else if (fadeMultiplier >= 0.98) {
      fadeDirection = FadeDirection.OUT;
    }

    if (fadeDirection == FadeDirection.IN) {
      fadeMultiplier += 0.02;
    } else if (fadeDirection == FadeDirection.OUT) {
      fadeMultiplier -= 0.02;
    }

    currentColor = color;
    LEDStrip.setData(LEDBuffer);
  }

  public LEDColor getCurrentColor() {
    return currentColor;
  }

  ///////////////////////
  /* COMMAND FACTORIES */
  ///////////////////////

  public Command setColorCommand(LEDColor color) {
    return new InstantCommand(() -> this.setColor(color));
  }

  public Command rainbowCommand() {
    return Commands.run(this::rainbow, this);
  }

  public Command flashCommand(LEDColor color, double interval, double time) {
    return new ParallelDeadlineGroup(
      new WaitCommand(time),
      Commands.run(() -> this.pulse(color, interval), this)
    );
  }

  public Command flashUntilCommand(LEDColor color, double interval, BooleanSupplier condition) {
    return new ParallelDeadlineGroup(
      new WaitUntilCommand(condition),
      Commands.run(() -> this.pulse(color, interval), this)
    );
  }

  public Command waveCommand(LEDColor color) {
    return Commands.run(() -> this.wave(color), this);
  }

  public Command bounceWaveCommand(LEDColor color) {
    return Commands.run(() -> this.bounceWave(color), this);
  }

  public Command fadeCommand(LEDColor color) {
    return Commands.run(() -> this.fade(color), this);
  }

  public Command stopCommand() {
    return Commands.runOnce(this::stop, this);
  }

  public void stop() {
    setColor(LEDColor.OFF);
  }

  @Override
  public void periodic() {}
}