package frc.robot.subsystems.led;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LED extends SubsystemBase {
  private final CANdle candle; // CANdle object to control the LED strip
  private static final int LED_COUNT = 90; // Total number of LEDs in the strip
  private Timer animationTimer = new Timer();

  // Team colors
  public static final Color orange = new Color(255, 25, 0);
  public static final Color black = new Color(0, 0, 0);

  // Game piece colors
  public static final Color yellow = new Color(242, 60, 0);
  public static final Color purple = new Color(184, 0, 185);

  // Indicator colors
  public static final Color white = new Color(255, 230, 220);
  public static final Color green = new Color(56, 209, 0);
  public static final Color blue = new Color(8, 32, 255);
  public static final Color red = new Color(255, 0, 0);

  public Color idle = new Color(255, 0, 0);

  public LED() {
    candle = new CANdle(Constants.RobotMap.CAN.CANDLE);
    candle.configFactoryDefault();
    candle.configLEDType(LEDStripType.RGBW);
  }

  public void setSolidColor(LEDSection section, Color color) {
    candle.setLEDs(color.red, color.green, color.blue, 0, section.start, section.length);
  }

  public void clearSection(LEDSection section) {
    candle.setLEDs(0, 0, 0, 0, section.start, section.length);
  }

  public Command setCurrentAnimationWithTimeout(
      Animation animation, LEDSection section, int seconds) {
    double startTime = Timer.getFPGATimestamp();
    return runOnce(
            () -> {
              candle.animate(animation, 0);
            })
        .until(() -> Timer.getFPGATimestamp() > (startTime + seconds))
        .andThen(
            () -> {
              candle.setLEDs(idle.red, idle.green, idle.blue, 0, section.start, section.length);
            });
  }
  // Animation Handlers

  public void setRainbow(double speed) {
    candle.animate(new RainbowAnimation(1, speed, LED_COUNT, false, 0));
  }

  public void setStrobeWhite(LEDSection section, double speed) {
    candle.animate(new StrobeAnimation(0, 0, 0, 255, speed, section.start, section.length));
  }

  public void setStrobe(Color color, LEDSection section, double speed) {
    candle.animate(
        new StrobeAnimation(
            color.red, color.green, color.blue, 0, speed, section.start, section.length));
  }

  public void setFade(Color color, LEDSection section, double speed) {
    candle.animate(
        new SingleFadeAnimation(
            color.red, color.green, color.blue, 0, speed, section.start, section.length));
  }

  // Enum to define different sections of the LED strip
  public enum LEDSection {
    ALL(0, 90, 0),
    CANdle(0, 8, 1),
    R45(8, 7, 2),
    L45(15, 6, 3),
    LEFTVERT(21, 9, 4),
    CROSSBAR(30, 14, 5),
    FRONTCHUTE(44, 13, 6),
    BACKCHUTE(57, 13, 7),
    CHUTEFULL(44, 26, 8);

    private final int start; // Start index of the section
    private final int length; // Length of the section
    private final int animSlot; // slot

    // Constructor for the enum to initialize start and length
    LEDSection(int start, int length, int animSlot) {
      this.start = start;
      this.length = length;
      this.animSlot = animSlot;
    }

    public int getStart() {
      return start; // Getter for start index
    }

    public int getLength() {
      return length; // Getter for length
    }

    public int getSlot() {
      return animSlot; // Getter for length
    }
  }

  public static class Color {
    public int red;
    public int green;
    public int blue;
    public int white;

    public Color(int red, int green, int blue) {
      this.red = red;
      this.green = green;
      this.blue = blue;
    }
  }
}
