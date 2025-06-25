package frc.robot.util;

public class Helpers {

  public static boolean withinTolerance(double value, double target, double tolerance) {
    return Math.abs(value - target) <= tolerance;
  }
}
