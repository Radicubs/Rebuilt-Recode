package frc.robot.constants;

import frc.robot.util.interpolation.InterpolatingDouble;
import frc.robot.util.interpolation.InterpolatingTreeMap;

/** Shooter tables: hub distance in meters to speed in RPS. Uses temporary test data. */
public final class InterpolatingConstants {
  private InterpolatingConstants() {}

  // Top shooter
  // public static final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>
  //     topShooterSpeedMap = new InterpolatingTreeMap<>();

  // static {
  //   // TODO: Replace these arbitrary test speeds with measured calibration data.
  //   // Distance (meters), speed (RPS)
  //   topShooterSpeedMap.put(new InterpolatingDouble(1.14), new InterpolatingDouble(.0));
  //   topShooterSpeedMap.put(new InterpolatingDouble(1.0), new InterpolatingDouble(27.0));
  //   topShooterSpeedMap.put(new InterpolatingDouble(1.5), new InterpolatingDouble(33.0));
  //   topShooterSpeedMap.put(new InterpolatingDouble(2.0), new InterpolatingDouble(41.0));
  //   topShooterSpeedMap.put(new InterpolatingDouble(3.0), new InterpolatingDouble(52.0));
  // }

  // Main shooter
  public static final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>
      mainShooterSpeedMap = new InterpolatingTreeMap<>();

  static {
    // TODO: Replace these arbitrary test speeds with measured calibration data.
    // Distance (meters), speed (RPS)
    mainShooterSpeedMap.put(new InterpolatingDouble(1.131), new InterpolatingDouble(27.0));
    mainShooterSpeedMap.put(new InterpolatingDouble(2.9), new InterpolatingDouble(37.0));
    mainShooterSpeedMap.put(new InterpolatingDouble(2.25), new InterpolatingDouble(35.0));
    mainShooterSpeedMap.put(new InterpolatingDouble(3.42), new InterpolatingDouble(42.5));
    mainShooterSpeedMap.put(new InterpolatingDouble(3.13), new InterpolatingDouble(41.0));
    mainShooterSpeedMap.put(new InterpolatingDouble(2.25), new InterpolatingDouble(45.0));
    mainShooterSpeedMap.put(new InterpolatingDouble(2.05), new InterpolatingDouble(35.0));
    mainShooterSpeedMap.put(new InterpolatingDouble(3.2), new InterpolatingDouble(40.0));
    mainShooterSpeedMap.put(new InterpolatingDouble(3.25), new InterpolatingDouble(41.0));
    mainShooterSpeedMap.put(new InterpolatingDouble(3.03), new InterpolatingDouble(39.5));
    mainShooterSpeedMap.put(new InterpolatingDouble(3.3), new InterpolatingDouble(42.5));
    mainShooterSpeedMap.put(new InterpolatingDouble(3.85), new InterpolatingDouble(44.5));
    mainShooterSpeedMap.put(new InterpolatingDouble(2.23), new InterpolatingDouble(35.0));
    mainShooterSpeedMap.put(new InterpolatingDouble(4.5), new InterpolatingDouble(48.0));
    mainShooterSpeedMap.put(new InterpolatingDouble(1.33), new InterpolatingDouble(31.0));
    mainShooterSpeedMap.put(new InterpolatingDouble(2.87), new InterpolatingDouble(39.0));
    mainShooterSpeedMap.put(new InterpolatingDouble(2.21), new InterpolatingDouble(38.0));
    // mainShooterSpeedMap.put(new InterpolatingDouble(2.25), new InterpolatingDouble(36.0));
    // mainShooterSpeedMap.put(new InterpolatingDouble(2.25), new InterpolatingDouble(36.0));
    // mainShooterSpeedMap.put(new InterpolatingDouble(2.25), new InterpolatingDouble(36.0));

  }
}
