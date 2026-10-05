package frc.robot.constants;

import frc.robot.util.interpolation.InterpolatingDouble;
import frc.robot.util.interpolation.InterpolatingTreeMap;

/** Shooter tables: hub distance in meters to speed in RPS. Uses temporary test data. */
public final class InterpolatingConstants {
  private InterpolatingConstants() {}

  // Top shooter
  public static final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>
      topShooterSpeedMap = new InterpolatingTreeMap<>();

  static {
    // TODO: Replace these arbitrary test speeds with measured calibration data.
    // Distance (meters), speed (RPS)
    topShooterSpeedMap.put(new InterpolatingDouble(0.5), new InterpolatingDouble(18.0));
    topShooterSpeedMap.put(new InterpolatingDouble(1.0), new InterpolatingDouble(27.0));
    topShooterSpeedMap.put(new InterpolatingDouble(1.5), new InterpolatingDouble(33.0));
    topShooterSpeedMap.put(new InterpolatingDouble(2.0), new InterpolatingDouble(41.0));
    topShooterSpeedMap.put(new InterpolatingDouble(3.0), new InterpolatingDouble(52.0));
  }

  // Main shooter
  public static final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>
      mainShooterSpeedMap = new InterpolatingTreeMap<>();

  static {
    // TODO: Replace these arbitrary test speeds with measured calibration data.
    // Distance (meters), speed (RPS)
    mainShooterSpeedMap.put(new InterpolatingDouble(0.5), new InterpolatingDouble(32.0));
    mainShooterSpeedMap.put(new InterpolatingDouble(1.0), new InterpolatingDouble(44.0));
    mainShooterSpeedMap.put(new InterpolatingDouble(1.5), new InterpolatingDouble(57.0));
    mainShooterSpeedMap.put(new InterpolatingDouble(2.0), new InterpolatingDouble(69.0));
    mainShooterSpeedMap.put(new InterpolatingDouble(3.0), new InterpolatingDouble(86.0));
  }
}
