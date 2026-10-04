package frc.robot.constants;
import frc.robot.util.interpolation.InterpolatingDouble;
import frc.robot.util.interpolation.InterpolatingTreeMap;

public class InterpolatingConstants {

// Shooter
  public static final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>
      ShooterSpeedMap = new InterpolatingTreeMap<>();
  static {
    // distance, deegres
    ShooterSpeedMap.put(new InterpolatingDouble(2.0), new InterpolatingDouble(50.0));
    // ShooterSpeedMap.put(new)
  }
  }

    
