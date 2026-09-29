package frc.robot.constants;
import frc.robot.util.interpolation.InterpolatingDouble;
import frc.robot.util.interpolation.InterpolatingTreeMap;

public class InterpolatingConstants {

// Shooter
  public static final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>
      ShooterSpeedMap = new InterpolatingTreeMap<>();
  static {
    // distance, deegres
    ShooterSpeedMap.put(new InterpolatingDouble(2.0), new InterpolatingDouble(8.0));
    // ShooterSpeedMap.put(new)
  }

    // public double getDesiredSpeed() {
  //   // Use the interpolation table to convert measured distance into a shooter setpoint.
  //   InterpolatingDouble interpolatedDegrees =
  //       constants.interpolation.ShooterSpeedMap.getInterpolated(
  //           new InterpolatingDouble(LimelightFunctions.getHubDistanceMeters()));
  //   return interpolatedDegrees.value;
  // }

    
}
