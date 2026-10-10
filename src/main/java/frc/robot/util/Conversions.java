package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public class Conversions {

    public static double RPSToMPS(double wheelRPS, double circumference) {
        return wheelRPS * circumference;
    }

    public static double MPSToRPS(double wheelMPS, double circumference) {
        return wheelMPS / circumference;
    }

    public static double rotationsToMeters(double wheelRotations, double circumference) {
        return wheelRotations * circumference;
    }

    public static double metersToRotations(double wheelMeters, double circumference) {
        return wheelMeters / circumference;
    }

    public static double[] poseToArray(Pose2d pose) {
        if (pose != null)
            return new double[] {pose.getX(), pose.getY(), pose.getRotation().getDegrees()};
        return null;
    }

    public static double[] poseToArray(Pose3d pose) {
        if (pose != null)
            return new double[] {pose.getX(), pose.getY(), pose.getZ(), pose.getRotation().toRotation2d().getDegrees()};
        return null;
    }
    
}
