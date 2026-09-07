package frc.robot.util;

/**
 * Unit conversions between wheel rotations and linear distance.
 *
 * <p>These take mechanism-side (wheel) rotations, not rotor rotations. The drive TalonFX applies
 * {@code Feedback.SensorToMechanismRatio = driveGearRatio} in {@link frc.robot.constants.CTREConfigs},
 * and the sim {@code DCMotorSim} is built with the same gearing, so both call sites already report
 * wheel rotations. Circumference is in meters.
 */
public final class Conversions {

    private Conversions() {}

    public static double rotationsToMeters(double wheelRotations, double circumference) {
        return wheelRotations * circumference;
    }

    public static double metersToRotations(double wheelMeters, double circumference) {
        return wheelMeters / circumference;
    }

    public static double RPSToMPS(double wheelRPS, double circumference) {
        return wheelRPS * circumference;
    }

    public static double MPSToRPS(double wheelMPS, double circumference) {
        return wheelMPS / circumference;
    }
}
