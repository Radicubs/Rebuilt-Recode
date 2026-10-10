package frc.robot.constants;

import edu.wpi.first.math.util.Units;

/** Camera mount offsets from robot center, meters. TODO: measure camera_0 X/Y on the real robot before competition. */
public final class VisionConstants {
    /* AutoAlignTag / lock on values */
    public static final double ALIGNMENT_TOLERANCE_DEGREES = 2;
    public static final double lockKP = 0.7; //TODO: This must be tuned to specific robot
    public static final double lockDeadband = 0.05;
    public static final double lockOnMaxSpeed = 2;

    /** Distance from the hub tag inward to the aiming target, meters. */
    public static final double HUB_OFFSET_METERS = 0.60365;

    /** Maximum capture age for live target helpers and incoming pose measurements. */
    public static final double maxResultAgeSeconds = 0.5;
    public static final double maxSingleTagAmbiguity = 0.2;
    public static final double maxSingleTagDistanceMeters = 4.0;
    public static final double maxMultiTagDistanceMeters = 6.0;
    public static final double maxPoseHeightMeters = 0.4;
    public static final double maxPoseTiltRadians = Units.degreesToRadians(20.0);
    public static final double maxPoseInnovationMeters = 1.5;
    public static final double singleTagXYStdDevMeters = 0.5;
    public static final double multiTagXYStdDevMeters = 0.2;
    /** Heading confidence for validated camera poses; multi-tag receives more trust. */
    public static final double singleTagHeadingStdDevRadians = Units.degreesToRadians(30.0);
    public static final double multiTagHeadingStdDevRadians = Units.degreesToRadians(10.0);
    public static double camera_0_OffsetX = 0;
    public static double camera_0_OffsetY = 0;
    public static double camera_0_OffsetZ = Units.inchesToMeters(18.5);

    public static double camera_1_OffsetX = Units.inchesToMeters(-2.5);
    public static double camera_1_OffsetY = Units.inchesToMeters(13.8);
    public static double camera_1_OffsetZ = Units.inchesToMeters(18.5);
}
