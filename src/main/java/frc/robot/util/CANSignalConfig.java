package frc.robot.util;

import com.revrobotics.spark.config.SignalsConfig;

/** Low-traffic SPARK telemetry with fast feedback only where needed. */
public final class CANSignalConfig {
    private CANSignalConfig() {}

    public static void configureSpark(SignalsConfig signals, boolean pivot) {
        int outputPeriodMs = pivot ? 20 : 200;
        signals.appliedOutputPeriodMs(outputPeriodMs)
                .outputCurrentPeriodMs(outputPeriodMs)
                .busVoltagePeriodMs(200)
                .motorTemperaturePeriodMs(1000)
                .primaryEncoderPositionPeriodMs(pivot ? 20 : 1000)
                .primaryEncoderPositionAlwaysOn(pivot)
                .primaryEncoderVelocityPeriodMs(pivot ? 1000 : 20)
                .primaryEncoderVelocityAlwaysOn(!pivot)
                .faultsPeriodMs(500)
                .warningsPeriodMs(500)
                .limitsPeriodMs(1000)
                .analogVoltagePeriodMs(1000)
                .analogVelocityPeriodMs(1000)
                .analogPositionPeriodMs(1000)
                .absoluteEncoderPositionPeriodMs(1000)
                .absoluteEncoderVelocityPeriodMs(1000)
                .iAccumulationPeriodMs(1000)
                .setpointPeriodMs(1000)
                .isAtSetpointPeriodMs(1000)
                .selectedSlotPeriodMs(1000)
                .maxMotionSetpointPositionPeriodMs(1000)
                .maxMotionSetpointVelocityPeriodMs(1000);
    }
}
