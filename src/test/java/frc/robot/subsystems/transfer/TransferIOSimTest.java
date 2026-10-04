package frc.robot.subsystems.transfer;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class TransferIOSimTest {
    @BeforeAll
    static void initializeHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @Test
    void updatesImmediatelyAfterConstructionWithoutMotorInput() {
        var io = new TransferIOSim();
        var inputs = new TransferIO.TransferIOInputs();

        io.updateInputs(inputs);

        assertEquals(0.0, inputs.velocityRPS, 1e-9);
        assertEquals(0.0, inputs.appliedVolts, 1e-9);
        assertEquals(0.0, inputs.currentAmps, 1e-9);
    }

    @Test
    void motorInputProducesMotionInBothDirections() {
        for (double dutyCycle : new double[] {0.5, -0.5}) {
            var io = new TransferIOSim();
            var inputs = new TransferIO.TransferIOInputs();
            io.DutyCycle(dutyCycle);

            for (int i = 0; i < 10; i++) {
                io.updateInputs(inputs);
            }

            assertEquals(dutyCycle * 12.0, inputs.appliedVolts, 1e-9);
            assertTrue(Double.isFinite(inputs.velocityRPS));
            assertTrue(inputs.velocityRPS * dutyCycle > 0.0);
            assertTrue(Double.isFinite(inputs.currentAmps));
        }
    }
}
