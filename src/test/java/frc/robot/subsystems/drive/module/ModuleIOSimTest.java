package frc.robot.subsystems.drive.module;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class ModuleIOSimTest {
    @BeforeAll
    static void initializeHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @Test
    void tracksRequestedWheelSpeedInBothDirections() {
        for (double speed : new double[] {0.35, -0.35, 2.1, -2.1}) {
            var io = new ModuleIOSim();
            var inputs = new ModuleIO.ModuleIOInputs();
            io.setDriveVelocity(speed);
            for (int i = 0; i < 250; i++) {
                io.updateInputs(inputs);
            }
            assertEquals(speed, inputs.driveVelocityMps, 0.05);
            assertTrue(inputs.drivePositionMeters * speed > 0.0);
        }
    }
}
