package frc.robot;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.VisionFunctions;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class RobotContainerTest {
    @BeforeAll
    static void initializeHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @Test
    void startupInitializesVisionBeforePublishingShooterSpeeds() {
        assertDoesNotThrow(RobotContainer::new);
        assertDoesNotThrow(SmartDashboard::updateValues);
        assertTrue(Double.isFinite(VisionFunctions.getHubDistanceMeters()));
        assertTrue(Double.isFinite(Shooter.getInstance().getMainDesiredSpeed()));
        assertTrue(Double.isFinite(Shooter.getInstance().getTopDesiredSpeed()));
        assertTrue(SmartDashboard.containsKey("Top Shooter/Top Shooter Desired Speed"));
    }
}
