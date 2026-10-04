package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Arrays;
import org.junit.jupiter.api.Test;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

class VisionTest {
    private static PhotonPipelineResult frame(double timestampSeconds, int... tagIds) {
        var targets = Arrays.stream(tagIds).mapToObj(id -> {
            var target = new PhotonTrackedTarget();
            target.fiducialId = id;
            return target;
        }).toList();
        long timestampMicros = (long) (timestampSeconds * 1e6);
        return new PhotonPipelineResult(1, timestampMicros, timestampMicros, 0, targets);
    }

    @Test
    void prefersOrangeWhenBothCamerasSeeTags() {
        assertEquals(10, Vision.selectTagId(frame(9.9, 10), true, frame(9.95, 26), true, 10.0));
    }

    @Test
    void usesJuiceWhenOrangeHasNoTagsOrHasNotSentAFrame() {
        assertEquals(26, Vision.selectTagId(frame(9.9), true, frame(9.9, 26), true, 10.0));
        assertEquals(26, Vision.selectTagId(null, true, frame(9.9, 26), true, 10.0));
    }

    @Test
    void cachedTargetsExpireEvenWithoutANewFrame() {
        var cachedFrame = frame(9.9, 10);
        assertEquals(10, Vision.selectTagId(cachedFrame, true, null, false, 10.0));
        assertEquals(-1, Vision.selectTagId(cachedFrame, true, null, false, 10.5));
        assertEquals(26, Vision.selectTagId(cachedFrame, true, frame(10.4, 26), true, 10.5));
    }

    @Test
    void ignoresDisconnectedCamerasAndFutureTimestamps() {
        assertEquals(26, Vision.selectTagId(frame(9.9, 10), false, frame(9.9, 26), true, 10.0));
        assertEquals(-1, Vision.selectTagId(frame(10.1, 10), true, null, false, 10.0));
        assertEquals(-1, Vision.selectTagId(frame(9.9, 10), false, frame(9.9, 26), false, 10.0));
    }

    @Test
    void ignoresUnknownTagsAndPreservesPipelineOrderForKnownTags() {
        assertEquals(10, Vision.selectTagId(frame(9.9, -1, 999, 10, 26), true, null, false, 10.0));
        assertEquals(26, Vision.selectTagId(frame(9.9, 999), true, frame(9.9, 26), true, 10.0));
        assertEquals(-1, Vision.selectTagId(frame(9.9, 999), true, null, false, 10.0));
    }
}
