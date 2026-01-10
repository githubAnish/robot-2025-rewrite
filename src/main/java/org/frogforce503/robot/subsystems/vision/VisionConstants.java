package org.frogforce503.robot.subsystems.vision;

import java.util.Set;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    /**
     * Cameras on robots are configured with a name.
     * Every camera on the robot must have a name from this enum.
     * This enum is used to identify specific cameras on a robot for any use case.
     */
    public enum CameraName {
        FRONT_LEFT,
        UPPER_FRONT_RIGHT,
        LOWER_FRONT_RIGHT,
        ELEVATOR_BACK,
        ELEVATOR_FRONT
    }

    public static final Set<Integer> BARGE_TAGS = Set.of(4, 5, 14, 15);
    public static final Set<Integer> CORAL_STATION_TAGS = Set.of(1, 2, 12, 13);
    public static final Set<Integer> PROCESSOR_TAGS = Set.of(3, 16);
    public static final Set<Integer> REEF_TAGS = Set.of(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22);

    public static final Matrix<N3, N1> DEFAULT_STANDARD_DEVIATIONS = VecBuilder.fill(0.8, 0.8, Units.degreesToRadians(30));
}
