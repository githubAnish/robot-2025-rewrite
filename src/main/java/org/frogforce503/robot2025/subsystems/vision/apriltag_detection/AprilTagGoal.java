package org.frogforce503.robot2025.subsystems.vision.apriltag_detection;

import java.util.Set;
import java.util.EnumSet;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Predicate;
import java.util.function.Supplier;

import org.frogforce503.lib.vision.VisionUtils;
import org.frogforce503.lib.vision.apriltag_detection.PoseObservation;
import org.frogforce503.lib.vision.apriltag_detection.PoseObservationType;
import org.frogforce503.lib.vision.apriltag_detection.TrackedAprilTag;
import org.frogforce503.robot2025.subsystems.vision.Vision.CameraName;
import org.frogforce503.robot2025.subsystems.vision.VisionConstants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import lombok.Getter;

/**
 * Enum representing different goals for robot localization with AprilTags.
 * Each goal specifies the cameras to use, a filter to check if a vision measurement is reliable, a configuration for a camera, a standard deviation calculator for fusing vision measurements, and an optional backup goal.
 */
public enum AprilTagGoal {
    /**
     * Uses every AprilTag camera available on the robot for localization.
     * Should not be used for precise tasks such as automated alignment.
     */
    GLOBAL_LOCALIZATION(
        () -> EnumSet.of(CameraName.FRONT_LEFT, CameraName.UPPER_FRONT_RIGHT, CameraName.LOWER_FRONT_RIGHT, CameraName.ELEVATOR_BACK, CameraName.ELEVATOR_FRONT),

        poseObservation -> {
            TrackedAprilTag[] tags = poseObservation.usedAprilTags();
            
            double ambiguity = VisionUtils.getLowestAmbiguity(tags);
            double distance = VisionUtils.getLowestDistanceToCamera(tags);

            double maxAmbiguity = tags.length > 1 ? 0.15 : 0.10;
            double maxDistance = tags.length > 1 ? Units.feetToMeters(15) : Units.feetToMeters(10);

            return ambiguity <= maxAmbiguity && distance <= maxDistance;
        },

        aprilTagIO -> {
            if (aprilTagIO instanceof AprilTagIOPhotonVision || aprilTagIO instanceof AprilTagIOPhotonSim) {
                aprilTagIO.setPoseObservationType(PoseObservationType.MULTI_TAG_PNP_ON_COPROCESSOR);
            } 
            aprilTagIO.setIgnoredAprilTags(Set.of(4, 5, 14, 15)); //Barge Tags
        },

        (poseObservation) -> VisionConstants.fixedStdDevs, // Standard deviations for the pose observation

        Optional.empty()
    ),

    /**
     * Uses available reef-facing cameras and trigonometric solving to localize the robot with reef AprilTags.
     * Biased at far distances; should only be used when aligning with the reef.
     * Backup goal for reef alignment is global localization.
     */
    REEF_ALIGNMENT(
        () -> EnumSet.of(CameraName.FRONT_LEFT, CameraName.UPPER_FRONT_RIGHT, CameraName.LOWER_FRONT_RIGHT),

        poseObservation -> {
            TrackedAprilTag[] tags = poseObservation.usedAprilTags();

            double distance = VisionUtils.getLowestDistanceToCamera(tags);

            double maxDistance = Units.feetToMeters(8);

            return distance <= maxDistance;
        },

        aprilTagIO -> {
            if (aprilTagIO instanceof AprilTagIOPhotonVision || aprilTagIO instanceof AprilTagIOPhotonSim) {
                aprilTagIO.setPoseObservationType(PoseObservationType.PNP_DISTANCE_TRIG_SOLVE);
            }
            aprilTagIO.setIgnoredAprilTags(Set.of(4, 5, 14, 15)); // Barge Tags
        },

        (poseObservation) -> VisionConstants.fixedStdDevs, // Standard deviations for the pose observation

        Optional.of(GLOBAL_LOCALIZATION)
    );
    
    @Getter private Supplier<EnumSet<CameraName>> camerasToUse;
    @Getter private Predicate<PoseObservation> cameraFilter;
    @Getter private Consumer<AprilTagIO> cameraConfiguration;
    @Getter private Function<PoseObservation, Matrix<N3, N1>> standardDeviationCalculator;
    @Getter private Optional<AprilTagGoal> backupGoal;
    
    /**
     * @param camerasToUse Supplier that provides the set of cameras to use for this goal
     * @param cameraFilter Predicate that takes the pose observation of a camera to determine if it should be used for localization
     * @param cameraConfiguration Consumer that configures the AprilTagIO for this goal
     * @param standardDeviationCalculator Function that provides the standard deviations for the pose observation
     * @param backupGoal Optional backup goal to use if this goal fails
     */
    private AprilTagGoal(
        Supplier<EnumSet<CameraName>> camerasToUse, 
        Predicate<PoseObservation> cameraFilter, 
        Consumer<AprilTagIO> cameraConfiguration, 
        Function<PoseObservation, Matrix<N3, N1>> standardDeviationCalculator,
        Optional<AprilTagGoal> backupGoal
    ) {
        this.camerasToUse = camerasToUse;
        this.cameraFilter = cameraFilter;
        this.cameraConfiguration = cameraConfiguration;
        this.standardDeviationCalculator = standardDeviationCalculator;
        this.backupGoal = backupGoal;
    }
}

