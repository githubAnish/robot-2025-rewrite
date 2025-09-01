package org.frogforce503.robot2025.subsystems.vision;

import java.util.EnumMap;
import java.util.EnumSet;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.frogforce503.robot2025.fields.FieldInfo;
import org.frogforce503.robot2025.subsystems.vision.apriltag_detection.AprilTagGoal;
import org.frogforce503.robot2025.subsystems.vision.apriltag_detection.AprilTagIO;
import org.frogforce503.robot2025.subsystems.vision.apriltag_detection.AprilTagInputsAutoLogged;
import org.frogforce503.robot2025.subsystems.vision.object_detection.ObjectDetectionIO;
import org.frogforce503.robot2025.subsystems.vision.object_detection.ObjectDetectionInputsAutoLogged;
import org.frogforce503.lib.vision.apriltag_detection.PoseObservation;
import org.frogforce503.lib.vision.apriltag_detection.VisionMeasurement;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

/**
 * The subsystem that handles vision processing for AprilTag detection and object detection.
 */
public class Vision extends SubsystemBase {
    private final FieldInfo field;
    private final Consumer<VisionMeasurement> visionConsumer;
    private final Supplier<Pose2d> robotPoseSupplier;

    // Maps camera names to their corresponding AprilTagIO instances.
    private EnumMap<CameraName, AprilTagIO> aprilTagIOMap = new EnumMap<>(CameraName.class);
    // Maps camera names to their corresponding AprilTagInputs instances.
    private EnumMap<CameraName, AprilTagInputsAutoLogged> aprilTagInputsMap = new EnumMap<>(CameraName.class);
    
    // The current goal for AprilTag detection, defaulting to global localization.
    private AprilTagGoal desiredAprilTagGoal = AprilTagGoal.GLOBAL_LOCALIZATION;


    // Maps camera names to their corresponding ObjectDetectionIO instances.
    private EnumMap<CameraName, ObjectDetectionIO> objectDetectionIOMap = new EnumMap<>(CameraName.class);
    // Maps camera names to their corresponding ObjectDetectionInputs instances.
    private EnumMap<CameraName, ObjectDetectionInputsAutoLogged> objectDetectionInputsMap = new EnumMap<>(CameraName.class);

    // Alerts
    private final Alert cameraDisconnect = new Alert("Camera " + "______" + " from inputs disconnected", AlertType.kError);
    private final Alert noTagsDetected = new Alert("No tags detected from any camera", AlertType.kWarning);

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
    
    /**
     * @param visionConsumer The consumer that will fuse vision measurements into the robot pose
     * @param robotPoseSupplier The supplier that provides the robot's pose
     * @param aprilTagIOs Array of AprilTagIO for AprilTag detection
     * @param objectDetectionIOs Array of ObjectDetectionIO for object detection
     */
    public Vision(
        FieldInfo field,
        Consumer<VisionMeasurement> visionConsumer, 
        Supplier<Pose2d> robotPoseSupplier, 
        AprilTagIO[] aprilTagIOs, 
        ObjectDetectionIO[] objectDetectionIOs
    ) {
        this.field = field;
        this.visionConsumer = visionConsumer;
        this.robotPoseSupplier = robotPoseSupplier;

        //Populate maps
        for (int i = 0; i < aprilTagIOs.length; i++) {
            aprilTagIOMap.put(aprilTagIOs[i].getCameraName(), aprilTagIOs[i]);
            aprilTagInputsMap.put(aprilTagIOs[i].getCameraName(), new AprilTagInputsAutoLogged());

            // Warm up resource-intensive logging
            Logger.recordOutput(
                "Vision/AprilTag Detection/" + aprilTagIOs[i].getCameraName().name() + "/Pose Observation",
                new PoseObservation()
            );
        }

        for (int i = 0; i < objectDetectionIOs.length; i++) {
            objectDetectionIOMap.put(objectDetectionIOs[i].getCameraName(), objectDetectionIOs[i]);
            objectDetectionInputsMap.put(objectDetectionIOs[i].getCameraName(), new ObjectDetectionInputsAutoLogged());
        }
    }

    @Override
    public void periodic() {
        /************************************ APRILTAG DETECTION LOGIC ************************************/
        EnumSet<AprilTagGoal> aprilTagGoalsRan = EnumSet.noneOf(AprilTagGoal.class); // Set to track which AprilTag goals have been run this periodic cycle.
        boolean anyAprilTagCamerasUsed = false; // Boolean to track if any AprilTag cameras were used for the current goal.

        AprilTagGoal currentAprilTagGoal = desiredAprilTagGoal; // The current AprilTag goal being processed.

        for (CameraName cameraName : aprilTagIOMap.keySet()) {
            AprilTagIO aprilTagIO = aprilTagIOMap.get(cameraName);
            AprilTagInputsAutoLogged aprilTagInputs = aprilTagInputsMap.get(cameraName);

            // Update AprilTag IO with the important information for accurate pose estimations.
            aprilTagIO.setRobotPose(robotPoseSupplier.get());

            // Update the inputs for the AprilTagIO and log them.
            aprilTagIO.updateInputs(aprilTagInputs);
            Logger.processInputs("Vision/AprilTag Detection/" + cameraName.name() + "/Inputs", aprilTagInputs);

            // Boolean representing if a VisionMeasurement from a camera's PoseObservation is accepted by the vision consumer.
            boolean visionMeasurementUsed = false;

            // Check if the camera is a potential camera to use for our current goal.
            if (currentAprilTagGoal.getCamerasToUse().get().contains(cameraName)) {
                Optional<VisionMeasurement> measurement = getVisionMeasurement(desiredAprilTagGoal, aprilTagIO);
                
                // If there is a vision measurement, it means the camera is outputting a pose observation reliable enough for the AprilTag goal.
                if (measurement.isPresent()) {
                    anyAprilTagCamerasUsed = true; // Set the boolean to true since we used a vision measurement.

                    visionConsumer.accept(measurement.get()); // Pass the measurement to the vision consumer so it can be fused with odometry.
                    visionMeasurementUsed = true; // Set the boolean to true since we used a vision measurement.
                }
            } else {
                // If the camera is not being used for the current goal, log default value for the Pose Observation
                Logger.recordOutput(
                    "Vision/AprilTag Detection/" + cameraName.name() + "/Pose Observation", 
                    new PoseObservation()
                );
            }

            Logger.recordOutput(
                "Vision/AprilTag Detection/" + cameraName.name() + "/Is Vision Measurement Used", 
                visionMeasurementUsed
            );

            aprilTagGoalsRan.add(currentAprilTagGoal); // Add the current goal to the set of goals that have been run.
        }

        /*
        If no vision measurements were accepted, the current goal has a backup goal, and the backup goal was never run before, try the backup goal.
        Loop in case backup goals have backup goals – effort to ensure we are always using vision measurements.
        Checks if the backup goal has been run before to avoid infinite loops in case of a cycle in the backup goals.
        */
        while (anyAprilTagCamerasUsed && currentAprilTagGoal.getBackupGoal().isPresent() && !aprilTagGoalsRan.contains(currentAprilTagGoal.getBackupGoal().get())) {
            currentAprilTagGoal = currentAprilTagGoal.getBackupGoal().get();

            for (CameraName cameraName : currentAprilTagGoal.getCamerasToUse().get()) {
                AprilTagIO aprilTagIO = aprilTagIOMap.get(cameraName);
                AprilTagInputsAutoLogged aprilTagInputs = aprilTagInputsMap.get(cameraName);

                if (aprilTagIO == null || aprilTagInputs == null) {
                    continue; // Skip if the camera is not an IO on the current robot, hashmaps will return null value if key isn't in the map.
                }
    
                Optional<VisionMeasurement> measurement = getVisionMeasurement(currentAprilTagGoal, aprilTagIO);
                
                // If there is a vision measurement, it means the camera is outputting a pose observation reliable enough for the AprilTag goal.
                if (measurement.isPresent()) {
                    anyAprilTagCamerasUsed = true; // Set the boolean to true since we used a vision measurement.

                    visionConsumer.accept(measurement.get()); // Pass the measurement to the vision consumer so it can be fused with odometry.
                    
                    Logger.recordOutput("Vision/AprilTag Detection/" + cameraName.name() + "/Is Vision Measurement Used", true);
                }
            }

            aprilTagGoalsRan.add(currentAprilTagGoal); // Add the current goal to the set of goals that have been run.
        }

        // Log desired goal and current goal
        Logger.recordOutput("Vision/AprilTag Detection/DesiredGoal", desiredAprilTagGoal);
        Logger.recordOutput("Vision/AprilTag Detection/CurrentGoal", desiredAprilTagGoal);

        
        /************************************ OBJECT DETECTION LOGIC ************************************/
        for (CameraName cameraName : objectDetectionIOMap.keySet()) {
            ObjectDetectionIO objectDetectionIO = objectDetectionIOMap.get(cameraName);
            ObjectDetectionInputsAutoLogged objectDetectionInputs = objectDetectionInputsMap.get(cameraName);

            objectDetectionIO.updateInputs(objectDetectionInputs);
            Logger.processInputs("Vision/Object Detection/" + cameraName.name() + "/Inputs", objectDetectionInputs);
        }
    }

    /** Cameras connected only when all cameras are connected & any tags are visible */
    public boolean checkConnections() {
        if (aprilTagInputsMap != null) {
            boolean anyHasTargets = false;

            for (var inputEntry : aprilTagInputsMap.entrySet()) {
                var inputs = inputEntry.getValue();

		        if (!inputs.connected) {
                    cameraDisconnect.setText("Camera " + inputEntry.getKey() + " from inputs disconnected");
                    cameraDisconnect.set(true);
		            return false;
		        }

                anyHasTargets |= inputs.hasTargets;
            }

            if (!anyHasTargets) {
		        noTagsDetected.set(true);
		        return false;
	        }
        }

        if (objectDetectionInputsMap != null) {
            boolean anyHasTargets = false;

            for (var inputEntry : objectDetectionInputsMap.entrySet()) {
                var inputs = inputEntry.getValue();

		        if (!inputs.connected) {
                    cameraDisconnect.setText("Camera " + inputEntry.getKey() + " from inputs disconnected");
                    cameraDisconnect.set(true);
		            return false;
		        }

                anyHasTargets |= inputs.hasTargets;
            }

            if (!anyHasTargets) {
		        noTagsDetected.set(true);
		        return false;
	        }
        }

        return true;
    }

    /**
     * Gets a VisionMeasurement from the AprilTagIO based on the current goal.
     * The camera is configured based on the goal's camera configuration, and the outputted pose observation is checked against the goal's camera filter.
     * If the camera has a vision measurement that meets the goal's criteria, a VisionMeasurement is created and returned.
     * 
     * @param goal the AprilTag goal being used to determine the camera configuration and filtering criteria
     * @param aprilTagIO the AprilTagIO instance for the camera being used
     * @return an Optional<VisionMeasurement> which is empty if no valid measurement is found, or contains a VisionMeasurement if a valid one is found.
     */
    private Optional<VisionMeasurement> getVisionMeasurement(AprilTagGoal goal, AprilTagIO aprilTagIO) {
        Optional<VisionMeasurement> measurement = Optional.empty();

        goal.getCameraConfiguration().accept(aprilTagIO); // Configure pose estimation parameters of the AprilTagIO for the AprilTagGoal
        PoseObservation poseObservation = aprilTagIO.estimateRobotPose(); // Estimate the robot's pose using the AprilTagIO.
        Logger.recordOutput("Vision/AprilTag Detection/" + aprilTagIO.getCameraName().name() + "/Pose Observation", poseObservation);
        field.getObject(aprilTagIO.getCameraName().name() + " Camera").setPose(poseObservation.robotPose().toPose2d());

        if (poseObservation.isReal()) { // Check if the pose observation is actually real
            // Check if the camera should be used for localization.
            if (goal.getCameraFilter().test(poseObservation)) {
                Matrix<N3, N1> standardDeviations = goal.getStandardDeviationCalculator().apply(poseObservation);

                // Create a VisionMeasurement and pass it to the vision consumer.
                measurement = Optional.of(
                    new VisionMeasurement(
                        poseObservation.timestamp(),
                        poseObservation.robotPose().toPose2d(),
                        standardDeviations
                    )
                );
            }
        }

        return measurement;
    }

    /**
     * Sets the current AprilTag goal for the Vision subsystem for global localization.
     */
    public void globalLocalization() {
        desiredAprilTagGoal = AprilTagGoal.GLOBAL_LOCALIZATION;
    }

    /**
     * Sets the current AprilTag goal for the Vision subsystem for alignment with the reef.
     */
    public void reefAlignment() {
        desiredAprilTagGoal = AprilTagGoal.REEF_ALIGNMENT;
    }
}
