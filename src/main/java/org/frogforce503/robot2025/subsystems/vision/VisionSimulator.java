package org.frogforce503.robot2025.subsystems.vision;

import org.frogforce503.robot2025.Constants;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import lombok.Getter;

/**
 * Wrapper class for multiple PhotonVision VisionSystemSims. 
 * Manages simulation worlds for object detection and AprilTag detection cameras.
 */
public class VisionSimulator {
    //AprilTag Detection
    private VisionSystemSim aprilTagDetectionSimulator;
    @Getter private AprilTagFieldLayout aprilTagFieldLayout;

    //Object Detection
    private VisionSystemSim objectDetectionSimulator;

    // Coral represented as a rectangular prism with dimensions 11 cm x 11 cm x 30 cm (w x l x h)
    private final TargetModel coralModel = new TargetModel(0.11, 0.11, 0.3);
    // Algae represented as a sphere with a diameter of 41.3 cm
    private final TargetModel algaeModel = new TargetModel(0.413);

    /**
     * @param aprilTagFieldLayout The AprilTagFieldLayout to use for AprilTag detection simulation.
     */
    public VisionSimulator(AprilTagFieldLayout aprilTagFieldLayout) {
        this.aprilTagFieldLayout = aprilTagFieldLayout;
    }

    public VisionSimulator() {
        this(Constants.fieldVenue.getAprilTagFieldLayout());
    }

    /**
     * Updates the simulation state with the robot's current pose.
     * This method should be called periodically to ensure the simulation reflects the robot's position.
     * 
     * @param robotPose The current pose of the robot in the simulation.
     */
    public void update(Pose2d robotPose) {
        if (aprilTagDetectionSimulator != null) {
            aprilTagDetectionSimulator.update(robotPose);
        }

        if (objectDetectionSimulator != null) {
            objectDetectionSimulator.update(robotPose);
        }
    }

    /************************************ APRILTAG DETECTION LOGIC ************************************/

    /**
     * Gets the VisionSystemSim that simulates the robot's AprilTag detection system.
     * Utilizes lazy initialization to create the simulator only when it is first requested.
     * 
     * @return The VisionSystemSim instance used for AprilTag detection.
     */
    public VisionSystemSim getAprilTagDetectionSimulator() {
        if (aprilTagDetectionSimulator == null) {
            aprilTagDetectionSimulator = new VisionSystemSim("AprilTagSimulator");

            aprilTagDetectionSimulator.addAprilTags(aprilTagFieldLayout);
        }

        return aprilTagDetectionSimulator;
    }

    /************************************ OBJECT DETECTION LOGIC ************************************/

    /**
     * Gets the VisionSystemSim that simulates the robot's object detection system.
     * Utilizes lazy initialization to create the simulator only when it is first requested.
     * 
     * @return The VisionSystemSim instance used for object detection.
     */
    public VisionSystemSim getObjectDetectionSimulator() {
        if (objectDetectionSimulator == null) {
            objectDetectionSimulator = new VisionSystemSim("ObjectDetectionSimulator");
        }

        return objectDetectionSimulator;
    }

    /**
     * Adds coral to the object detection simulator at the specified poses.
     * 
     * @param coralPoses The 3d poses where coral should be added in the simulation.
     */
    public void addCoral(Pose3d... coralPoses) {
        if (objectDetectionSimulator != null) {
            VisionTargetSim[] coral = new VisionTargetSim[coralPoses.length];

            for (int i = 0; i < coralPoses.length; i++) {
                coral[i] = new VisionTargetSim(coralPoses[i], coralModel, 0); // Coral has a class Id of 0 in our coral/algae detection model
            }

            objectDetectionSimulator.addVisionTargets("Coral", coral);
        }
    }

    /**
     * Adds algae to the object detection simulator at the specified poses.
     * 
     * @param algaePoses The 3d poses where algae should be added in the simulation.
     */
    public void addAlgae(Pose3d... algaePoses) {
        if (objectDetectionSimulator != null) {
            VisionTargetSim[] algae = new VisionTargetSim[algaePoses.length];

            for (int i = 0; i < algaePoses.length; i++) {
                algae[i] = new VisionTargetSim(algaePoses[i], algaeModel, 1); // Algae has a class Id of 1 in our coral/algae detection model
            }

            objectDetectionSimulator.addVisionTargets("Algae", algae);
        }
    }

    /**
     * Clears all objects from the object detection simulator.
     * This method is useful for resetting the simulation environment.
     */
    public void clearObjects() {
        if (objectDetectionSimulator != null) {
            objectDetectionSimulator.clearVisionTargets();
        }
    }
}
