package org.frogforce503.robot2025.subsystems.vision.apriltag_detection;

import org.frogforce503.robot2025.subsystems.vision.Vision.CameraName;
import org.frogforce503.robot2025.subsystems.vision.VisionSimulator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * An implementation of the AprilTagIO interface.
 * 
 * It extends AprilTagIOPhotonVision to simulate AprilTag detection with a PhotonVision camera using PhotonSim.
 */
public class AprilTagIOPhotonSim extends AprilTagIOPhotonVision {
    private PhotonCameraSim cameraSim;
    private VisionSystemSim aprilTagDetectionSimulator;

    /**
     * @param cameraName The enum representing the name of the camera configured in PhotonVision
     * @param robotToCameraOffset The transform3d representing the offset from the robot's origin to the camera's origin
     * @param aprilTagFieldLayout The AprilTagFieldLayout to use for pose estimation
     * @param visionSimulator The VisionSimulator that contains the VisionSystemSim instance that the camera will be added – the simulation world for the camera.
     * @param cameraProperties The SimCameraProperties to configure the camera simulation
     */
    public AprilTagIOPhotonSim(
        CameraName cameraName, 
        Transform3d robotToCameraOffset, 
        AprilTagFieldLayout aprilTagFieldLayout,
        VisionSimulator visionSimulator, 
        SimCameraProperties cameraProperties
    ) {
        super(cameraName, robotToCameraOffset, aprilTagFieldLayout);

        cameraSim = new PhotonCameraSim(super.getCamera(), cameraProperties);   
        cameraSim.enableRawStream(true);
        cameraSim.enableProcessedStream(true);
        cameraSim.enableDrawWireframe(true);

        this.aprilTagDetectionSimulator = visionSimulator.getAprilTagDetectionSimulator();
        aprilTagDetectionSimulator.addCamera(cameraSim, robotToCameraOffset);
    }

    /**
     * @param cameraName The enum representing the name of the camera configured in PhotonVision
     * @param robotToCameraOffset The transform3d representing the offset from the robot's origin to the camera's origin
     * @param visionSimulator The VisionSimulator that contains the VisionSystemSim instance that the camera will be added – the simulation world for the camera.
     * @param cameraProperties The SimCameraProperties to configure the camera simulation
     */
    public AprilTagIOPhotonSim(
        CameraName cameraName, 
        Transform3d robotToCameraOffset,  
        VisionSimulator visionSimulator, 
        SimCameraProperties cameraProperties
    ) {
        this(
            cameraName, 
            robotToCameraOffset, 
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded), 
            visionSimulator,
            cameraProperties
        );
    }

    /**
     * @param cameraName The enum representing the name of the camera configured in PhotonVision
     * @param robotToCameraOffset The transform3d representing the offset from the robot's origin to the camera's origin
     * @param visionSimulator The VisionSimulator that contains the VisionSystemSim instance that the camera will be added – the simulation world for the camera.
     */
    public AprilTagIOPhotonSim(
        CameraName cameraName, 
        Transform3d robotToCameraOffset, 
        VisionSimulator visionSimulator
    ) {
        this(
            cameraName, 
            robotToCameraOffset, 
            visionSimulator, 
            new SimCameraProperties()
                .setCalibration(1280, 800, Rotation2d.fromDegrees(78.2))
                .setCalibError(0.25, 0.08)
                .setFPS(50)
                .setAvgLatencyMs(35)
                .setLatencyStdDevMs(5)
        );
    }

    @Override
    public void setRobotToCameraOffset(Transform3d robotToCameraOffset) {
        super.setRobotToCameraOffset(robotToCameraOffset);
        aprilTagDetectionSimulator.adjustCamera(cameraSim, robotToCameraOffset);
    }
}
