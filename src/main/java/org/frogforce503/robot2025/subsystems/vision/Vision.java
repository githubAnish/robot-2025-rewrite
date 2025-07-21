package org.frogforce503.robot2025.subsystems.vision;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.frogforce503.robot2025.Robot;
import org.frogforce503.robot2025.fields.FieldInfo;
import org.frogforce503.robot2025.subsystems.vision.camera_types.AprilTagCamera;
import org.frogforce503.robot2025.subsystems.vision.camera_types.PhotonVisionCamera;
import org.frogforce503.robot2025.subsystems.vision.camera_types.PhotonVisionCamera.CameraType;
import org.littletonrobotics.junction.Logger;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private final FieldInfo field;
    private final Consumer<EstimatedRobotPose> visionEstimateConsumer;
    private final Supplier<Pose2d> globalPoseSupplier;

    //Instance Data
    public Camera[] cameras = Camera.values();

    private HashMap<Camera, PhotonVisionCamera> cameraMap = new HashMap<>();
    private VisionSystemSim visionSim;
    private List<AprilTagCamera> potentialCameras = new ArrayList<>();
    private List<AprilTagCamera> camerasUsed = new ArrayList<>();
    private List<PhotonTrackedTarget> tagsUsed = new ArrayList<>();
    
    //Constructor
    public Vision(FieldInfo field, Consumer<EstimatedRobotPose> visionEstimateConsumer, Supplier<Pose2d> globalPoseSupplier) {
        this.field = field;
        this.visionEstimateConsumer = visionEstimateConsumer;
        this.globalPoseSupplier = globalPoseSupplier;

        for (Camera cam : cameras) {
            if (cam.getCameraType() == CameraType.APRIL_TAG_DETECTION) {
                cameraMap.put(
                    cam,
                    new AprilTagCamera(field, globalPoseSupplier, cam.getCameraName(), cam.getCameraToRobot()));
            }
            /*else if(cam.getCameraType() == CameraType.OBJECT_DETECTION) {
                cameraMap.put(cam, new YoloCamera(cam.getCameraName(), cam.getCameraToRobot(), cam.getSimCameraProperties()));
            } //for object detection class*/
        }

        if (Robot.isSimulation()) {
            initSimulation();
        }
    }

    // Photon Sim
    public void initSimulation() {
        System.out.println("Initializing simulation...");
        visionSim = new VisionSystemSim("Main");

        AprilTagFieldLayout tagLayout = field.getConfiguration().fieldLayout;

        if (tagLayout != null) {
            visionSim.addAprilTags(tagLayout);
            System.out.println("AprilTags added to visionSim.");
        }

        for(PhotonVisionCamera photonVisionCamera : cameraMap.values()) {
            visionSim.addCamera(photonVisionCamera.getCameraSim(), photonVisionCamera.getRobotToCamera());
            photonVisionCamera.configSimCamProperties();
            System.out.println("Camera added to visionSim: " + photonVisionCamera.getCamera().getName());
        }
    }

    public void updateSimulation() {
        if (visionSim != null) {
            visionSim.update(globalPoseSupplier.get());
        }
    }

    //Method from SubsystemBase
    @Override
    public void periodic() {
        camerasUsed.clear();
        potentialCameras.clear();
        tagsUsed.clear();

        for (Camera cam : cameras) {
            PhotonVisionCamera camera = cameraMap.get(cam);
            camera.update();
            camera.logData();

            if (camera instanceof AprilTagCamera) {
                AprilTagCamera aprilTagCamera = (AprilTagCamera) camera;

                if (aprilTagCamera.isPoseReliable(0.2, 18)) {
                    potentialCameras.add(aprilTagCamera);
                }
            }
        }

        AprilTagCamera bestCamera =
            potentialCameras.isEmpty()
                ? null
                : potentialCameras.get(0);

        for (AprilTagCamera potentialCamera : potentialCameras) {
            if (potentialCamera.getAverageTagDistance() < bestCamera.getAverageTagDistance()) {
                bestCamera = potentialCamera;
            }

            if (potentialCamera.isPoseReliable(
                    potentialCamera.getNumberOfTagsDetected() > 1
                        ? 0.15
                        : 0.10,
                    potentialCamera.getNumberOfTagsDetected() > 1
                        ? 15
                        : 10)
            ) {
                camerasUsed.add(potentialCamera);
            }
        }

        if (camerasUsed.isEmpty() && !potentialCameras.isEmpty()) {
            camerasUsed.add(bestCamera);
        }

        if (!camerasUsed.isEmpty()) {
            for (AprilTagCamera camera : camerasUsed) {
                visionEstimateConsumer.accept(camera.getEstimatedPose());
                camera.getEstimatedPose().targetsUsed.forEach(tag -> tagsUsed.add(tag));
            }
            
            tagsUsed = new ArrayList<>(tagsUsed.stream().distinct().toList()); //Gets rid of duplicate tags and sorts them

            Logger.recordOutput("Vision/CamerasUsed",
                camerasUsed
                    .stream()
                    .map(camera -> camera.getCameraName())
                    .toArray(String[]::new));

            Logger.recordOutput("Vision/TagsUsed",
                tagsUsed
                    .stream()
                    .mapToInt(tag -> tag.getFiducialId())
                    .toArray());
        } else {
            Logger.recordOutput("Vision/CamerasUsed", new String[0]);
            Logger.recordOutput("Vision/TagsUsed", new int[0]);
        }

        if (Robot.isSimulation()) {
            updateSimulation();
        }

    }

    public boolean doesCameraSeeTag(Camera cam, int tagID) {
        AprilTagCamera camera = (AprilTagCamera) cameraMap.get(cam);
        EstimatedRobotPose pose = camera.getEstimatedPose();

        if (pose != null) {
            for (PhotonTrackedTarget target : pose.targetsUsed) {
                if (target.getFiducialId() == tagID && target.getPoseAmbiguity() < 0.1) {
                    return true;
                }
            }
        }

        return false;
    }

    public boolean doesCameraSeeAnyTags(Camera cam) {
        AprilTagCamera camera = (AprilTagCamera) cameraMap.get(cam);
        return camera.getEstimatedPose() != null;
    }

    public Transform3d getTagToCamera(Camera cam, int tagID) {
        AprilTagCamera camera = (AprilTagCamera) cameraMap.get(cam);
        EstimatedRobotPose pose = camera.getEstimatedPose();

        if (pose != null) {
            for (PhotonTrackedTarget target : pose.targetsUsed) {
                if (target.getFiducialId() == tagID) {
                    return target.getBestCameraToTarget();
                }
            }
        }

        return new Transform3d();
    }

    public int maxTagsFromOneCam() {
        return
            cameraMap
                .values()
                .stream()
                .filter(camera -> camera instanceof AprilTagCamera)
                .mapToInt(camera -> ((AprilTagCamera) camera).getNumberOfTagsDetected())
                .max()
                .orElse(0);
    }

    public boolean areTagsVisible() {
        return maxTagsFromOneCam() > 0;
    }

    public boolean getCameraStatus(Camera cam) {
        return
            cameraMap
                .get(cam)
                .getStatus();
    }

    public int getClosestTagID(Camera cam) {
        int closestTagID = -1;

        AprilTagCamera aprilTagCamera = (AprilTagCamera) cameraMap.get(cam);

        PhotonTrackedTarget closestTag = aprilTagCamera.getClosestTag();
        if (closestTag != null) {
            closestTagID = closestTag.getFiducialId();
        }

        return closestTagID;
    }

    public int getNumberOfTags(Camera cam) {
        AprilTagCamera aprilTagCamera = (AprilTagCamera) cameraMap.get(cam);
        return aprilTagCamera.getNumberOfTagsDetected();
    }

    public int[] getArrayOfTagsDetected(Camera cam) {
        AprilTagCamera aprilTagCamera = (AprilTagCamera) cameraMap.get(cam);
        return aprilTagCamera.getArrayOfTagIDs();
    }

    public Supplier<Pose2d> getSpecializedPose(Camera cam) {
        // Returns global pose if the specialized pose is not reliable
        AprilTagCamera aprilTagCamera = (AprilTagCamera) cameraMap.get(cam);

        return
            () ->
                aprilTagCamera.isSpecializedPoseReliable(4)
                    ? aprilTagCamera
                        .getSpecializedPose()
                        .estimatedPose
                        .toPose2d()
                    : globalPoseSupplier
                        .get();
    }

    public Supplier<Pose2d> getReefSpecializedPose(Supplier<Command> signalGlobalPoseUsed) {
        AprilTagCamera frontRightCamera = (AprilTagCamera) cameraMap.get(Camera.FRONT_RIGHT);
        AprilTagCamera frontLeftCamera = (AprilTagCamera) cameraMap.get(Camera.FRONT_LEFT);
        AprilTagCamera lowerfrontRightCamera = (AprilTagCamera) cameraMap.get(Camera.LOWER_FRONT_RIGHT);

        return
            () -> {
                AprilTagCamera cameraUsed = frontRightCamera;

                if (frontRightCamera.isSpecializedPoseReliable(8)) {
                    cameraUsed = frontRightCamera;
                } else if (lowerfrontRightCamera.isSpecializedPoseReliable(8)) {
                    cameraUsed = lowerfrontRightCamera;
                } else if (frontLeftCamera.isSpecializedPoseReliable(8)) {
                    cameraUsed = frontLeftCamera;
                } else {
                    Logger.recordOutput("Vision/Auto-align Pose", "Global");
                    signalGlobalPoseUsed
                        .get()
                        .schedule();
                    return
                        globalPoseSupplier.get();
                }
                
                Logger.recordOutput("Vision/Auto-align Pose", cameraUsed.getCameraName());
                return cameraUsed.getSpecializedPose().estimatedPose.toPose2d();
            };
    }

    public Pose2d getCalculatedTagPose(Camera cam, int tagID) {
        AprilTagCamera camera = (AprilTagCamera) cameraMap.get(cam);
        return camera.getCalculatedTagPose(tagID);
    }
}