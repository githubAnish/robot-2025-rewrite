package org.frogforce503.robot2025.subsystems.vision.oldvisionbackups;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import org.littletonrobotics.junction.Logger;
import org.frogforce503.robot2025.Robot;
import org.frogforce503.robot2025.fields.FieldInfo;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTagDetector extends SubsystemBase {
    private final FieldInfo field;
    private final Consumer<List<EstimatedRobotPose>> visionEstimateConsumer;
    private final Supplier<Pose2d> globalPoseSupplier;

    //Instance Data
    public Camera[] cameras = Camera.values(); //list of cameras on robot, used to iterate through hashmaps below

    private HashMap<Camera, PhotonCamera> photonCameras = new HashMap<>();
    private HashMap<Camera, EstimatedRobotPose> estimatedPoses = new HashMap<>();
    private HashMap<Camera, PhotonPoseEstimator> poseEstimators = new HashMap<>();
    private HashMap<Camera, Integer> numTagsDetected = new HashMap<>();

    private List<EstimatedRobotPose> posesUsed = new ArrayList<>(); //Cameras with their estimated robot poses fused
    private List<EstimatedRobotPose> potentialPoses = new ArrayList<>();
    private List<PhotonTrackedTarget> tagsUsed = new ArrayList<>();

    //Constructor
    public AprilTagDetector(FieldInfo field, Consumer<List<EstimatedRobotPose>> visionEstimateConsumer, Supplier<Pose2d> globalPoseSupplier) {
        this.field = field;
        this.visionEstimateConsumer = visionEstimateConsumer;
        this.globalPoseSupplier = globalPoseSupplier;

        var aprilTagLayoutField = AprilTagFields.kDefaultField.loadAprilTagLayoutField();

        for (Camera cam : cameras) {
            photonCameras.put(cam, new PhotonCamera(cam.getCameraName()));
            
            PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(aprilTagLayoutField, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cam.getCameraToRobot());
            poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
            poseEstimators.put(cam, poseEstimator);

            estimatedPoses.put(cam, null);

            numTagsDetected.put(cam, 0);
        }

    }
    //Enum to represent the different April Tag cameras on the robot
    public enum Camera {
        FRONT_LEFT("FrontLeftCamera", Robot.bot.FRONT_LEFT_CAMERA_TO_CENTER), 
        FRONT_RIGHT("FrontRightCamera", Robot.bot.FRONT_RIGHT_CAMERA_TO_CENTER), 
        ELEVATOR_BACK("ElevatorBackCamera", Robot.bot.ELEVATOR_BACK_CAMERA_TO_CENTER), 
        ELEVATOR_FRONT("ElevatorFrontCamera", Robot.bot.LOWER_FRONT_RIGHT_CAMERA_TO_CENTER);

        private String cameraName;
        private Transform3d cameraToRobot;

        Camera(String cameraName, Transform3d cameraToRobot) {
            this.cameraName = cameraName;
            this.cameraToRobot = cameraToRobot;
        }

        public Transform3d getCameraToRobot() {
            return cameraToRobot;
        }

        public String getCameraName() {
            return cameraName;
        }
    }

    /*--------------------------------------------- Overridden Methods (from FFSubsystemBase and SubsystemBase) ---------------------------------------------*/
    @Override
    public void periodic() {
        posesUsed.clear();
        potentialPoses.clear();
        tagsUsed.clear();

        for(Camera cam : cameras){
            updatePose(cam);
            logCameraData(cam);
            if (isPoseReliable(estimatedPoses.get(cam), 0.2, 18)) potentialPoses.add(estimatedPoses.get(cam));
        }


        EstimatedRobotPose bestPose = potentialPoses.isEmpty() ? null : potentialPoses.get(0);
        
        for(EstimatedRobotPose potentialPose : potentialPoses) {
            if (getAverageTagDistance(potentialPose) < getAverageTagDistance(bestPose)) {
                bestPose = potentialPose;
            }

            if (isPoseReliable(potentialPose, potentialPose.targetsUsed.size() > 1 ? 0.15 : 0.10, potentialPose.targetsUsed.size() > 1 ? 15 : 10)) posesUsed.add(potentialPose);
        }

        if(posesUsed.isEmpty() && !potentialPoses.isEmpty()) {
            posesUsed.add(bestPose);
        }

        if(!posesUsed.isEmpty()) {
            visionEstimateConsumer.accept(posesUsed);

            posesUsed.forEach(pose -> pose.targetsUsed.forEach(tag -> tagsUsed.add(tag))); //add all of the tags detected to tagsUsed ArrayList
            tagsUsed = new ArrayList<>(tagsUsed.stream().distinct().toList()); //gets rid of duplicate tags and sorts them

            Logger.recordOutput("Vision/CamerasUsed", posesUsed.stream().map(pose -> getCameraOfEstimatedPose(pose).name()).toArray(String[]::new));
            Logger.recordOutput("Vision/TagsUsed", getArrayOfTagIDs(tagsUsed));
        } else {
            Logger.recordOutput("Vision/CamerasUsed", new String[0]);
            Logger.recordOutput("Vision/TagsUsed", new int[0]);
        }
    }

    /* --------------------------------------------- PRIVATE METHODS --------------------------------------------- */
    private void updatePose(Camera cam) {
        EstimatedRobotPose pose = estimatedPoses.get(cam);

        if (getCameraStatus(cam)){
            var result = poseEstimators.get(cam).update(photonCameras.get(cam).getLatestResult()); //Optional<EstimatedRobotPose> might be empty or present

            if(result.isPresent()) {
                pose = result.get();
            } else if (pose != null) {
                pose = Timer.getFPGATimestamp() - pose.timestampSeconds < 0.25 ? pose : null; //if pose doesn't update, retain pose if it has been less than 0.25 seconds
            }
        } else {
            pose = null;
        }

        estimatedPoses.put(cam, pose);
        numTagsDetected.put(cam, (pose == null) ? 0 : pose.targetsUsed.size());
    }
    
    private void logCameraData(Camera cam) {
        Logger.recordOutput("Vision/" + cam.getCameraName() + "/CameraStatus", getCameraStatus(cam));
        Logger.recordOutput("Vision/" + cam.getCameraName() + "/NumTags", numTagsDetected.get(cam));

        if (estimatedPoses.get(cam) != null) {
            Logger.recordOutput("Vision/" + cam.getCameraName() + "/TagsDetected", getArrayOfTagIDs(estimatedPoses.get(cam).targetsUsed));
            field.getObject("Vision-" + cam.getCameraName() + "-EstimatedRobotPose").setPose(estimatedPoses.get(cam).estimatedPose.toPose2d());
            Logger.recordOutput("Vision/" + cam.getCameraName() + "/EstimatedRobotPose", estimatedPoses.get(cam).estimatedPose);
        } else {
            Logger.recordOutput("Vision/" + cam.getCameraName() + "/TagsDetected", new int[0]);
        }

        Logger.recordOutput("Vision/" + cam.getCameraName() + "/CameraPose", new Pose3d(globalPoseSupplier.get()).plus(poseEstimators.get(cam).getRobotToCameraTransform()));
    }

    private int[] getArrayOfTagIDs(List<PhotonTrackedTarget> targets) { //takes a list of PhotonTrackedTargets and returns an int array of their fiducial IDs
        return targets.stream().map(x -> x.getFiducialId()).mapToInt(x -> x).toArray();
    }

    private Camera getCameraOfEstimatedPose(EstimatedRobotPose val) {
        Camera result = null;
        for (Camera cam : cameras) {
            if (val.equals(estimatedPoses.get(cam))) {
                result = cam;
                break;
            }
        }

        return result;
    }

    private boolean isPoseReliable(EstimatedRobotPose pose, double maxAmbiguity, double maxDistance) {
        boolean result = false;
        if(pose != null) {
        PhotonTrackedTarget leastAmbiguousTag = pose.targetsUsed.stream().min((tag1, tag2) -> Double.compare(tag1.getPoseAmbiguity(), tag2.getPoseAmbiguity())).orElse(null);
            if(leastAmbiguousTag != null) {
                boolean tooFar = leastAmbiguousTag.getBestCameraToTarget().getTranslation().getNorm() >= Units.feetToMeters(maxDistance);
                boolean tooAmbiguous = leastAmbiguousTag.getPoseAmbiguity() > maxAmbiguity;

                result = !tooFar && !tooAmbiguous;
            }
        }
        return result;
    }

    private double getAverageTagDistance(EstimatedRobotPose pose) {
        double average;

        if (pose == null) {
            average = Double.MAX_VALUE;
        } else {
            average = 0;
            for (PhotonTrackedTarget tag : pose.targetsUsed) average += tag.getBestCameraToTarget().getTranslation().getNorm();
            average /= pose.targetsUsed.size();
        }

        return average;
    }

    /* --------------------------------------------- PUBLIC METHODS --------------------------------------------- */
    public boolean doesCameraSeeTag(Camera cam, int tagID) { //does a camera see a specific tag with an ambiguity of less than 0.1
        EstimatedRobotPose pose = estimatedPoses.get(cam);

        if(pose != null) {
            for(PhotonTrackedTarget target : pose.targetsUsed) {
                if (target.getFiducialId() == tagID && target.getPoseAmbiguity() < 0.1) return true;
            }
        }

        return false;
     }

     public boolean doesCameraSeeAnyTags(Camera cam) { //does a camera see any tags
         return numTagsDetected.get(cam) != 0;
     }
 
    public Transform3d getTagToCamera(Camera cam, int tagID) { //3d offsets of a specific tag to a specific camera used for auto alignment
        EstimatedRobotPose pose = estimatedPoses.get(cam);

        if(pose != null) {
            for(PhotonTrackedTarget target : pose.targetsUsed) {
                if (target.getFiducialId() == tagID) return target.getBestCameraToTarget();
            }
        }

        return new Transform3d();
    }

    public int maxTagsFromOneCam() { //the maximum number of april tags detected by one camera
        return Collections.max(numTagsDetected.values());
    }

    public boolean areTagsVisible() { //do any cameras see any tags
        return maxTagsFromOneCam() > 0;
    }

    public boolean getCameraStatus(Camera cam) {//sees if a camera is connected
        return photonCameras.get(cam).isConnected();
    }

    public int getClosestTagID(Camera cam) {
        int tagID = -1;
        EstimatedRobotPose pose = estimatedPoses.get(cam);
        if(pose != null) {
            PhotonTrackedTarget closestTag = pose.targetsUsed.stream().min((tag1, tag2) -> Double.compare(tag1.getBestCameraToTarget().getTranslation().getNorm(), tag2.getBestCameraToTarget().getTranslation().getNorm())).orElse(null);
            if(closestTag != null) {
                tagID = closestTag.getFiducialId();
            }
        }
        return tagID;
    }

    public int getNumberOfTags(Camera cam) {
        return numTagsDetected.get(cam);
    }

    public int[] getArrayOfTagsDetected(Camera cam) {
        EstimatedRobotPose pose = estimatedPoses.get(cam);

        return (pose == null) ? new int[0] : getArrayOfTagIDs(pose.targetsUsed);
    }
}

