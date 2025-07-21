package org.frogforce503.robot2025.subsystems.vision.camera_types;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

import org.frogforce503.robot2025.fields.FieldInfo;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import lombok.Getter;

public class AprilTagCamera extends PhotonVisionCamera {
    private final FieldInfo field;
    private Supplier<Pose2d> globalPoseSupplier;

    @Getter private EstimatedRobotPose estimatedPose;
    private PhotonPoseEstimator poseEstimator;
    private int[] tagsDetected;
    @Getter private EstimatedRobotPose specializedPose;
    private PhotonPoseEstimator specializedPoseEstimator;

    public AprilTagCamera(FieldInfo field, Supplier<Pose2d> globalPoseSupplier, String cameraName, Transform3d robotToCamera) {
        super(cameraName, CameraType.APRIL_TAG_DETECTION, robotToCamera);

        this.field = field;
        this.globalPoseSupplier = globalPoseSupplier;

        AprilTagFieldLayout aprilTagFieldLayout = field.getConfiguration().fieldLayout;
        poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
        specializedPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.PNP_DISTANCE_TRIG_SOLVE, robotToCamera);
    }
    
    @Override
    public void configSimCamProperties() {
        getCameraProperties().setCalibration(1280, 800, Rotation2d.fromDegrees(78.2));
        getCameraProperties().setCalibError(0.25, 0.08);
        getCameraProperties().setFPS(30);
        getCameraProperties().setAvgLatencyMs(35);
        getCameraProperties().setLatencyStdDevMs(5);

        getCameraSim().enableRawStream(true);
        getCameraSim().enableProcessedStream(true);
        getCameraSim().enableDrawWireframe(true);
    }

    public int[] getArrayOfTagIDs() {
        return tagsDetected;
    }

    public int getNumberOfTagsDetected() {
        if (tagsDetected != null) {
            return tagsDetected.length;
        }
        return 0;
    }

    public double getAverageTagDistance() {
        double average = getNumberOfTagsDetected() == 0 ? Double.MAX_VALUE : 0;
        for(PhotonTrackedTarget target : estimatedPose.targetsUsed) {
            average += target.getBestCameraToTarget().getTranslation().getNorm();
        }
        return average / estimatedPose.targetsUsed.size();
    }

    public PhotonTrackedTarget getClosestTag() {
        PhotonTrackedTarget closestTag = null;
        double closestDistance = Double.MAX_VALUE;

        if (estimatedPose != null) {
            for(PhotonTrackedTarget target : estimatedPose.targetsUsed) {
                double distance = target.getBestCameraToTarget().getTranslation().getNorm();
                if(distance < closestDistance) {
                    closestDistance = distance;
                    closestTag = target;
                }
            }
        }

        return closestTag;
    }

    public PhotonTrackedTarget getLeastAmbiguousTag() {
        PhotonTrackedTarget leastAmbiguousTag = null;
        double leastAmbiguity = Double.MAX_VALUE;

        if (estimatedPose != null) {
            for(PhotonTrackedTarget target : estimatedPose.targetsUsed) {
                double ambiguity = target.getPoseAmbiguity();
                if(ambiguity < leastAmbiguity) {
                    leastAmbiguity = ambiguity;
                    leastAmbiguousTag = target;
                }
            }
        }

        return leastAmbiguousTag;
    }


    public Pose2d getCalculatedTagPose(int tagID) {
        Pose2d tagPose = null;

        if (isSpecializedPoseReliable(5) && specializedPose.targetsUsed.get(0).getFiducialId() == tagID) {
            Pose3d tagPose3d = new Pose3d(globalPoseSupplier.get());
            tagPose3d = tagPose3d.plus(getRobotToCamera()).plus(specializedPose.targetsUsed.get(0).getBestCameraToTarget());

            tagPose = tagPose3d.toPose2d();
        } 

        return tagPose;
    }

    public boolean isPoseReliable(double maxAmbiguity, double maxDistance) {
        if (estimatedPose != null) {
            boolean tooFar = getClosestTag().getBestCameraToTarget().getTranslation().getNorm() >= Units.feetToMeters(maxDistance) && getAverageTagDistance() >= Units.feetToMeters(maxDistance);
            boolean tooAmbiguous = getLeastAmbiguousTag().getPoseAmbiguity() > maxAmbiguity;

            return !tooFar && !tooAmbiguous;
        }

        return false;
    }

    public boolean isSpecializedPoseReliable(double maxDistance) {
        boolean isPoseReliable = false;

        if (specializedPose != null) {
            isPoseReliable = getClosestTag().getBestCameraToTarget().getTranslation().getNorm() <= Units.feetToMeters(maxDistance);
        }

        return isPoseReliable;
    }

    @Override
    public void update() {
        if (getStatus()) {
            specializedPoseEstimator.addHeadingData(Timer.getFPGATimestamp(), globalPoseSupplier.get().getRotation());
            var results = getCamera().getAllUnreadResults();

            for(var result : results) {
                var poseResult = poseEstimator.update(result);

                if(poseResult.isPresent()) {
                    estimatedPose = poseResult.get();
                    tagsDetected = estimatedPose.targetsUsed.stream().mapToInt(PhotonTrackedTarget::getFiducialId).toArray();
                } else {
                    estimatedPose = null;
                    tagsDetected = new int[0];
                }
            
                var specializedPoseResult = specializedPoseEstimator.update(result);
                
                if(specializedPoseResult.isPresent()) {
                    specializedPose = specializedPoseResult.get();
                } else {
                    specializedPose = null;
                }
                
            }

        } else {
            tagsDetected = new int[0];
        }
    }

    @Override
    public void logData() {
        Logger.recordOutput("Vision/" + getCamera().getName() + "/CameraStatus", getStatus());
        if(estimatedPose != null) {
            field.getObject("Vision-" + getCamera().getName() + "-EstimatedRobotPose").setPose(estimatedPose.estimatedPose.toPose2d());
            // Logger.recordOutput("Vision/" + getCamera().getName() + "/EstimatedPose", estimatedPose.estimatedPose.toPose2d());
        } 
        if(specializedPose != null) {
            field.getObject("Vision-" + getCamera().getName() + "-SpecializedRobotPose").setPose(specializedPose.estimatedPose.toPose2d());
            // Logger.recordOutput("Vision/" + getCamera().getName() + "/SpecializedPose", specializedPose.estimatedPose.toPose2d());
        } 
        Logger.recordOutput("Vision/" + getCamera().getName() + "/NumTagsDetected", getNumberOfTagsDetected());
        Logger.recordOutput("Vision/" + getCamera().getName() + "/TagsDetected", tagsDetected);
        Logger.recordOutput("Vision/" + getCamera().getName() + "/SpecializedPoseReliable", isSpecializedPoseReliable(8));
    }
}