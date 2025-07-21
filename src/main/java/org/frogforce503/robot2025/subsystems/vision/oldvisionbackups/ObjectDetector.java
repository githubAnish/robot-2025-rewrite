package org.frogforce503.robot2025.subsystems.vision.oldvisionbackups;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import org.littletonrobotics.junction.Logger;
import org.frogforce503.robot2025.Robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ObjectDetector extends SubsystemBase {
    private Supplier<Pose2d> globalPoseSupplier;
    
    private PhotonCamera objectDetCam;
    private PhotonPipelineResult result;
    private List<PhotonTrackedTarget> targets;
    private PhotonTrackedTarget bestTarget;

    NetworkTableInstance inst;
    NetworkTable photonVisionTable, objectDetTable;
    DoubleSubscriber objectDistanceXSub,objectDistanceYSub, objectFieldXSub, objectFieldYSub, pitchSub, yawSub;
    DoublePublisher objectDistanceXPub, objectDistanceYPub, objectFieldXPub, objectFieldYPub, pitchPub, yawPub; 

    private Translation2d lastBestObjectToRobot = new Translation2d();
    private Translation2d lastBestObjectToField = new Translation2d();

    private SORTING_MODE sortingMode = SORTING_MODE.LEFT_TO_RIGHT;

    public static boolean OBJECT_DETECTION_DISABLED = true; // TODO: UPDATE THIS IF NEEDED

    public ObjectDetector(Supplier<Pose2d> globalPoseSupplier) {
        this.globalPoseSupplier = globalPoseSupplier;

        objectDetCam = new PhotonCamera("ObjectDetCam");
        inst = NetworkTableInstance.getDefault();

        photonVisionTable = inst.getTable("photonvision");
        objectDetTable = photonVisionTable.getSubTable("object_detection_camera");

        objectDistanceXPub = photonVisionTable.getDoubleTopic("objectDistanceX").publish();
        objectDistanceYPub = photonVisionTable.getDoubleTopic("objectDistanceY").publish();
        objectFieldXPub =  photonVisionTable.getDoubleTopic("objectFieldX").publish();
        objectFieldYPub = photonVisionTable.getDoubleTopic("objectFieldY").publish();
        pitchPub = photonVisionTable.getDoubleTopic("pitch").publish();
        yawPub = photonVisionTable.getDoubleTopic("yaw").publish();

        objectDistanceXSub = photonVisionTable.getDoubleTopic("objectDistanceX").subscribe(0);
        objectDistanceYSub = photonVisionTable.getDoubleTopic("objectDistanceY").subscribe(0);
        objectFieldXSub =  photonVisionTable.getDoubleTopic("objectFieldX").subscribe(0);
        objectFieldYSub = photonVisionTable.getDoubleTopic("objectFieldY").subscribe(0);
        yawSub = photonVisionTable.getDoubleTopic("yaw").subscribe(0);
        pitchSub = photonVisionTable.getDoubleTopic("pitch").subscribe(0);
       
    }
    
    public void updateTargets() {
        result = objectDetCam.getLatestResult();

        if(result.hasTargets()){
            targets = result.getTargets();
            bestTarget = result.getBestTarget();
        }
    }

    public boolean objectInView() {
        if (this.result == null) {
            return false;
        }
        return this.result.hasTargets();
    }

    // private int calls = 0;


    private SIDE lastSelectedSide = null;
    public SIDE firstAvailableObject() {
        System.out.println("CHECKING CAMERA");

        if (RobotBase.isSimulation() || OBJECT_DETECTION_DISABLED) {
            boolean leftToRight = this.sortingMode == SORTING_MODE.LEFT_TO_RIGHT;
            if (lastSelectedSide == null) {
                lastSelectedSide = leftToRight ? SIDE.LEFT : SIDE.RIGHT;
                return lastSelectedSide;
            }

            switch (lastSelectedSide) {
                case LEFT:
                    lastSelectedSide = leftToRight ? SIDE.CENTER : SIDE.NONE;
                break;
                case CENTER:
                    lastSelectedSide = leftToRight ? SIDE.RIGHT : SIDE.LEFT;
                break;
                case RIGHT:
                    lastSelectedSide = leftToRight ? SIDE.NONE : SIDE.CENTER;
                case NONE:
                    lastSelectedSide = SIDE.NONE;
                break;
            }

            return lastSelectedSide;
        }

        if (this.bestTarget == null) {
            return SIDE.NONE;
        }

        if (this.bestTarget.getYaw() < -10) {
            return SIDE.LEFT;
        } else if (this.bestTarget.getYaw() < 10) {
            return SIDE.CENTER;
        } else {
            return SIDE.RIGHT;
        }
    }

    public void setSortingMode(SORTING_MODE mode) { // should NOT change the pipeline
        if (mode == SORTING_MODE.LEFT_TO_RIGHT) {
            objectDetCam.setPipelineIndex(0);
        } else if (mode == SORTING_MODE.RIGHT_TO_LEFT) {
            objectDetCam.setPipelineIndex(1);
        } else {
            objectDetCam.setPipelineIndex(2);
        }
        this.sortingMode = mode;
    }



    private Translation2d calculateObjectToRobot(PhotonTrackedTarget target) {
        double distance = PhotonUtils.calculateDistanceToTargetMeters(
            Robot.bot.OBJECT_DETECTION_CAMERA_TO_CENTER.getZ(),
            Units.inchesToMeters(1.75/2),
            Robot.bot.OBJECT_DETECTION_CAMERA_TO_CENTER.getRotation().getY(),
            Units.degreesToRadians(target.getPitch())
        );

        Translation2d objectToCamera = PhotonUtils.estimateCameraToTargetTranslation(distance, Rotation2d.fromDegrees(-target.getYaw()));
        Translation2d objectToRobot =  objectToCamera.plus(Robot.bot.OBJECT_DETECTION_CAMERA_TO_CENTER.getTranslation().toTranslation2d());
        
        return objectToRobot;

    }

    private Translation2d calculateObjectToField(PhotonTrackedTarget target) {
        Translation2d robotToObject = calculateObjectToRobot(target);
        Translation2d robotToField = globalPoseSupplier.get().getTranslation();
        Translation2d objectToField = robotToObject.rotateBy(globalPoseSupplier.get().getRotation()).plus(robotToField);
        
        return objectToField;
    }

    public Translation2d getBestObjectToRobot() {
        if (RobotBase.isSimulation())
            return new Translation2d();
        return this.lastBestObjectToRobot;
    }

    @Override
    public void periodic() {
        updateTargets();

        if (objectInView()) {
            this.lastBestObjectToRobot = calculateObjectToRobot(bestTarget);
            this.lastBestObjectToField = calculateObjectToField(bestTarget);

            Logger.recordOutput("Vision/ObjectToRobot", new Pose2d(lastBestObjectToRobot, Rotation2d.kZero));
            Logger.recordOutput("Vision/Object", new Pose2d(lastBestObjectToField, Rotation2d.kZero));
        }

        Logger.recordOutput("Vision/ObjectInView", objectInView());
    }    
    

    public List<Translation2d> getAllObjectPositions(){
        List<Translation2d> objects = new ArrayList<>();

        for(PhotonTrackedTarget target : targets){
            objects.add(new Translation2d(target.getYaw(), target.getPitch()));
        }

        return objects;
    }

    public enum SIDE {
        LEFT, CENTER, RIGHT, NONE;

        public int priorityIndex() {
            switch (this) {
                case LEFT:
                    return 0;
                case CENTER:
                    return 1;
                case RIGHT:
                    return 2;
                case NONE:
                    return 3;
            }
            return 3;
        }
    }

    public enum SORTING_MODE {
        LEFT_TO_RIGHT, RIGHT_TO_LEFT, CENTERMOST;
    }

    public enum MODELS {
        ALGAE_AND_CORAL(0), BRANCHES(1), BUMPERS(2);

        int pipelineIndex;

        private MODELS(int index) {
            pipelineIndex = index;
        }

        public int getIndex() {
            return pipelineIndex;
        }
    }

    public void setModels(MODELS mode) { // should NOT change the pipeline
        objectDetCam.setPipelineIndex(mode.getIndex());
    }
}

    
