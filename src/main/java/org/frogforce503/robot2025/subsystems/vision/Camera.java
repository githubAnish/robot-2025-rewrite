package org.frogforce503.robot2025.subsystems.vision;

import org.frogforce503.robot2025.Robot;
import org.frogforce503.robot2025.subsystems.vision.camera_types.PhotonVisionCamera.CameraType;

import edu.wpi.first.math.geometry.Transform3d;
import lombok.Getter;

public enum Camera {
    FRONT_LEFT("FrontLeftCamera", Robot.bot.FRONT_LEFT_CAMERA_TO_CENTER, CameraType.APRIL_TAG_DETECTION), 
    FRONT_RIGHT("FrontRightCamera", Robot.bot.FRONT_RIGHT_CAMERA_TO_CENTER, CameraType.APRIL_TAG_DETECTION), 
    ELEVATOR_BACK("ElevatorBackCamera", Robot.bot.ELEVATOR_BACK_CAMERA_TO_CENTER, CameraType.APRIL_TAG_DETECTION), 
    LOWER_FRONT_RIGHT("LowerFrontRightCamera", Robot.bot.LOWER_FRONT_RIGHT_CAMERA_TO_CENTER, CameraType.APRIL_TAG_DETECTION);

    @Getter private String cameraName;
    @Getter private Transform3d cameraToRobot;
    @Getter private CameraType cameraType;

    private Camera(String cameraName, Transform3d cameraToRobot, CameraType cameraType) {
        this.cameraName = cameraName;
        this.cameraToRobot = cameraToRobot;
        this.cameraType = cameraType;
    }
}