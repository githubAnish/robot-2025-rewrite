package org.frogforce503.robot2025.subsystems.vision.camera_types;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;

import edu.wpi.first.math.geometry.Transform3d;
import lombok.Getter;

public abstract class PhotonVisionCamera {
    @Getter private String cameraName;
    @Getter private PhotonCamera camera;
    @Getter private PhotonCameraSim cameraSim;
    @Getter private SimCameraProperties cameraProperties;
    @Getter private Transform3d robotToCamera;
    @Getter private CameraType cameraType;

    public enum CameraType {
        APRIL_TAG_DETECTION,
        OBJECT_DETECTION
    }
    
    public PhotonVisionCamera(String cameraName, CameraType cameraType, Transform3d robotToCamera) {
        this.cameraName = cameraName;
        this.camera = new PhotonCamera(cameraName);
        this.cameraType = cameraType;
        this.robotToCamera = robotToCamera;
        this.cameraProperties = new SimCameraProperties();
        this.cameraSim = new PhotonCameraSim(this.camera, this.cameraProperties);
    }

    public boolean getStatus() {
        return camera.isConnected();
    }

    public abstract void update();

    public abstract void logData();

    public abstract void configSimCamProperties();
}
