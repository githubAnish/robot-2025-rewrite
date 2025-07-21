package org.frogforce503.robot2025.subsystems.vision.camera_types;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

public class YoloCamera extends PhotonVisionCamera {
    public YoloCamera(String cameraName, Transform3d robotToCamera) {
        super(cameraName, CameraType.OBJECT_DETECTION, robotToCamera);
    }

    @Override
    public void configSimCamProperties() {
        //Object Detection Camera settings....
        getCameraProperties().setCalibration(640, 480, Rotation2d.fromDegrees(127.83));
        getCameraProperties().setCalibError(0.25, 0.08);
        getCameraProperties().setFPS(20);
        getCameraProperties().setAvgLatencyMs(35);
        getCameraProperties().setLatencyStdDevMs(5);

        getCameraSim().enableRawStream(true);
        getCameraSim().enableProcessedStream(true);
        getCameraSim().enableDrawWireframe(true);
    }

    @Override
    public void update() {
        // TO-DO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'update'");
    }

    @Override
    public void logData() {
        // TO-DO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'logData'");
    }
    
}
