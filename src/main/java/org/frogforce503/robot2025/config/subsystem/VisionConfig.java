package org.frogforce503.robot2025.config.subsystem;

import edu.wpi.first.math.geometry.Transform3d;

public record VisionConfig(
    Transform3d FRONT_LEFT_CAMERA_TO_CENTER,
    Transform3d FRONT_RIGHT_CAMERA_TO_CENTER,
    Transform3d ELEVATOR_BACK_CAMERA_TO_CENTER,
    Transform3d LOWER_FRONT_RIGHT_CAMERA_TO_CENTER) {}