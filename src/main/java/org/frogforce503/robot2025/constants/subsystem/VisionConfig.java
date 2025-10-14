package org.frogforce503.robot2025.constants.subsystem;

import edu.wpi.first.math.geometry.Transform3d;

/*
 * Positive X axis points ahead, positive Y axis points left, and positive Z axis points up referenced from the floor. 
 * When viewed with each positive axis pointing toward you, counter-clockwise (CCW) is a positive value and clockwise (CW) is a negative value.
 * 
 * Positive X: Front of Robot
 * Positive Y: Left of Robot
 * Positive Z: Up
 * 
 * Positive Roll (Rotation about X): Robot rolls to its right
 * Positive Pitch (Rotation about Y): Robot points downwards
 * Positive Yaw (Rotation about Z): Robot rotates left
 */
public class VisionConfig {
    // Camera positions relative to center of robot (insert below) as Transform3d
    public Transform3d FRONT_LEFT_CAMERA_TO_CENTER;
    public Transform3d UPPER_FRONT_RIGHT_CAMERA_TO_CENTER;
    public Transform3d LOWER_FRONT_RIGHT_CAMERA_TO_CENTER;
    public Transform3d ELEVATOR_BACK_CAMERA_TO_CENTER;
    public Transform3d ELEVATOR_FRONT_CAMERA_TO_CENTER;

    public Transform3d OBJECT_DETECTION_CAMERA_TO_CENTER;
}