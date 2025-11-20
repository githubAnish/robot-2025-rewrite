package org.frogforce503.robot2025.config.field;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public enum FieldVenue {
    Shop("Shop.json", AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded));

    public String filePath;
    public AprilTagFieldLayout aprilTagFieldLayout;

    private FieldVenue(String filePath, AprilTagFieldLayout aprilTagFieldLayout) {
        this.filePath = filePath;
        this.aprilTagFieldLayout = aprilTagFieldLayout;
    }
}
