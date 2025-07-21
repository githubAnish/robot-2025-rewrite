package org.frogforce503.robot2025.fields;

import org.frogforce503.lib.math.Polygon2d;
import org.frogforce503.robot2025.fields.FieldConfig.Venue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lombok.Getter;

/** Wrapper class for all field-related information. */
public class FieldInfo {
    @Getter private FieldState status;
    @Getter private FieldConfig configuration;
    @Getter private Field2d display;

    public FieldInfo() {
        this.configuration = new FieldConfig();
        this.status = new FieldState();
        this.display = new Field2d();

        SmartDashboard.putData("Field", this.display);

        configuration.setVenue(Venue.Shop);
    }

    // Reef Maps
    public Polygon2d getRedReef() {
        return
            new Polygon2d(
                configuration.Red_Algae_AB
                    .interpolate(configuration.Red_Algae_GH, 0.5),
                configuration.RedReefSideLength + configuration.RedReefInnerToOuter,
                6,
                Rotation2d.fromDegrees(30));
    }

    public Polygon2d getBlueReef() {
        return
            new Polygon2d(
                configuration.Blue_Algae_AB
                    .interpolate(configuration.Blue_Algae_GH, 0.5),
                configuration.BlueReefSideLength + configuration.BlueReefInnerToOuter,
                6,
                Rotation2d.fromDegrees(30));
    }

    // Status
    public boolean onRedAlliance() {
        return status.isAllianceRed();
    }

    public boolean onBlueAlliance() {
        return status.isAllianceBlue();
    }

    public void overrideAllianceColor(Alliance color) {
        status.overrideAllianceColor(color);
    }

    // Display
    public void setRobotPose(Pose2d robotPose) {
        display.setRobotPose(robotPose);
    }

    public FieldObject2d getObject(String name) {
        return display.getObject(name);
    }

    // Configuration
    public void setVenue(Venue venue) {
        configuration.setVenue(venue);
    }
    
    public Pose2d getTagById(int tagID) {
        return configuration.getTagById(tagID);
    }

    public <T> T flip(T red, T blue) {
        return onRedAlliance() ? red : blue;
    }

    public Translation2d getLeftStation() {
        return flip(configuration.RedLeftStation, configuration.BlueLeftStation);
    }

    public Translation2d getRightStation() {
        return flip(configuration.RedRightStation, configuration.BlueRightStation);
    }

    public Translation2d getLeftCage() {
        return flip(configuration.RedLeftCage, configuration.BlueLeftCage);
    }

    public Translation2d getCenterCage() {
        return flip(configuration.RedCenterCage, configuration.BlueCenterCage);
    }

    public Translation2d getRightCage() {
        return flip(configuration.RedRightCage, configuration.BlueRightCage);
    }
}