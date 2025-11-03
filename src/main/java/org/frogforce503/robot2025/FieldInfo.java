package org.frogforce503.robot2025;

import org.frogforce503.lib.math.Polygon2d;
import org.frogforce503.robot2025.constants.field.FieldConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lombok.Getter;

/** Wrapper class for all field-related information. */
public class FieldInfo extends Field2d {
    private Alliance allianceColor = Alliance.Red;
    private boolean allianceColorBeenOverriden = false;

    @Getter private FieldConfig config;

    public FieldInfo() {
        this.config = new FieldConfig();

        SmartDashboard.putData("Field", this);
    }

    public void setAlliance(Alliance color) {
        allianceColorBeenOverriden = true;
        this.allianceColor = color;
    }

    public Alliance getAlliance() {
        return 
            allianceColorBeenOverriden || DriverStation.getAlliance().isEmpty()
                ? this.allianceColor
                : DriverStation.getAlliance().get();
    }

    public boolean onRedAlliance() {
        return getAlliance() == Alliance.Red;
    }

    public boolean onBlueAlliance() {
        return getAlliance() == Alliance.Blue;
    }

    // Configuration
    public Pose2d getTagById(int tagID) {
        return config.getTagById(tagID);
    }

    public <T> T flip(T red, T blue) {
        return onRedAlliance() ? red : blue;
    }

    // Reefscape-Specific objects
    public Polygon2d getRedReef() {
        return
            new Polygon2d(
                config.Red_Algae_AB
                    .interpolate(config.Red_Algae_GH, 0.5),
                config.RedReefSideLength + config.RedReefInnerToOuter,
                6,
                Rotation2d.fromDegrees(30));
    }

    public Polygon2d getBlueReef() {
        return
            new Polygon2d(
                config.Blue_Algae_AB
                    .interpolate(config.Blue_Algae_GH, 0.5),
                config.BlueReefSideLength + config.BlueReefInnerToOuter,
                6,
                Rotation2d.fromDegrees(30));
    }

    public Translation2d getLeftStation() {
        return flip(config.RedLeftStation, config.BlueLeftStation);
    }

    public Translation2d getRightStation() {
        return flip(config.RedRightStation, config.BlueRightStation);
    }

    public Translation2d getLeftCage() {
        return flip(config.RedLeftCage, config.BlueLeftCage);
    }

    public Translation2d getCenterCage() {
        return flip(config.RedCenterCage, config.BlueCenterCage);
    }

    public Translation2d getRightCage() {
        return flip(config.RedRightCage, config.BlueRightCage);
    }
}