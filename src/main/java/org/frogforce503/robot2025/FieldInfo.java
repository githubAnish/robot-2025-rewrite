package org.frogforce503.robot2025;

import org.frogforce503.lib.math.Polygon2d;
import org.frogforce503.robot2025.constants.field.FieldConfig;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lombok.Getter;

/** Wrapper class for all field-related information. */
public class FieldInfo extends Field2d {
    // Config
    @Getter private FieldConfig config = new FieldConfig();

    // Selectors
    private final LoggedDashboardChooser<Alliance> allianceSelector = new LoggedDashboardChooser<>("Alliance Color");

    public FieldInfo() {
        SmartDashboard.putData("Field", this);

        this.allianceSelector.addDefaultOption("Red", Alliance.Red);
        this.allianceSelector.addOption("Blue", Alliance.Blue);
    }

    public Alliance getAlliance() {
        return
            RobotBase.isSimulation() || DriverStation.getAlliance().isEmpty() // if in sim or alliance not known
                ? allianceSelector.get()
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
                config.Red_Algae_AB.interpolate(config.Red_Algae_GH, 0.5),
                config.RedReefSideLength + config.RedReefInnerToOuter,
                6,
                Rotation2d.fromDegrees(30));
    }

    public Polygon2d getBlueReef() {
        return
            new Polygon2d(
                config.Blue_Algae_AB.interpolate(config.Blue_Algae_GH, 0.5),
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
}