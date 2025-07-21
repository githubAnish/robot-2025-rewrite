package org.frogforce503.robot2025.fields;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;

public class FieldConfig {
    // Field Variables
    public Translation2d FIELD_DIMENSIONS;
    public final double FIELD_X = Units.feetToMeters(57) + Units.inchesToMeters(6) + Units.inchesToMeters(7.0 / 8.0);
    public final double FIELD_Y = Units.feetToMeters(26) + Units.inchesToMeters(5);

    // Blue Measurements
    public double BlueWallToRightAlgae;
    public double BlueCenterAlgaeToRightAlgae;
    public double BlueCenterAlgaeToLeftAlgae;
    public double BlueCenterAlgaeToReefSide;

    public double BlueWallToReef_ALMiddle_X;
    public double BlueLeftHPWallToReefALMiddle_Y;

    public double BlueReefEdgeToBranch;
    public double BlueReefSideLength;
    public double BlueReefInnerToOuter;

    public double BlueLeftHPWallToLeftCage_Y;
    public double BlueInitLineToLeftCage;
    public double BlueCenterCageToLeftCage;
    public double BlueCenterCageToRightCage;

    public double BlueProcToInitLine;

    // Red Measurements
    public double RedWallToRightAlgae;
    public double RedCenterAlgaeToRightAlgae;
    public double RedCenterAlgaeToLeftAlgae;
    public double RedCenterAlgaeToReefSide;

    public double RedWallToReef_ALMiddle_X;
    public double RedLeftHPWallToReefALMiddle_Y;

    public double RedReefEdgeToBranch;
    public double RedReefSideLength;
    public double RedReefInnerToOuter;

    public double RedLeftHPWallToLeftCage_Y;
    public double RedInitLineToLeftCage;
    public double RedCenterCageToLeftCage;
    public double RedCenterCageToRightCage;

    public double RedProcToInitLine;

    // Blue Locations
    public Translation2d BlueLeftAlgae, BlueCenterAlgae, BlueRightAlgae;

    public Translation2d Blue_Coral_A, Blue_Coral_B, Blue_Coral_C, Blue_Coral_D, Blue_Coral_E, Blue_Coral_F,
                  Blue_Coral_G, Blue_Coral_H, Blue_Coral_I, Blue_Coral_J, Blue_Coral_K, Blue_Coral_L;

    public Translation2d Blue_Algae_AB, Blue_Algae_CD, Blue_Algae_EF, Blue_Algae_GH, Blue_Algae_IJ, Blue_Algae_KL;
    
    public Translation2d BlueProc;
    
    public Translation2d BlueLeftCage, BlueCenterCage, BlueRightCage;

    public double BlueInitLine;

    public Translation2d BlueLeftStation;
    public Translation2d BlueRightStation;

    // Red Locations
    public Translation2d RedLeftAlgae, RedCenterAlgae, RedRightAlgae;

    public Translation2d Red_Coral_A, Red_Coral_B, Red_Coral_C, Red_Coral_D, Red_Coral_E, Red_Coral_F,
                  Red_Coral_G, Red_Coral_H, Red_Coral_I, Red_Coral_J, Red_Coral_K, Red_Coral_L;

    public Translation2d Red_Algae_AB, Red_Algae_CD, Red_Algae_EF, Red_Algae_GH, Red_Algae_IJ, Red_Algae_KL;
    
    public Translation2d RedProc;
    
    public Translation2d RedLeftCage, RedCenterCage, RedRightCage;

    public double RedInitLine;

    public Translation2d RedLeftStation;
    public Translation2d RedRightStation;

    // Key positions used to calculate locations above
    // Blue
    public Translation2d BlueALMiddle;
    public Translation2d BlueBCMiddle;
    public Translation2d BlueDEMiddle;
    public Translation2d BlueFGMiddle;
    public Translation2d BlueHIMiddle;
    public Translation2d BlueJKMiddle;

    // Red
    public Translation2d RedALMiddle;
    public Translation2d RedBCMiddle;
    public Translation2d RedDEMiddle;
    public Translation2d RedFGMiddle;
    public Translation2d RedHIMiddle;
    public Translation2d RedJKMiddle;

    // AprilTag Layout on Field
    public AprilTagFieldLayout fieldLayout;

    private void loadConstants(String file, AprilTagFieldLayout aprilTagFieldLayout) throws FileNotFoundException, IOException, ParseException {
        JSONObject field =
            (JSONObject) new JSONParser()
                .parse(
                    new FileReader(
                        Filesystem.getDeployDirectory().getAbsolutePath() + "/fields/" + file));

        // Blue Measurements
        BlueWallToRightAlgae = convertJsonEntryToDouble(field, "BlueWallToRightAlgae");
        BlueCenterAlgaeToRightAlgae = convertJsonEntryToDouble(field, "BlueCenterAlgaeToRightAlgae");
        BlueCenterAlgaeToLeftAlgae = convertJsonEntryToDouble(field, "BlueCenterAlgaeToLeftAlgae");
        BlueCenterAlgaeToReefSide = convertJsonEntryToDouble(field, "BlueCenterAlgaeToReefSide");

        BlueWallToReef_ALMiddle_X = convertJsonEntryToDouble(field, "BlueWallToReef_ALMiddle_X");
        BlueLeftHPWallToReefALMiddle_Y = convertJsonEntryToDouble(field, "BlueLeftHPWallToReef_ALMiddle_Y");

        BlueReefEdgeToBranch = convertJsonEntryToDouble(field, "BlueReefEdgeToBranch");
        BlueReefSideLength = convertJsonEntryToDouble(field, "BlueReefSideLength");
        BlueReefInnerToOuter = convertJsonEntryToDouble(field, "BlueReefInnerToOuter");

        BlueLeftHPWallToLeftCage_Y = convertJsonEntryToDouble(field, "BlueLeftHPWallToLeftCage_Y");
        BlueInitLineToLeftCage = convertJsonEntryToDouble(field, "BlueInitLineToLeftCage");
        BlueCenterCageToLeftCage = convertJsonEntryToDouble(field, "BlueCenterCageToLeftCage");
        BlueCenterCageToRightCage = convertJsonEntryToDouble(field, "BlueCenterCageToRightCage");

        BlueProcToInitLine = convertJsonEntryToDouble(field, "BlueProcToInitLine");

        // Red Measurements
        RedWallToRightAlgae = convertJsonEntryToDouble(field, "RedWallToRightAlgae");
        RedCenterAlgaeToRightAlgae = convertJsonEntryToDouble(field, "RedCenterAlgaeToRightAlgae");
        RedCenterAlgaeToLeftAlgae = convertJsonEntryToDouble(field, "RedCenterAlgaeToLeftAlgae");
        RedCenterAlgaeToReefSide = convertJsonEntryToDouble(field, "RedCenterAlgaeToReefSide");

        RedWallToReef_ALMiddle_X = convertJsonEntryToDouble(field, "RedWallToReef_ALMiddle_X");
        RedLeftHPWallToReefALMiddle_Y = convertJsonEntryToDouble(field, "RedLeftHPWallToReef_ALMiddle_Y");

        RedReefEdgeToBranch = convertJsonEntryToDouble(field, "RedReefEdgeToBranch");
        RedReefSideLength = convertJsonEntryToDouble(field, "RedReefSideLength");
        RedReefInnerToOuter = convertJsonEntryToDouble(field, "RedReefInnerToOuter");

        RedLeftHPWallToLeftCage_Y = convertJsonEntryToDouble(field, "RedLeftHPWallToLeftCage_Y");
        RedInitLineToLeftCage = convertJsonEntryToDouble(field, "RedInitLineToLeftCage");
        RedCenterCageToLeftCage = convertJsonEntryToDouble(field, "RedCenterCageToLeftCage");
        RedCenterCageToRightCage = convertJsonEntryToDouble(field, "RedCenterCageToRightCage");

        RedProcToInitLine = convertJsonEntryToDouble(field, "RedProcToInitLine");

        // Blue Locations
        BlueALMiddle = new Translation2d(BlueWallToReef_ALMiddle_X, FIELD_Y - BlueLeftHPWallToReefALMiddle_Y);
        BlueBCMiddle = BlueALMiddle.plus(new Translation2d(BlueReefSideLength, Rotation2d.fromDegrees(270)));
        BlueDEMiddle = BlueBCMiddle.plus(new Translation2d(BlueReefSideLength, Rotation2d.fromDegrees(330)));
        BlueFGMiddle = BlueDEMiddle.plus(new Translation2d(BlueReefSideLength, Rotation2d.fromDegrees(30)));
        BlueHIMiddle = BlueFGMiddle.plus(new Translation2d(BlueReefSideLength, Rotation2d.fromDegrees(90)));
        BlueJKMiddle = BlueHIMiddle.plus(new Translation2d(BlueReefSideLength, Rotation2d.fromDegrees(150)));

        Blue_Coral_A = BlueALMiddle.plus(new Translation2d(BlueReefEdgeToBranch, Rotation2d.fromDegrees(270)));
        Blue_Coral_B = BlueBCMiddle.plus(new Translation2d(BlueReefEdgeToBranch, Rotation2d.fromDegrees(90)));
        Blue_Coral_C = BlueBCMiddle.plus(new Translation2d(BlueReefEdgeToBranch, Rotation2d.fromDegrees(330)));
        Blue_Coral_D = BlueDEMiddle.plus(new Translation2d(BlueReefEdgeToBranch, Rotation2d.fromDegrees(150)));
        Blue_Coral_E = BlueDEMiddle.plus(new Translation2d(BlueReefEdgeToBranch, Rotation2d.fromDegrees(30)));
        Blue_Coral_F = BlueFGMiddle.plus(new Translation2d(BlueReefEdgeToBranch, Rotation2d.fromDegrees(210)));
        Blue_Coral_G = BlueFGMiddle.plus(new Translation2d(BlueReefEdgeToBranch, Rotation2d.fromDegrees(90)));
        Blue_Coral_H = BlueHIMiddle.plus(new Translation2d(BlueReefEdgeToBranch, Rotation2d.fromDegrees(270)));
        Blue_Coral_I = BlueHIMiddle.plus(new Translation2d(BlueReefEdgeToBranch, Rotation2d.fromDegrees(150)));
        Blue_Coral_J = BlueJKMiddle.plus(new Translation2d(BlueReefEdgeToBranch, Rotation2d.fromDegrees(330)));
        Blue_Coral_K = BlueJKMiddle.plus(new Translation2d(BlueReefEdgeToBranch, Rotation2d.fromDegrees(210)));
        Blue_Coral_L = BlueALMiddle.plus(new Translation2d(BlueReefEdgeToBranch, Rotation2d.fromDegrees(30)));

        Blue_Algae_AB = BlueALMiddle.plus(new Translation2d(BlueReefSideLength / 2, Rotation2d.fromDegrees(270)));
        Blue_Algae_CD = BlueBCMiddle.plus(new Translation2d(BlueReefSideLength / 2, Rotation2d.fromDegrees(330)));
        Blue_Algae_EF = BlueDEMiddle.plus(new Translation2d(BlueReefSideLength / 2, Rotation2d.fromDegrees(30)));
        Blue_Algae_GH = BlueFGMiddle.plus(new Translation2d(BlueReefSideLength / 2, Rotation2d.fromDegrees(90)));
        Blue_Algae_IJ = BlueHIMiddle.plus(new Translation2d(BlueReefSideLength / 2, Rotation2d.fromDegrees(150)));
        Blue_Algae_KL = BlueJKMiddle.plus(new Translation2d(BlueReefSideLength / 2, Rotation2d.fromDegrees(210)));

        BlueCenterAlgae = Blue_Algae_AB.plus(new Translation2d(-BlueCenterAlgaeToReefSide, 0));
        BlueLeftAlgae = BlueCenterAlgae.plus(new Translation2d(0, BlueCenterAlgaeToLeftAlgae));
        BlueRightAlgae = BlueCenterAlgae.plus(new Translation2d(0, -BlueCenterAlgaeToRightAlgae));

        BlueInitLine = (FIELD_X / 2) - BlueInitLineToLeftCage;

        BlueProc = new Translation2d(BlueInitLine - BlueProcToInitLine, 0);
        
        BlueLeftCage = new Translation2d(BlueInitLine + BlueInitLineToLeftCage, FIELD_Y - BlueLeftHPWallToLeftCage_Y);
        BlueCenterCage = BlueLeftCage.plus(new Translation2d(0, -BlueCenterCageToLeftCage));
        BlueRightCage = BlueCenterCage.plus(new Translation2d(0, -BlueCenterCageToRightCage));

        BlueLeftStation = new Translation2d(Units.inchesToMeters(65)/2, FIELD_Y - Units.inchesToMeters(47)/2);
        BlueRightStation = new Translation2d(Units.inchesToMeters(65)/2, Units.inchesToMeters(47)/2);

        // Red Locations
        RedALMiddle = new Translation2d(FIELD_X - RedWallToReef_ALMiddle_X, RedLeftHPWallToReefALMiddle_Y);
        RedBCMiddle = RedALMiddle.plus(new Translation2d(RedReefSideLength, Rotation2d.fromDegrees(270 + 180)));
        RedDEMiddle = RedBCMiddle.plus(new Translation2d(RedReefSideLength, Rotation2d.fromDegrees(330 + 180)));
        RedFGMiddle = RedDEMiddle.plus(new Translation2d(RedReefSideLength, Rotation2d.fromDegrees(30 + 180)));
        RedHIMiddle = RedFGMiddle.plus(new Translation2d(RedReefSideLength, Rotation2d.fromDegrees(90 + 180)));
        RedJKMiddle = RedHIMiddle.plus(new Translation2d(RedReefSideLength, Rotation2d.fromDegrees(150 + 180)));

        Red_Coral_A = RedALMiddle.plus(new Translation2d(RedReefEdgeToBranch, Rotation2d.fromDegrees(270 + 180)));
        Red_Coral_B = RedBCMiddle.plus(new Translation2d(RedReefEdgeToBranch, Rotation2d.fromDegrees(90 + 180)));
        Red_Coral_C = RedBCMiddle.plus(new Translation2d(RedReefEdgeToBranch, Rotation2d.fromDegrees(330 + 180)));
        Red_Coral_D = RedDEMiddle.plus(new Translation2d(RedReefEdgeToBranch, Rotation2d.fromDegrees(150 + 180)));
        Red_Coral_E = RedDEMiddle.plus(new Translation2d(RedReefEdgeToBranch, Rotation2d.fromDegrees(30 + 180)));
        Red_Coral_F = RedFGMiddle.plus(new Translation2d(RedReefEdgeToBranch, Rotation2d.fromDegrees(210 + 180)));
        Red_Coral_G = RedFGMiddle.plus(new Translation2d(RedReefEdgeToBranch, Rotation2d.fromDegrees(90 + 180)));
        Red_Coral_H = RedHIMiddle.plus(new Translation2d(RedReefEdgeToBranch, Rotation2d.fromDegrees(270 + 180)));
        Red_Coral_I = RedHIMiddle.plus(new Translation2d(RedReefEdgeToBranch, Rotation2d.fromDegrees(150 + 180)));
        Red_Coral_J = RedJKMiddle.plus(new Translation2d(RedReefEdgeToBranch, Rotation2d.fromDegrees(330 + 180)));
        Red_Coral_K = RedJKMiddle.plus(new Translation2d(RedReefEdgeToBranch, Rotation2d.fromDegrees(210 + 180)));
        Red_Coral_L = RedALMiddle.plus(new Translation2d(RedReefEdgeToBranch, Rotation2d.fromDegrees(30 + 180)));

        Red_Algae_AB = RedALMiddle.plus(new Translation2d(RedReefSideLength / 2, Rotation2d.fromDegrees(270 + 180)));
        Red_Algae_CD = RedBCMiddle.plus(new Translation2d(RedReefSideLength / 2, Rotation2d.fromDegrees(330 + 180)));
        Red_Algae_EF = RedDEMiddle.plus(new Translation2d(RedReefSideLength / 2, Rotation2d.fromDegrees(30 + 180)));
        Red_Algae_GH = RedFGMiddle.plus(new Translation2d(RedReefSideLength / 2, Rotation2d.fromDegrees(90 + 180)));
        Red_Algae_IJ = RedHIMiddle.plus(new Translation2d(RedReefSideLength / 2, Rotation2d.fromDegrees(150 + 180)));
        Red_Algae_KL = RedJKMiddle.plus(new Translation2d(RedReefSideLength / 2, Rotation2d.fromDegrees(210 + 180)));

        RedCenterAlgae = Red_Algae_AB.plus(new Translation2d(RedCenterAlgaeToReefSide, 0));
        RedLeftAlgae = RedCenterAlgae.plus(new Translation2d(0, -RedCenterAlgaeToLeftAlgae));
        RedRightAlgae = RedCenterAlgae.plus(new Translation2d(0, RedCenterAlgaeToRightAlgae));

        RedInitLine = (FIELD_X / 2) + RedInitLineToLeftCage;

        RedProc = new Translation2d(RedInitLine + RedProcToInitLine, FIELD_Y);
        
        RedLeftCage = new Translation2d(RedInitLine - RedInitLineToLeftCage, RedLeftHPWallToLeftCage_Y);
        RedCenterCage = RedLeftCage.plus(new Translation2d(0, RedCenterCageToLeftCage));
        RedRightCage = RedCenterCage.plus(new Translation2d(0, RedCenterCageToRightCage));

        RedLeftStation = new Translation2d(FIELD_X - Units.inchesToMeters(65)/2, Units.inchesToMeters(47)/2);
        RedRightStation = new Translation2d(FIELD_X - Units.inchesToMeters(65)/2, FIELD_Y - Units.inchesToMeters(47)/2);

        // Field Dimensions
        FIELD_DIMENSIONS = new Translation2d(FIELD_X, FIELD_Y);

        // Field Layout
        fieldLayout = aprilTagFieldLayout;
    }

    private double convertJsonEntryToDouble(JSONObject map, String key) {
        return
            Units.inchesToMeters(
                Double.parseDouble(
                    map
                        .get(key)
                        .toString()));
    }

    // Override if field is measured otherwise
    public Translation2d getFieldDimensions() {
        return new Translation2d(FIELD_X, FIELD_Y);
    }

    public Pose2d getTagById(int id) {
        var t = fieldLayout.getTagPose(id);
        return t.isPresent() ? t.get().toPose2d() : null;
    }

    public void setVenue(Venue venue) {
        try {
            loadConstants(venue.filePath, venue.aprilTagFieldLayout);
        } catch (IOException | ParseException e) {
            e.printStackTrace();
        }
    }

    public enum Venue {
        Shop("Shop.json", AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded));

        public String filePath;
        public AprilTagFieldLayout aprilTagFieldLayout;

        private Venue(String p, AprilTagFieldLayout aprilTagFieldLayout) {
            this.filePath = p;
            this.aprilTagFieldLayout = aprilTagFieldLayout;
        }
    }
}
