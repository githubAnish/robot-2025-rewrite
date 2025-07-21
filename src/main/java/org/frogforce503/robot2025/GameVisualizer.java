package org.frogforce503.robot2025;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.frogforce503.robot2025.fields.FieldInfo;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/** Simulates the field, including interaction with & movement of game elements. Implement physics simulation here. */
public class GameVisualizer {
    private final FieldInfo field;

    private List<Pose3d> algaePoses = new ArrayList<>();
    private List<Pose3d> floorCoralPoses = new ArrayList<>();
    
    public GameVisualizer(FieldInfo field) {
        this.field = field;
    }

    public void update(Pose2d robotPose, double elevatorPosition, double armPosition, double wristPosition, boolean hasAlgaeInClaw) {
        // Pluck Algae from Reef
        if (!hasAlgaeInClaw) {
            for (int index = 0; index < algaePoses.size(); index++) {
                Translation3d algae = algaePoses.get(index).getTranslation();
                
                Translation2d displacement = algae.toTranslation2d().minus(robotPose.getTranslation());

                double elevatorSimTol = 12; // in inches

                boolean algaeCloseEnough =
                    robotPose
                        .getTranslation()
                        .getDistance(
                            algae.toTranslation2d()) < Units.inchesToMeters(20);
                            
                boolean algaeFacingCloseEnough =
                    Math.abs(
                        displacement
                            .getAngle()
                            .minus(
                                robotPose.getRotation())
                        .getRadians()) < Units.degreesToRadians(90);

                boolean elevatorAtHeight =
                    MathUtil.isNear(
                        elevatorPosition,
                        Units.metersToInches(algae.getZ()),
                        elevatorSimTol);            

                if (algaeCloseEnough && algaeFacingCloseEnough && elevatorAtHeight) {
                    // simulationHasAlgae.set(true);
                    algaePoses.remove(index);
                    break;
                }
            }
        }

        Logger.recordOutput("Simulation/Algae", algaePoses.toArray(Pose3d[]::new));
        Logger.recordOutput("Simulation/Floor Coral", floorCoralPoses.toArray(Pose3d[]::new));
    }

    public void addAlgae(AlgaeInfo... algaeData) {
        Arrays
            .stream(algaeData)
            .forEach(
                algaeInfo -> {
                    Translation2d loc = algaeInfo.loc(); // 2D location of algae
                    double height = algaeInfo.height().height; // Height of algae
                    Rotation2d rot = Rotation2d.fromDegrees(algaeInfo.rotDegrees()); // Rotation of algae relative to FieldConfig algae location

                    Translation2d newLoc =
                        loc.minus(
                            new Translation2d(Units.inchesToMeters(6.5), rot));

                    algaePoses.add(
                        new Pose3d(
                            new Translation3d(newLoc.getX(), newLoc.getY(), height),
                            new Rotation3d()));
            });
    }

    public void addFloorAlgae(Translation2d... algaePoses) {
        Arrays
            .stream(algaePoses)
            .forEach(
                loc ->
                    addAlgae(
                        new AlgaeInfo(
                            loc.plus(
                                new Translation2d(Units.inchesToMeters(6.5), 0.0)),
                            SimHeights.kAlgaePresetFloor,
                            0.0
            )));
    }

    public void addFloorCoral(Translation2d... coralPoses) {
        Arrays
            .stream(coralPoses)
            .forEach(
                coralInfo -> {
                    Translation2d loc = coralInfo; // 2D location of floor coral
                    double height = SimHeights.kCoralPresetFloor.height; // Height of floor coral

                    floorCoralPoses.add(
                        new Pose3d(
                            new Translation3d(loc.getX(), loc.getY(), height),
                            new Rotation3d(0, Math.PI/2, 0)));
            });
    }

    public void clearAllPieces() {
        algaePoses.clear();
        floorCoralPoses.clear();
    }

    public void reset() {
        clearAllPieces();

        addAlgae(
            new AlgaeInfo(field.getConfiguration().Blue_Algae_AB, SimHeights.kAlgaeL3, 180.0),
            new AlgaeInfo(field.getConfiguration().Blue_Algae_CD, SimHeights.kAlgaeL2, 230.0),
            new AlgaeInfo(field.getConfiguration().Blue_Algae_EF, SimHeights.kAlgaeL3, 290.0),
            new AlgaeInfo(field.getConfiguration().Blue_Algae_GH, SimHeights.kAlgaeL2, 0.0),
            new AlgaeInfo(field.getConfiguration().Blue_Algae_IJ, SimHeights.kAlgaeL3, 70.0),
            new AlgaeInfo(field.getConfiguration().Blue_Algae_KL, SimHeights.kAlgaeL2, 130.0));

        addFloorAlgae(
            field.getConfiguration().BlueLeftAlgae,
            field.getConfiguration().BlueCenterAlgae,
            field.getConfiguration().BlueRightAlgae);

        addFloorCoral(
            field.getConfiguration().BlueLeftAlgae,
            field.getConfiguration().BlueCenterAlgae,
            field.getConfiguration().BlueRightAlgae);
    }

    public void setupAuto() {
        reset();
    }
   
    private enum SimHeights {
        kCoralPresetFloor(Units.inchesToMeters(6)),
        kAlgaePresetFloor(Units.inchesToMeters(19.5)),

        kAlgaeL2(Units.inchesToMeters(35)),
        kAlgaeL3(Units.inchesToMeters(51)),

        kCoralL1(Units.inchesToMeters(0)),
        kCoralL2(Units.inchesToMeters(0)),
        kCoralL3(Units.inchesToMeters(0)),
        kCoralL4(Units.inchesToMeters(0));

        public double height;

        private SimHeights(double height) {
            this.height = height;
        }
    }

    private record AlgaeInfo(Translation2d loc, SimHeights height, double rotDegrees) {}
}