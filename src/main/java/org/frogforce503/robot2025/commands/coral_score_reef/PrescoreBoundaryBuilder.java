package org.frogforce503.robot2025.commands.coral_score_reef;

import java.util.function.Supplier;

import org.frogforce503.lib.math.GeomUtil;
import org.frogforce503.lib.math.Polygon2d;
import org.frogforce503.robot2025.fields.FieldInfo;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;

/** Builds a hexagonal boundary surrounding the reef that signals the robot when to switch from prescoring to scoring mode. */
public class PrescoreBoundaryBuilder {
    private final FieldInfo field;
    private final Supplier<Pose2d> robotPose;

    /** Distance from the colored tape (surrounding the reef) that the robot needs to switch from prescoring to scoring position. */
    private final double distanceFromTape = Units.inchesToMeters(18);

    public PrescoreBoundaryBuilder(FieldInfo field, Supplier<Pose2d> robotPose) {
        this.field = field;
        this.robotPose = robotPose;
    }

    public Polygon2d getBoundary() {
        Polygon2d temp =
            field.onRedAlliance()
                ? field.getRedReef()
                : field.getBlueReef();

        return
            temp
                .withNewRadius(
                    temp.getRadius() +
                    distanceFromTape);
    }

    public boolean insideBoundary() {
        return
            getBoundary()
                .contains(
                    robotPose
                        .get()
                        .getTranslation());
    }

    public void drawBoundary() {
        field
            .getObject("Prescore Boundary")
            .setPoses(
                getBoundary()
                    .getVertices()
                    .stream()
                    .map(v -> GeomUtil.toPose2d(v))
                    .toList());
    }
}