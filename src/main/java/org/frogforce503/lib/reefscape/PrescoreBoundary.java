package org.frogforce503.lib.reefscape;

import java.util.stream.Collectors;

import org.frogforce503.lib.math.GeomUtil;
import org.frogforce503.lib.math.Polygon2d;
import org.frogforce503.robot2025.FieldInfo;
import org.frogforce503.robot2025.subsystems.drive.Drive;

import edu.wpi.first.math.util.Units;

/** Builds a hexagonal boundary surrounding the reef that signals the robot when to switch from prescoring to scoring mode. */
public class PrescoreBoundary {
    private final Drive drive;
    private final FieldInfo field;

    /** Distance from the colored tape (surrounding the reef) that the robot needs to switch from prescoring to scoring position. */
    private final double distanceFromTape = Units.inchesToMeters(18);

    public PrescoreBoundary(Drive drive, FieldInfo field) {
        this.drive = drive;
        this.field = field;
    }

    public Polygon2d get() {
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
            get()
                .contains(
                    drive
                        .getCurrentPose()
                        .getTranslation());
    }

    public void drawBoundary() {
        field
            .getObject("Prescore Boundary")
            .setPoses(
                get()
                    .getVertices()
                    .stream()
                    .map(GeomUtil::toPose2d)
                    .collect(Collectors.toList()));
    }
}