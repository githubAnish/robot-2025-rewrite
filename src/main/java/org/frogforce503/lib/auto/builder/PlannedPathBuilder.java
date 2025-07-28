package org.frogforce503.lib.auto.builder;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.frogforce503.lib.auto.trajectory.path.PlannedPath;
import org.frogforce503.lib.auto.trajectory.path.Waypoint;
import org.frogforce503.robot2025.Robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;

/** Static wrapper for the {@link CustomTrajectoryGenerator}. */
public class PlannedPathBuilder {
    public PlannedPath generate(TrajectoryConfig config, List<Waypoint> waypoints) {
        CustomTrajectoryGenerator generator = new CustomTrajectoryGenerator();

        try {
            generator.generate(config, waypoints);
        } catch (TrajectoryGenerationException exception) {
            System.out.print("TRAJECTORY GENERATION FAILED");
            exception.printStackTrace();
            return null;
        }
        
        return generator.getPlannedPath(waypoints);
    }

    public PlannedPath generate(double vMax, double aMax, double vInitial, double vFinal, List<Waypoint> waypoints) {
        return
            generate(
                makeConfig(vMax, aMax, vInitial, vFinal),
                waypoints);
    }

    public PlannedPath generate(double vMax, double aMax, double vInitial, double vFinal, Waypoint... waypoints) {
        return generate(vMax, aMax, vInitial, vFinal, Arrays.asList(waypoints));
    }

    public PlannedPath regenerate(double vMax, double aMax, double vi, double vf, PlannedPath existing) {
        return generate(vMax, aMax, vi, vf, existing.getWaypoints());
    }

    public PlannedPath reversedOf(PlannedPath other, double vMax, double aMax, double vi, double vf) {
        List<Waypoint> waypoints = other.getWaypoints();
        List<Waypoint> reversedWaypoints = new ArrayList<>();

        for (int i = waypoints.size() - 1; i >= 0; i--) {
            Waypoint w = waypoints.get(i);
            
            if (w.getDriveRotation().isPresent()) {
                w =
                    new Waypoint(
                        w.getTranslation(),
                        w.getDriveRotation()
                            .get()
                            .plus(Rotation2d.kPi),
                        w.getHolonomicRotation().isEmpty()
                            ? null :
                            w.getHolonomicRotation().get());
            }

            reversedWaypoints.add(w);
        }

        return generate(vMax, aMax, vi, vf, reversedWaypoints);
    }

    public PlannedPath reversedOf(PlannedPath other, double vMax, double aMax) {
        return reversedOf(other, vMax, aMax, 0.0, 0.0);
    }

    public TrajectoryConfig makeConfig(double vMax, double aMax, double vInitial, double vFinal) {
        return
            new TrajectoryConfig(vMax, aMax)
                .setKinematics(Robot.bot.kinematics)
                .setStartVelocity(vInitial)
                .setEndVelocity(vFinal);
    }

    // private final List<TrajectoryConstraint> crescendoConstraints() {
    //     TrajectoryConstraint pickupConstraint = new RectangularRegionConstraint(FieldConfig.getInstance().NOTE_D.plus(new Translation2d(-1, -1)), FieldConfig.getInstance().NOTE_H.plus(new Translation2d(1, 1)), new MaxVelocityConstraint(2.0));
    //     return List.of(pickupConstraint);
    // }

    // make into method
    // private List<TrajectoryConstraint> chargedUpTrajectoryConstraints = List.of(
    //             // Cable bump
    //             new RectangularRegionConstraint(
    //                 new Translation2d(Community.chargingStationInnerX, Community.rightY),
    //                 new Translation2d(Community.chargingStationOuterX, Community.chargingStationRightY),
    //                 new MaxVelocityConstraint(cableBumpMaxVelocity)),

    //             // Charging station
    //             new RectangularRegionConstraint(
    //                 new Translation2d(
    //                     Community.chargingStationInnerX - 0.8, Community.chargingStationRightY),
    //                 new Translation2d(
    //                     Community.chargingStationOuterX + 0.8, Community.chargingStationLeftY),
    //                 new MaxVelocityConstraint(chargingStationMaxVelocity)));
}
