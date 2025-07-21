package org.frogforce503.lib.util;

import java.util.Arrays;
import java.util.function.Function;
import java.util.function.Supplier;

import org.frogforce503.robot2025.commands.algae_score_barge.Barge;
import org.frogforce503.robot2025.commands.algae_score_processor.Processor;
import org.frogforce503.robot2025.commands.cage.Cage;
import org.frogforce503.robot2025.commands.coral_intake_station.Station;
import org.frogforce503.robot2025.commands.coral_score_reef.ReefSide;
import org.frogforce503.robot2025.fields.FieldInfo;
import org.frogforce503.robot2025.subsystems.drive.Drive;

import edu.wpi.first.math.geometry.Pose2d;

public class ProximityService {
    private final Drive drive;
    private final FieldInfo field;

    public ProximityService(Drive drive, FieldInfo field) {
        this.drive = drive;
        this.field = field;
    }

    private int poseDistanceComparator(Pose2d target, Pose2d first, Pose2d second) {
        return
            Double.compare(
                first
                    .minus(target)
                    .getTranslation()
                    .getNorm(),
                second
                    .minus(target)
                    .getTranslation()
                    .getNorm());
    }

    public <T> T closestTo(Supplier<Pose2d> target, Function<T, Pose2d> poseExtractor, T... options) {
        return
            Arrays
                .stream(options)
                .min(
                    (option1, option2) ->
                        poseDistanceComparator(
                            target.get(),
                            poseExtractor.apply(option1),
                            poseExtractor.apply(option2)))
                .orElse(options[0]); // Return the first option if none are closest
    }

    public <T> T chassisClosestTo(Function<T, Pose2d> poseExtractor, T... options) {
        return closestTo(drive::getCurrentPose, poseExtractor, options);
    }

    private Pose2d chassisClosestTo(Pose2d... options) {
        return chassisClosestTo(Function.identity(), options);
    }

    public ReefSide getClosestReefSide() {
        return
            chassisClosestTo(
                side -> side.getTarget(field).get(),
                ReefSide.values());
    }

    public Station getClosestStation() {
        return
            chassisClosestTo(
                station -> station.getTarget(field).get(),
                Station.values());
    }

    public Processor getClosestProcessor() {
        return
            chassisClosestTo(
                processor -> processor.getTarget(field).get(),
                Processor.values());
    }

    public Barge getClosestBarge() {
        return
            chassisClosestTo(
                barge -> barge.getTarget(drive, field).get(),
                Barge.values());
    }

    public Cage getClosestCage() {
        return
            chassisClosestTo(
                cage -> cage.getTarget(field).get(),
                Cage.values());
    }
}