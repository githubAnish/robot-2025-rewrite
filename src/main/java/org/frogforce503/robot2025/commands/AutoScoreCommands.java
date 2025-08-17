package org.frogforce503.robot2025.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.frogforce503.lib.io.JoystickInputs;
import org.frogforce503.lib.util.ProximityService;
import org.frogforce503.robot2025.commands.algae_score_barge.DriveToClosestBarge;
import org.frogforce503.robot2025.commands.algae_score_processor.DriveToClosestProcessor;
import org.frogforce503.robot2025.commands.coral_score_reef.AutoCoralScore;
import org.frogforce503.robot2025.commands.coral_score_reef.Branch;
import org.frogforce503.robot2025.commands.coral_score_reef.PrescoreBoundaryBuilder;
import org.frogforce503.robot2025.fields.FieldInfo;
import org.frogforce503.robot2025.offsets.OffsetManager;
import org.frogforce503.robot2025.subsystems.drive.Drive;
import org.frogforce503.robot2025.subsystems.leds.Leds;
import org.frogforce503.robot2025.subsystems.superstructure.Superstructure;
import org.frogforce503.robot2025.subsystems.superstructure.Superstructure.Mode;
import org.frogforce503.robot2025.subsystems.vision.Vision;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoScoreCommands {
    // Subsystems
    private final Drive drive;
    private final Vision vision;
    private final Superstructure superstructure;
    private final Leds leds;

    // Field
    private final FieldInfo field;

    // Offset Manager
    private final OffsetManager offsetManager;

    // Proximity Service
    private final ProximityService proximityService;

    // Reef Boundary (Used for L2 - L4 coral scoring)
    private final PrescoreBoundaryBuilder prescoreBoundaryBuilder;

    // Joystick Inputs
    private final Supplier<JoystickInputs> driverInputs;

    // Auto-driving enable
    private BooleanSupplier autoDrivingEnabled;

    public AutoScoreCommands(
        Drive drive,
        Vision vision,
        Superstructure superstructure,
        Leds leds,
        FieldInfo field,
        OffsetManager offsetManager,
        ProximityService proximityService,
        Supplier<JoystickInputs> driverInputs,
        BooleanSupplier autoDrivingEnabled
    ) {
        this.drive = drive;
        this.vision = vision;
        this.superstructure = superstructure;
        this.leds = leds;
        this.field = field;
        this.offsetManager = offsetManager;
        this.proximityService = proximityService;
        this.driverInputs = driverInputs;
        this.prescoreBoundaryBuilder = new PrescoreBoundaryBuilder(field, drive::getCurrentPose);
        this.autoDrivingEnabled = autoDrivingEnabled;
    }

    public Command coralAutoScore(Supplier<Branch> branchSupplier) {
        return
            Commands.deferredProxy(
                () ->
                    new AutoCoralScore(
                        drive,
                        field,
                        superstructure,
                        driverInputs.get(),
                        offsetManager,
                        proximityService,
                        vision.getReefSpecializedPose(leds::usingGlobalPose),
                        branchSupplier,
                        prescoreBoundaryBuilder::insideBoundary,
                        autoDrivingEnabled));
    }

    public Command coralAutoScore() {
        return
            coralAutoScore(
                superstructure::getCurrentBranch);
    }

    public Command coralAutoScore(Supplier<Branch> branchSupplier, Supplier<Mode> superstructureModeSupplier) {
        return
            Commands.sequence(
                Commands.runOnce(() -> superstructure.setCurrentMode(superstructureModeSupplier.get()))
                    .ignoringDisable(true),
                coralAutoScore(branchSupplier));
    }

    public Command coralAutoScoreL1() {
        return
            Commands.deferredProxy(
                () ->
                    new AutoCoralScore(
                        drive,
                        field,
                        superstructure,
                        driverInputs.get(),
                        proximityService,
                        vision.getReefSpecializedPose(leds::usingGlobalPose),
                        proximityService::getClosestReefSide,
                        prescoreBoundaryBuilder::insideBoundary,
                        autoDrivingEnabled));
    }

    public Command processorAutoScore() {
        return
            Commands.deferredProxy(() -> new DriveToClosestProcessor(drive, field, proximityService))
                .onlyIf(autoDrivingEnabled)
                .andThen(superstructure.scoreProcessor());
    }

    public Command bargeAutoScore() {
        return
            Commands.deferredProxy(() -> new DriveToClosestBarge(drive, field, proximityService, driverInputs.get()))
                .onlyIf(autoDrivingEnabled)
                .andThen(superstructure.scoreBarge());
    }
}