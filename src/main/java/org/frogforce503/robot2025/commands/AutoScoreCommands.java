package org.frogforce503.robot2025.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.frogforce503.lib.io.JoystickInputs;
import org.frogforce503.lib.reefscape.Branch;
import org.frogforce503.lib.reefscape.PrescoreBoundary;
import org.frogforce503.lib.util.ProximityUtil;
import org.frogforce503.robot2025.FieldInfo;
import org.frogforce503.robot2025.commands.drive.DriveToBarge;
import org.frogforce503.robot2025.commands.drive.DriveToProcessor;
import org.frogforce503.robot2025.subsystems.drive.Drive;
import org.frogforce503.robot2025.subsystems.leds.Leds;
import org.frogforce503.robot2025.subsystems.offsets.OffsetManager;
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

    // Reef Boundary (Used for L2 - L4 coral scoring)
    private final PrescoreBoundary prescoreBoundaryBuilder;

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
        Supplier<JoystickInputs> driverInputs,
        BooleanSupplier autoDrivingEnabled
    ) {
        this.drive = drive;
        this.vision = vision;
        this.superstructure = superstructure;
        this.leds = leds;
        this.field = field;
        this.offsetManager = offsetManager;
        this.driverInputs = driverInputs;
        this.prescoreBoundaryBuilder = new PrescoreBoundary(field, drive::getCurrentPose);
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
                        drive::getCurrentPose,
                        branchSupplier
                            .get()
                            .getTarget(drive, field, offsetManager.getOffsetData()),
                        prescoreBoundaryBuilder::insideBoundary,
                        autoDrivingEnabled));
    }

    public Command coralAutoScore() {
        return
            coralAutoScore(
                superstructure::getCurrentBranch);
    }

    public Command coralAutoScore(Branch branch, Mode mode) {
        return
            Commands.sequence(
                Commands.runOnce(() -> superstructure.setCurrentMode(mode))
                    .ignoringDisable(true),
                coralAutoScore(() -> branch));
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
                        drive::getCurrentPose,
                        ProximityUtil
                            .getClosestReefSide(drive, field)
                            .getTarget(field),
                        prescoreBoundaryBuilder::insideBoundary,
                        autoDrivingEnabled));
    }

    public Command processorAutoScore() {
        return
            Commands.deferredProxy(() -> new DriveToProcessor(drive, field))
                .onlyIf(autoDrivingEnabled)
                .andThen(superstructure.scoreProcessor());
    }

    public Command bargeAutoScore() {
        return
            Commands.deferredProxy(() -> new DriveToBarge(drive, field, driverInputs.get()))
                .onlyIf(autoDrivingEnabled)
                .andThen(superstructure.scoreBarge());
    }
}