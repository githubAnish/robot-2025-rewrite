package org.frogforce503.robot2025.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.frogforce503.lib.io.JoystickInputs;
import org.frogforce503.lib.util.ProximityService;
import org.frogforce503.robot2025.commands.algae_backup.BackupFromClosestReefAlgae;
import org.frogforce503.robot2025.commands.algae_intake_pluck.DriveToClosestReefAlgae;
import org.frogforce503.robot2025.commands.coral_intake_station.DriveToClosestStation;
import org.frogforce503.robot2025.fields.FieldInfo;
import org.frogforce503.robot2025.offsets.OffsetManager;
import org.frogforce503.robot2025.subsystems.drive.Drive;
import org.frogforce503.robot2025.subsystems.leds.Leds;
import org.frogforce503.robot2025.subsystems.superstructure.Superstructure;
import org.frogforce503.robot2025.subsystems.vision.Vision;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoIntakeCommands {
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

    // Joystick Inputs
    private final Supplier<JoystickInputs> driverInputs;

    // Auto-driving enable
    private BooleanSupplier autoDrivingEnabled;

    public AutoIntakeCommands(
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
        this.autoDrivingEnabled = autoDrivingEnabled;
    }

    public Command coralAutoIntake() {
        return
            Commands.deferredProxy(() -> new DriveToClosestStation(drive, field, proximityService, driverInputs.get()))
                .onlyIf(autoDrivingEnabled)
                .alongWith(superstructure.intakeCoral());
    }

    public Command algaeAutoHighPluck() {
        return
            superstructure
                .pluckHighAlgae()
                .andThen(
                    Commands.deferredProxy(() -> new DriveToClosestReefAlgae(drive, field, proximityService))
                        .onlyIf(autoDrivingEnabled));
    }

    public Command algaeAutoLowPluck() {
        return
            superstructure
                .pluckLowAlgae()
                .andThen(
                    Commands.deferredProxy(() -> new DriveToClosestReefAlgae(drive, field, proximityService))
                        .onlyIf(autoDrivingEnabled));
    }

    public Command algaeBackup() {
        return
            Commands.deferredProxy(() -> new BackupFromClosestReefAlgae(drive, field, proximityService))
                .onlyIf(autoDrivingEnabled)
                .andThen(superstructure.holdAlgaeFromPluck());
    }
}