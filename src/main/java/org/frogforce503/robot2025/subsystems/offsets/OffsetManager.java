package org.frogforce503.robot2025.subsystems.offsets;

import java.util.Map;

import org.frogforce503.lib.logging.LoggedTracer;
import org.frogforce503.lib.subsystem.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import lombok.Getter;

public class OffsetManager extends VirtualSubsystem {
    private final OffsetsIO io;
    private final OffsetsIOInputsAutoLogged inputs = new OffsetsIOInputsAutoLogged();

    @Getter private final Map<String, Offset> offsetData;

    public OffsetManager(OffsetsIO io) {
        this.io = io;
        this.offsetData = new OffsetDecoder().getMapper();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Offsets", inputs);

        boolean tuning = inputs.tuning;
        String branch = inputs.branch;
        String direction = inputs.direction;
        double value = inputs.value;

        if (tuning &&
            branch != null &&
            direction != null &&
            value != 0.0
        ) {
            offsetData.put(
                branch,
                getNewOffset(
                    offsetData.get(branch).horizontal(),
                    offsetData.get(branch).vertical(),
                    direction,
                    Units.inchesToMeters(value)));

            // Reset value to prevent continually applying nonzero offset
            io.setValue(0.0);
        }

        // Record cycle time
        LoggedTracer.record("OffsetManager");
    }

    /**
     * Method to log all offsets to NT.
     * Publishes an array that consumes a lot of memory,
     * so call this method only when necessary (e.g. to view that offsets are being recorded correctly.)
     */
    public void logOffsetData() {
        Logger.recordOutput("Offset Data",
            offsetData
                .values()
                .stream()
                .map(offset -> offset.metersToInches())
                .toArray(Offset[]::new));
    }

    private Offset getNewOffset(double oldHorizontal, double oldVertical, String direction, double value) {
        return
            new Offset(
                oldHorizontal + handleValueBasedOnHorizontalDirection(direction, value),
                oldVertical + handleValueBasedOnVerticalDirection(direction, value));
    }

    private double handleValueBasedOnHorizontalDirection(String wantedDirection, double value) {
        if (Direction.LEFT.equals(wantedDirection)) {
            return value;
        } else if (Direction.RIGHT.equals(wantedDirection)) {
            return -value;
        } else {
            return 0.0;
        }
    }

    private double handleValueBasedOnVerticalDirection(String wantedDirection, double value) {
        if (Direction.FORWARD.equals(wantedDirection)) {
            return -value;
        } else if (Direction.BACKWARD.equals(wantedDirection)) {
            return value;
        } else {
            return 0.0;
        }
    }
}