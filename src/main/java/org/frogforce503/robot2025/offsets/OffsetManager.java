package org.frogforce503.robot2025.offsets;

import java.util.Map;

import org.frogforce503.lib.subsystem.VirtualSubsystem;
import org.frogforce503.lib.util.LoggedTracer;
import org.frogforce503.robot2025.fields.FieldConfig.Venue;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import lombok.Getter;

public class OffsetManager extends VirtualSubsystem {
    private final OffsetsIO io;
    private final OffsetsIOInputsAutoLogged inputs = new OffsetsIOInputsAutoLogged();

    @Getter private Map<String, Offset> offsetData;

    public OffsetManager(String fileName, OffsetsIO io) {
        this.io = io;
        
        this.offsetData = new OffsetDecoder(fileName).getMapper();
    }

    /**
     * <p> Gets the offsets file based on the selected field config venue. </p>
     * <p> Add the word "Offsets" between the field config file name and .json to get the offsets file name. </p>
     * <p> For example, if the field config file path = "Shop.json," then the offsets file path should be "ShopOffsets.json". </p>
     */
    public OffsetManager(Venue fieldconfigVenue, OffsetsIO io) {
        this(fieldconfigVenue.filePath.split(".json")[0] + "Offsets.json", io);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Offsets", inputs);

        if (inputs.data.tuning() &&
            inputs.data.branch() != null &&
            inputs.data.direction() != null &&
            inputs.data.value() != 0.0
        ) {
            offsetData.put(
                inputs.data.branch(),
                getNewOffset(
                    offsetData
                        .get(inputs.data.branch())
                        .horizontal(),
                    offsetData
                        .get(inputs.data.branch())
                        .vertical(),
                    inputs.data.direction(),
                    Units.inchesToMeters(inputs.data.value())));

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
        return new Offset(
            oldHorizontal + handleValueBasedOnHorizontalDirection(direction, value),
            oldVertical + handleValueBasedOnVerticalDirection(direction, value));
    }

    private double handleValueBasedOnHorizontalDirection(String wantedDirection, double value) {
        if (Direction.LEFT.equalsTo(wantedDirection)) {
            return value;
        } else if (Direction.RIGHT.equalsTo(wantedDirection)) {
            return -value;
        } else {
            return 0.0;
        }
    }

    private double handleValueBasedOnVerticalDirection(String wantedDirection, double value) {
        if (Direction.FORWARD.equalsTo(wantedDirection)) {
            return -value;
        } else if (Direction.BACKWARD.equalsTo(wantedDirection)) {
            return value;
        } else {
            return 0.0;
        }
    }
}