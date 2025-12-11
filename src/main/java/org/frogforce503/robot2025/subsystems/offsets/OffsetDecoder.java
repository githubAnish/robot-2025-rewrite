package org.frogforce503.robot2025.subsystems.offsets;

import java.util.HashMap;
import java.util.Map;

import org.frogforce503.lib.util.FieldConstantsUtil;

import lombok.Getter;

public class OffsetDecoder {
    private final String LEFT_RED_AB = "LEFT_RED_AB";
    private final String LEFT_RED_CD = "LEFT_RED_CD";
    private final String LEFT_RED_EF = "LEFT_RED_EF";
    private final String LEFT_RED_GH = "LEFT_RED_GH";
    private final String LEFT_RED_IJ = "LEFT_RED_IJ";
    private final String LEFT_RED_KL = "LEFT_RED_KL";
    private final String LEFT_BLUE_AB = "LEFT_BLUE_AB";
    private final String LEFT_BLUE_CD = "LEFT_BLUE_CD";
    private final String LEFT_BLUE_EF = "LEFT_BLUE_EF";
    private final String LEFT_BLUE_GH = "LEFT_BLUE_GH";
    private final String LEFT_BLUE_IJ = "LEFT_BLUE_IJ";
    private final String LEFT_BLUE_KL = "LEFT_BLUE_KL";
    private final String RIGHT_RED_AB = "RIGHT_RED_AB";
    private final String RIGHT_RED_CD = "RIGHT_RED_CD";
    private final String RIGHT_RED_EF = "RIGHT_RED_EF";
    private final String RIGHT_RED_GH = "RIGHT_RED_GH";
    private final String RIGHT_RED_IJ = "RIGHT_RED_IJ";
    private final String RIGHT_RED_KL = "RIGHT_RED_KL";
    private final String RIGHT_BLUE_AB = "RIGHT_BLUE_AB";
    private final String RIGHT_BLUE_CD = "RIGHT_BLUE_CD";
    private final String RIGHT_BLUE_EF = "RIGHT_BLUE_EF";
    private final String RIGHT_BLUE_GH = "RIGHT_BLUE_GH";
    private final String RIGHT_BLUE_IJ = "RIGHT_BLUE_IJ";
    private final String RIGHT_BLUE_KL = "RIGHT_BLUE_KL";

    @Getter private final Map<String, Offset> mapper;

    public OffsetDecoder() {
        mapper =
            new HashMap<>() {{
                put(LEFT_RED_AB, jsonEntryToOffset(LEFT_RED_AB));
                put(LEFT_RED_CD, jsonEntryToOffset(LEFT_RED_CD));
                put(LEFT_RED_EF, jsonEntryToOffset(LEFT_RED_EF));
                put(LEFT_RED_GH, jsonEntryToOffset(LEFT_RED_GH));
                put(LEFT_RED_IJ, jsonEntryToOffset(LEFT_RED_IJ));
                put(LEFT_RED_KL, jsonEntryToOffset(LEFT_RED_KL));
                put(LEFT_BLUE_AB, jsonEntryToOffset(LEFT_BLUE_AB));
                put(LEFT_BLUE_CD, jsonEntryToOffset(LEFT_BLUE_CD));
                put(LEFT_BLUE_EF, jsonEntryToOffset(LEFT_BLUE_EF));
                put(LEFT_BLUE_GH, jsonEntryToOffset(LEFT_BLUE_GH));
                put(LEFT_BLUE_IJ, jsonEntryToOffset(LEFT_BLUE_IJ));
                put(LEFT_BLUE_KL, jsonEntryToOffset(LEFT_BLUE_KL));
                put(RIGHT_RED_AB, jsonEntryToOffset(RIGHT_RED_AB));
                put(RIGHT_RED_CD, jsonEntryToOffset(RIGHT_RED_CD));
                put(RIGHT_RED_EF, jsonEntryToOffset(RIGHT_RED_EF));
                put(RIGHT_RED_GH, jsonEntryToOffset(RIGHT_RED_GH));
                put(RIGHT_RED_IJ, jsonEntryToOffset(RIGHT_RED_IJ));
                put(RIGHT_RED_KL, jsonEntryToOffset(RIGHT_RED_KL));
                put(RIGHT_BLUE_AB, jsonEntryToOffset(RIGHT_BLUE_AB));
                put(RIGHT_BLUE_CD, jsonEntryToOffset(RIGHT_BLUE_CD));
                put(RIGHT_BLUE_EF, jsonEntryToOffset(RIGHT_BLUE_EF));
                put(RIGHT_BLUE_GH, jsonEntryToOffset(RIGHT_BLUE_GH));
                put(RIGHT_BLUE_IJ, jsonEntryToOffset(RIGHT_BLUE_IJ));
                put(RIGHT_BLUE_KL, jsonEntryToOffset(RIGHT_BLUE_KL));
            }};
    }

    private Offset jsonEntryToOffset(String key) {
        String[] jsonEntryValue = FieldConstantsUtil.jsonEntryToString(key).split(", ");

        return
            new Offset(
                Double.parseDouble(jsonEntryValue[0]),
                Double.parseDouble(jsonEntryValue[1]))
            .inchesToMeters();
    }
}