package org.frogforce503.robot2025.offsets;

import java.io.FileReader;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

import org.frogforce503.lib.util.ErrorUtil;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import edu.wpi.first.wpilibj.Filesystem;

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

    private JSONObject rawOffsets;

    public OffsetDecoder(String fileName) {
        try {
            this.rawOffsets =
                (JSONObject) new JSONParser()
                    .parse(
                        new FileReader(
                            Filesystem.getDeployDirectory().getAbsolutePath() + "/offsets/" + fileName));
        } catch (IOException | ParseException e) {
            System.out.println("Error reading offsets file: " + fileName  + ErrorUtil.attachJavaClassName(OffsetDecoder.class));
            e.printStackTrace();
        }
    }

    public Map<String, Offset> getMapper() {
        return new HashMap<>() {{
            put(LEFT_RED_AB, convertJsonEntryToOffset(rawOffsets, LEFT_RED_AB));
            put(LEFT_RED_CD, convertJsonEntryToOffset(rawOffsets, LEFT_RED_CD));
            put(LEFT_RED_EF, convertJsonEntryToOffset(rawOffsets, LEFT_RED_EF));
            put(LEFT_RED_GH, convertJsonEntryToOffset(rawOffsets, LEFT_RED_GH));
            put(LEFT_RED_IJ, convertJsonEntryToOffset(rawOffsets, LEFT_RED_IJ));
            put(LEFT_RED_KL, convertJsonEntryToOffset(rawOffsets, LEFT_RED_KL));
            put(LEFT_BLUE_AB, convertJsonEntryToOffset(rawOffsets, LEFT_BLUE_AB));
            put(LEFT_BLUE_CD, convertJsonEntryToOffset(rawOffsets, LEFT_BLUE_CD));
            put(LEFT_BLUE_EF, convertJsonEntryToOffset(rawOffsets, LEFT_BLUE_EF));
            put(LEFT_BLUE_GH, convertJsonEntryToOffset(rawOffsets, LEFT_BLUE_GH));
            put(LEFT_BLUE_IJ, convertJsonEntryToOffset(rawOffsets, LEFT_BLUE_IJ));
            put(LEFT_BLUE_KL, convertJsonEntryToOffset(rawOffsets, LEFT_BLUE_KL));
            put(RIGHT_RED_AB, convertJsonEntryToOffset(rawOffsets, RIGHT_RED_AB));
            put(RIGHT_RED_CD, convertJsonEntryToOffset(rawOffsets, RIGHT_RED_CD));
            put(RIGHT_RED_EF, convertJsonEntryToOffset(rawOffsets, RIGHT_RED_EF));
            put(RIGHT_RED_GH, convertJsonEntryToOffset(rawOffsets, RIGHT_RED_GH));
            put(RIGHT_RED_IJ, convertJsonEntryToOffset(rawOffsets, RIGHT_RED_IJ));
            put(RIGHT_RED_KL, convertJsonEntryToOffset(rawOffsets, RIGHT_RED_KL));
            put(RIGHT_BLUE_AB, convertJsonEntryToOffset(rawOffsets, RIGHT_BLUE_AB));
            put(RIGHT_BLUE_CD, convertJsonEntryToOffset(rawOffsets, RIGHT_BLUE_CD));
            put(RIGHT_BLUE_EF, convertJsonEntryToOffset(rawOffsets, RIGHT_BLUE_EF));
            put(RIGHT_BLUE_GH, convertJsonEntryToOffset(rawOffsets, RIGHT_BLUE_GH));
            put(RIGHT_BLUE_IJ, convertJsonEntryToOffset(rawOffsets, RIGHT_BLUE_IJ));
            put(RIGHT_BLUE_KL, convertJsonEntryToOffset(rawOffsets, RIGHT_BLUE_KL));
        }};
    }

    private Offset convertJsonEntryToOffset(JSONObject map, String key) {
        String[] jsonEntryValue =
            map
                .get(key)
                .toString()
                .split(", ");

        return
            new Offset(
                Double.parseDouble(jsonEntryValue[0]),
                Double.parseDouble(jsonEntryValue[1]))
            .inchesToMeters();
    }
}