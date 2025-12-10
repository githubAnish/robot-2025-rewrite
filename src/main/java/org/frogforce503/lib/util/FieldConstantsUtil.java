package org.frogforce503.lib.util;

import java.io.FileReader;
import java.io.IOException;

import org.frogforce503.robot2025.Constants;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;

public final class FieldConstantsUtil {
    private static JSONObject fieldJson;

    static {
        try {
            fieldJson =
                (JSONObject) new JSONParser()
                    .parse(
                        new FileReader(
                            Filesystem.getDeployDirectory().getAbsolutePath() + "/fields/" + Constants.fieldVenue.getFilePath()));
        } catch (IOException | ParseException e) {
            System.out.println("Error finding / reading field json" + ErrorUtil.attachJavaClassName(FieldConstantsUtil.class));
            e.printStackTrace();
        }
    }

    public static String jsonEntryToString(String key) {
        return fieldJson.get(key).toString();
    }

    public static boolean jsonEntryToBool(String key) {
        return Boolean.parseBoolean(jsonEntryToString(key));
    }

    public static double jsonEntryToInt(String key) {
        return Integer.parseInt(jsonEntryToString(key));
    }

    public static double jsonEntryToDouble(String key) {
        return Double.parseDouble(jsonEntryToString(key));
    }

    /** Gets the value associated with {@code key} and converts it from inches to meters. */
    public static double getFieldValueMeters(String key) {
        return Units.inchesToMeters(jsonEntryToDouble(key));
    }
}
