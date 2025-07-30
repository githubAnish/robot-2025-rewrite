package org.frogforce503.lib.drawing;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;

public class DrawOnField {
    private DrawOnField() {}

    public static String path(List<Pose2d> poses) {
        String pointsList = "[";

        int i = 0;
        int granularity = 25;
        
        for (Pose2d pose : poses) {
            if (i % granularity == 0) {
                if (i != 0) {
                    pointsList += ", ";
                }
                pointsList += "[" + ((int) (pose.getX() / 100)) + ", " + (int) (pose.getY() / 100) + "]";
            }
            i++;
        }

        return pointsList += "]";
    }
}
