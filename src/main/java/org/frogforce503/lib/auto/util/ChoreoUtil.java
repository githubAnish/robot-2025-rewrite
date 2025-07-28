package org.frogforce503.lib.auto.util;

import java.util.List;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

public class ChoreoUtil {
    private ChoreoUtil() {}
    
    /** Returns auto trajectories from an {@code AutoRoutine} based on splits in the given choreo file. */
    public static List<AutoTrajectory> loadAutoTrajSplits(AutoRoutine routine, String choreoFileName) {
        List<Integer> splits =
            routine
                .trajectory(choreoFileName)
                .getRawTrajectory()
                .splits();

        return
            splits
                .stream()
                .map(
                    split ->
                        routine.trajectory(
                            choreoFileName,
                            splits.indexOf(split)))
                .toList();
    }
}