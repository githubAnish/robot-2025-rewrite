package org.frogforce503.lib.math;

import java.util.Optional;
import java.util.TreeMap;
import java.util.Map.Entry;

public class StepFunction {
    private TreeMap<Double, Double> map = new TreeMap<>();

    public void put(double x, double y) {
        map.put(x, y);
    }

    public double get(double lookupX) {
        return getFloorEntry(lookupX);
    }

    private double handleNull(Optional<Entry<Double, Double>> entry) {
        return
            entry
                .orElse(map.firstEntry())
                .getValue();
    }
    
    public double getFloorEntry(double lookupX) {
        return
            handleNull(
                Optional.ofNullable(map.floorEntry(lookupX)));
    }

    public double getCeilingEntry(double lookupX) {
        return
            handleNull(
                Optional.ofNullable(map.ceilingEntry(lookupX)));
    }
}
