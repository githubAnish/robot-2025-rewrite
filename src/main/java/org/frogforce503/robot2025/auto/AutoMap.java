package org.frogforce503.robot2025.auto;

import java.util.HashMap;
import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.frogforce503.robot2025.auto.AutoMap.StartingLocation;

/**
 * AutoMap is a registry that stores available autonomous routines.
 * It maps: {@link Alliance} → {@link StartingLocation} → Auto Name (String) → Supplier<AutoMode>.
 */
public class AutoMap extends HashMap<Alliance, HashMap<StartingLocation, HashMap<String, Supplier<AutoMode>>>> {
    public AutoMap() {
        for (Alliance alliance : Alliance.values()) {
            put(alliance, new HashMap<>());

            for (StartingLocation loc : StartingLocation.values()) {
                get(alliance).put(loc, new HashMap<>());
            }
        }
    }

    /**
     * Puts an autonomous routine in the map.
     */
    public void putAuto(
        Alliance alliance,
        StartingLocation location,
        String name,
        Supplier<AutoMode> supplier
    ) {
        get(alliance).get(location).put(name, supplier);
    }

    /**
     * Puts an autonomous routine in the map.
     */
    public void putAuto(
        Alliance alliance,
        StartingLocation location,
        Supplier<AutoMode> supplier
    ) {
        putAuto(alliance, location, supplier.get().getName(), supplier);
    }

    /**
     * Get an autonomous routine by name.
     */
    public Supplier<AutoMode> getAuto(Alliance alliance, StartingLocation location, String name) {
        return
            get(alliance)
                .getOrDefault(location, new HashMap<>())
                .get(name);
    }

    public List<Supplier<AutoMode>> getAutos() {
        return
            values()
                .stream()
                .flatMap(locMap -> locMap.values().stream())
                .flatMap(nameMap -> nameMap.values().stream())
                .collect(Collectors.toList());
    }

    public enum StartingLocation {
        LEFT,
        CENTER,
        RIGHT
    }
}
