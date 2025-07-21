package org.frogforce503.robot2025.offsets;

import org.frogforce503.lib.math.GeomUtil;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public record Offset(
    double horizontal,
    double vertical
) {
    public Offset inchesToMeters() {
        return new Offset(
            Units.inchesToMeters(horizontal),
            Units.inchesToMeters(vertical));
    }

    public Offset metersToInches() {
        return new Offset(
            Units.metersToInches(horizontal),
            Units.metersToInches(vertical));
    }

    public Translation2d toTranslation2d() {
        return new Translation2d(-vertical, horizontal);
    }

    public Transform2d toTransform2d() {
        return GeomUtil.toTransform2d(-vertical, horizontal);
    }
}