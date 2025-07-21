package org.frogforce503.lib.control.speed;

import org.frogforce503.lib.util.LoggedTunableNumber;

public record SpeedConstraintsTuningConfig(
        LoggedTunableNumber maxVelocityMetersPerSec,
        LoggedTunableNumber maxAccelerationMetersPerSec2) {}