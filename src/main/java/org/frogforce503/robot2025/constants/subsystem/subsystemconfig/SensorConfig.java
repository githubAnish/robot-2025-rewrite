package org.frogforce503.robot2025.constants.subsystem.subsystemconfig;

public record SensorConfig(
    int elevatorLimitSwitchId,
    int upperBeamBreakId,
    int lowerBeamBreakId,
    int winchLimitSwitchId) {}