package org.frogforce503.robot2025.constants.hardware.subsystem_config;

public record SensorConfig(
    int elevatorLimitSwitchId,
    int upperBeamBreakId,
    int lowerBeamBreakId,
    int winchLimitSwitchId) {}