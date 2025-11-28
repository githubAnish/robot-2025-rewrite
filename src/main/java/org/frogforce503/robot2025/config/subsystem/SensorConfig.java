package org.frogforce503.robot2025.config.subsystem;

public record SensorConfig(
    int elevatorZeroSwitchID,
    int lowerBeamID,
    int upperBeamID,
    int winchSwitchID) {}