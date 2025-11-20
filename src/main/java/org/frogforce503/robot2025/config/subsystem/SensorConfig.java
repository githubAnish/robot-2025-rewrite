package org.frogforce503.robot2025.config.subsystem;
import lombok.Builder;

@Builder
public record SensorConfig(
    int elevatorZeroSwitchID,
    int lowerBeamID,
    int upperBeamID,
    int winchSwitchID) {}