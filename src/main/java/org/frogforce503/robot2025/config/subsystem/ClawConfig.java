package org.frogforce503.robot2025.config.subsystem;

import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;

import lombok.Builder;

@Builder
public record ClawConfig(
    int leftMotorID,
    boolean leftMotorInverted,

    int rightMotorID,
    boolean rightMotorInverted,
    
    PIDConfig kPID,
    FFConfig kFF) {}