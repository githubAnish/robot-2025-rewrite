package org.frogforce503.robot.constants.hardware.subsystem_config;

import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;

public record IntakeRollerConfig(
    boolean isSparkFlex,
    int id,
    double mechanismRatio,

    boolean inverted,
    int statorCurrentLimit,
    
    PIDConfig kPID,
    FFConfig kFF) {}