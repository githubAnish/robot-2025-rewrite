package org.frogforce503.robot2025.constants.configs;

import org.frogforce503.lib.motorcontrol.tuning.pidf.PIDFConfig;
import lombok.Builder;

@Builder
public record ClawConfig(
    int leftMotorID,
    boolean leftMotorInverted,

    int rightMotorID,
    boolean rightMotorInverted,
    
    PIDFConfig kPIDF) {}