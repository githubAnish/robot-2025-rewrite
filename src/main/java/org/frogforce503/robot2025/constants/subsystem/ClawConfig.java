package org.frogforce503.robot2025.constants.subsystem;

import org.frogforce503.lib.motorcontrol.tuning.pidf.PIDFConfig;
import lombok.Builder;

@Builder
public record ClawConfig(
    int leftMotorID,
    boolean leftMotorInverted,
    PIDFConfig leftMotorPIDF,

    int rightMotorID,
    boolean rightMotorInverted,
    PIDFConfig rightMotorPIDF) {}