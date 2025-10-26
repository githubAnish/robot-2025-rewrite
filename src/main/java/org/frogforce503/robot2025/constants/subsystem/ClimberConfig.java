package org.frogforce503.robot2025.constants.subsystem;

import org.frogforce503.lib.motorcontrol.tuning.pidf.PIDFConfig;
import lombok.Builder;

@Builder
public record ClimberConfig(
    int winchID,
    boolean winchInverted,
    PIDFConfig kPIDF) {}