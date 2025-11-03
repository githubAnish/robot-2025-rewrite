package org.frogforce503.robot2025.constants.configs;

import org.frogforce503.lib.math.Range;
import org.frogforce503.lib.motorcontrol.tuning.pidf.PIDFConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import lombok.Builder;

@Builder
public record IntakeConfig(
    int pivotID,
    boolean pivotInverted,
    double pivotOffset,
    PIDFConfig pivotPIDF,
    Constraints pivotConstraints,
    Range pivotRange,

    int rollerID,
    boolean rollerInverted,
    boolean rollerIsSparkFlex,
    PIDFConfig rollerPIDF) {}