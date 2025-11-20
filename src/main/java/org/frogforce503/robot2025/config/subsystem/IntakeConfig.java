package org.frogforce503.robot2025.config.subsystem;

import org.frogforce503.lib.math.Range;
import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import lombok.Builder;

@Builder
public record IntakeConfig(
    int pivotID,
    boolean pivotInverted,
    double pivotOffset,
    PIDConfig pivotPID,
    FFConfig pivotFF,
    Constraints pivotConstraints,
    Range pivotRange,
    double pivotHorizontalAngle,

    int rollerID,
    boolean rollerInverted,
    boolean rollerIsSparkFlex,
    PIDConfig rollerPID,
    FFConfig rollerFF) {}