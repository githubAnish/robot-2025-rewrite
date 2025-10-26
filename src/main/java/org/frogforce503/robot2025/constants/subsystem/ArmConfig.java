package org.frogforce503.robot2025.constants.subsystem;

import org.frogforce503.lib.math.Range;
import org.frogforce503.lib.motorcontrol.tuning.pidf.PIDFConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import lombok.Builder;

@Builder
public record ArmConfig(
    int armID,
    boolean armInverted,
    double armOffset,
    PIDFConfig kPIDF,
    Constraints kConstraints,
    Range range) {}