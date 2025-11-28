package org.frogforce503.robot2025.config.subsystem;

import org.frogforce503.lib.math.Range;
import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public record ArmConfig(
    int id,
    double mechanismRatio,

    boolean inverted,
    int statorCurrentLimit,
    double zeroOffset,

    PIDConfig kPID,
    FFConfig kFF,
    Constraints kConstraints,
    Range motionRange) {}