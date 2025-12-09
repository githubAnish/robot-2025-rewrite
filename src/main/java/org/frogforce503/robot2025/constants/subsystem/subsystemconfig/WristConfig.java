package org.frogforce503.robot2025.constants.subsystem.subsystemconfig;

import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public record WristConfig(
    int id,
    double mechanismRatio,

    boolean inverted,
    int statorCurrentLimit,
    double zeroOffset,

    PIDConfig kPID,
    FFConfig kFF,
    Constraints kConstraints,

    double minAngle,
    double maxAngle) {}