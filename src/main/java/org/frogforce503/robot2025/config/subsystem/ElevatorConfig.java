package org.frogforce503.robot2025.config.subsystem;

import org.frogforce503.lib.math.Range;
import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public record ElevatorConfig(
    int id,
    double mechanismRatio,
    double sprocketPitchDiameter,

    boolean inverted,
    int statorCurrentLimit,

    PIDConfig kPID,
    FFConfig kFF,
    Constraints kConstraints,
    Range motionRange) {}