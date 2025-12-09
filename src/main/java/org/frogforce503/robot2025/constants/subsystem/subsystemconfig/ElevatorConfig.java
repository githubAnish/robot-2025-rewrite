package org.frogforce503.robot2025.constants.subsystem.subsystemconfig;

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
    
    double minHeight,
    double maxHeight) {}