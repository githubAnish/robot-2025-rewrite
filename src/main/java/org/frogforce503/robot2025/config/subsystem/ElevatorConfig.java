package org.frogforce503.robot2025.config.subsystem;

import org.frogforce503.lib.math.Range;
import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import lombok.Builder;

@Builder
public record ElevatorConfig(
    int elevatorID,
    boolean elevatorInverted,
    PIDConfig kPID,
    FFConfig kFF,
    Constraints kConstraints,
    Range range) {}