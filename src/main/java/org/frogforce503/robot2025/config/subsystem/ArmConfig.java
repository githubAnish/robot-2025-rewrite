package org.frogforce503.robot2025.config.subsystem;

import org.frogforce503.lib.math.Range;
import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import lombok.Builder;

@Builder
public record ArmConfig(
    int armID,
    boolean armInverted,
    double armOffset,
    PIDConfig kPID,
    FFConfig kFF,
    Constraints kConstraints,
    Range range,
    double horizontalAngle) {}