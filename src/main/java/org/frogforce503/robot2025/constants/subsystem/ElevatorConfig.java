package org.frogforce503.robot2025.constants.subsystem;

import org.frogforce503.lib.math.Range;
import org.frogforce503.lib.motorcontrol.tuning.pidf.PIDFConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class ElevatorConfig {
    public int elevatorID;
    public boolean elevatorInverted;
    public PIDFConfig kPIDF;
    public Constraints kConstraints;
    public Range range;
}