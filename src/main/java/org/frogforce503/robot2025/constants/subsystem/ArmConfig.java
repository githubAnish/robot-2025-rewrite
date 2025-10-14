package org.frogforce503.robot2025.constants.subsystem;

import org.frogforce503.lib.math.Range;
import org.frogforce503.lib.motorcontrol.tuning.pidf.PIDFConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class ArmConfig {
    public int armID;
    public boolean armInverted;
    public double armOffset;
    public PIDFConfig kPIDF;
    public Constraints kConstraints;
    public Range range;
}