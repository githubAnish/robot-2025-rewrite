package org.frogforce503.robot2025.constants.subsystem;

import org.frogforce503.lib.math.Range;
import org.frogforce503.lib.motorcontrol.tuning.pidf.PIDFConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class IntakeConfig {
    public int pivotID;
    public boolean pivotInverted;
    public double pivotOffset;
    public PIDFConfig pivotPIDF;
    public Constraints pivotConstraints;
    public Range pivotRange;

    public int rollerID;
    public boolean rollerInverted;
    public boolean rollerIsSparkFlex;
    public PIDFConfig rollerPIDF;
}