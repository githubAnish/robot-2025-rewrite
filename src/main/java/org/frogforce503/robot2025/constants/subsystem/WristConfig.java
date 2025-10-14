package org.frogforce503.robot2025.constants.subsystem;

import org.frogforce503.lib.math.Range;
import org.frogforce503.lib.motorcontrol.tuning.pidf.PIDFConfig;

public class WristConfig {
    public int wristID;
    public boolean wristInverted;
    public double wristOffset;
    public PIDFConfig kPIDF;
    public Range range;
}