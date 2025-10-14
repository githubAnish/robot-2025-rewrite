package org.frogforce503.robot2025.constants.subsystem;

import org.frogforce503.lib.motorcontrol.tuning.pidf.PIDFConfig;

public class ClawConfig {
    public int leftMotorID;
    public int leftMotorInverted;
    public PIDFConfig leftMotorPIDF;

    public int rightMotorID;
    public int rightMotorInverted;
    public PIDFConfig rightMotorPIDF;
}