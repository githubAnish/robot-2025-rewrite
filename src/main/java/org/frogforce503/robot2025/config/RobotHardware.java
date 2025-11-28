package org.frogforce503.robot2025.config;

import org.frogforce503.lib.motorcontrol.PIDConfig;
import org.frogforce503.robot2025.config.subsystem.ArmConfig;
import org.frogforce503.robot2025.config.subsystem.ClawConfig;
import org.frogforce503.robot2025.config.subsystem.ClimberConfig;
import org.frogforce503.robot2025.config.subsystem.DriveConfig;
import org.frogforce503.robot2025.config.subsystem.ElevatorConfig;
import org.frogforce503.robot2025.config.subsystem.IntakePivotConfig;
import org.frogforce503.robot2025.config.subsystem.IntakeRollerConfig;
import org.frogforce503.robot2025.config.subsystem.LedsConfig;
import org.frogforce503.robot2025.config.subsystem.SensorConfig;
import org.frogforce503.robot2025.config.subsystem.VisionConfig;
import org.frogforce503.robot2025.config.subsystem.WristConfig;

public abstract class RobotHardware {
    // Subsystem Configuration Getters
    public abstract ElevatorConfig getElevatorConfig();
    public abstract ArmConfig getArmConfig();
    public abstract WristConfig getWristConfig();
    public abstract ClawConfig getClawConfig();
    public abstract IntakePivotConfig getIntakePivotConfig();
    public abstract IntakeRollerConfig getIntakeRollerConfig();
    public abstract ClimberConfig getClimberConfig();

    public abstract SensorConfig getSensorsConfig();
    public abstract LedsConfig getLedsConfig();

    public abstract VisionConfig getVisionConfig();
    public abstract DriveConfig getDriveConfig();

    // Other
    public PIDConfig xPID;
    public PIDConfig yPID;
    public PIDConfig thetaPID;
}