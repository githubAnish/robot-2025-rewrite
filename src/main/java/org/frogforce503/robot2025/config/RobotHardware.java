package org.frogforce503.robot2025.config;

import org.frogforce503.lib.motorcontrol.PIDConfig;
import org.frogforce503.robot2025.config.subsystem.*;

public abstract class RobotHardware {
    // Subsystem Configuration Getters
    public abstract ElevatorConfig getElevatorConfig();
    public abstract ArmConfig getArmConfig();
    public abstract WristConfig getWristConfig();
    public abstract ClawConfig getClawConfig();
    public abstract IntakeConfig getIntakeConfig();
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