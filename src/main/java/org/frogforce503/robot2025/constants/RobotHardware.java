package org.frogforce503.robot2025.constants;

import org.frogforce503.robot2025.constants.configs.*;

public interface RobotHardware {
    // Subsystem Configuration Getters
    public ElevatorConfig getElevatorConfig();
    public ArmConfig getArmConfig();
    public WristConfig getWristConfig();
    public ClawConfig getClawConfig();
    public IntakeConfig getIntakeConfig();
    public ClimberConfig getClimberConfig();

    public SensorConfig getSensorsConfig();
    public LedsConfig getLedsConfig();

    public VisionConfig getVisionConfig();
    public DriveConfig getDriveConfig();

    // Other Configuration Getters
    public PathFollowingConfig getPathFollowingConfig();
}