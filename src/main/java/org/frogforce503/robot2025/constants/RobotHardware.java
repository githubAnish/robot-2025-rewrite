package org.frogforce503.robot2025.constants;

import org.frogforce503.robot2025.constants.subsystem.*;

public interface RobotHardware {
    public ElevatorConfig getElevatorConfig();
    public ArmConfig getArmConfig();
    public WristConfig getWristConfig();
    public ClawConfig getClawConfig();
    public IntakeConfig getIntakeConfig();
    public ClimberConfig getClimberConfig();

    public SensorConfig getSensorsConfig();
    public LedsConfig getLedsConfig();

    public VisionConfig getVisionConfig();
}