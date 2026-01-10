package org.frogforce503.robot.constants.hardware;

import org.frogforce503.lib.motorcontrol.PIDConfig;
import org.frogforce503.robot.constants.hardware.subsystem_config.*;

import lombok.Getter;

public abstract class RobotHardware {
    // Subsystem Configs
    @Getter protected DriveConfig driveConfig;
    @Getter protected VisionConfig visionConfig;

    @Getter protected ElevatorConfig elevatorConfig;
    @Getter protected ArmConfig armConfig;
    @Getter protected WristConfig wristConfig;
    @Getter protected ClawConfig clawConfig;
    @Getter protected IntakePivotConfig intakePivotConfig;
    @Getter protected IntakeRollerConfig intakeRollerConfig;

    @Getter protected ClimberConfig climberConfig;
    
    @Getter protected SensorConfig sensorConfig;
    @Getter protected LedsConfig ledsConfig;

    // Other
    @Getter protected PIDConfig followerLinearPID;
    @Getter protected PIDConfig followerThetaPID;
}