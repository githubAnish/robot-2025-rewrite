package org.frogforce503.robot2025.config;

import org.frogforce503.robot2025.config.subsystem.ArmConfig;
import org.frogforce503.robot2025.config.subsystem.ClawConfig;
import org.frogforce503.robot2025.config.subsystem.ClimberConfig;
import org.frogforce503.robot2025.config.subsystem.DriveConfig;
import org.frogforce503.robot2025.config.subsystem.ElevatorConfig;
import org.frogforce503.robot2025.config.subsystem.IntakeConfig;
import org.frogforce503.robot2025.config.subsystem.LedsConfig;
import org.frogforce503.robot2025.config.subsystem.SensorConfig;
import org.frogforce503.robot2025.config.subsystem.VisionConfig;
import org.frogforce503.robot2025.config.subsystem.WristConfig;
import org.frogforce503.robot2025.config.tunerconstants.TunerConstantsProgrammingBot;

public class RobotHardwareProgrammingBot extends RobotHardware {
    public RobotHardwareProgrammingBot() {

    }

    @Override
    public ElevatorConfig getElevatorConfig() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getElevatorConfig'");
    }

    @Override
    public ArmConfig getArmConfig() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getArmConfig'");
    }

    @Override
    public WristConfig getWristConfig() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getWristConfig'");
    }

    @Override
    public ClawConfig getClawConfig() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getClawConfig'");
    }

    @Override
    public IntakeConfig getIntakeConfig() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getIntakeConfig'");
    }

    @Override
    public ClimberConfig getClimberConfig() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getClimberConfig'");
    }

    @Override
    public SensorConfig getSensorsConfig() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getSensorsConfig'");
    }

    @Override
    public LedsConfig getLedsConfig() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getLedsConfig'");
    }

    @Override
    public VisionConfig getVisionConfig() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getVisionConfig'");
    }

    @Override
    public DriveConfig getDriveConfig() {
        return
            DriveConfig.builder()
                .drivetrainConstants(TunerConstantsProgrammingBot.DrivetrainConstants)
                .frontLeft(TunerConstantsProgrammingBot.FrontLeft)
                .frontRight(TunerConstantsProgrammingBot.FrontRight)
                .backLeft(TunerConstantsProgrammingBot.BackLeft)
                .backRight(TunerConstantsProgrammingBot.BackRight)
                .build();
    }
}