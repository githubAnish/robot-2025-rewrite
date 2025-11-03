package org.frogforce503.robot2025.constants;

import org.frogforce503.robot2025.constants.configs.ArmConfig;
import org.frogforce503.robot2025.constants.configs.ClawConfig;
import org.frogforce503.robot2025.constants.configs.ClimberConfig;
import org.frogforce503.robot2025.constants.configs.DriveConfig;
import org.frogforce503.robot2025.constants.configs.ElevatorConfig;
import org.frogforce503.robot2025.constants.configs.IntakeConfig;
import org.frogforce503.robot2025.constants.configs.LedsConfig;
import org.frogforce503.robot2025.constants.configs.PathFollowingConfig;
import org.frogforce503.robot2025.constants.configs.SensorConfig;
import org.frogforce503.robot2025.constants.configs.VisionConfig;
import org.frogforce503.robot2025.constants.configs.WristConfig;
import org.frogforce503.robot2025.constants.tunerconstants.TunerConstantsProgrammingBot;

public class RobotHardwareProgrammingBot implements RobotHardware {

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

    @Override
    public PathFollowingConfig getPathFollowingConfig() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPathFollowingConfig'");
    }
}