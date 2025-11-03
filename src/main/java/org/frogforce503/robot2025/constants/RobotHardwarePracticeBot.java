package org.frogforce503.robot2025.constants;

import org.frogforce503.robot2025.constants.configs.*;
import org.frogforce503.robot2025.constants.tunerconstants.TunerConstantsPracticeBot;

public class RobotHardwarePracticeBot implements RobotHardware {

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
                .drivetrainConstants(TunerConstantsPracticeBot.DrivetrainConstants)
                .frontLeft(TunerConstantsPracticeBot.FrontLeft)
                .frontRight(TunerConstantsPracticeBot.FrontRight)
                .backLeft(TunerConstantsPracticeBot.BackLeft)
                .backRight(TunerConstantsPracticeBot.BackRight)
                .build();
    }

    @Override
    public PathFollowingConfig getPathFollowingConfig() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPathFollowingConfig'");
    }
}