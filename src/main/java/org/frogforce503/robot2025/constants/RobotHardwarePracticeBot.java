package org.frogforce503.robot2025.constants;

import org.frogforce503.lib.math.Range;
import org.frogforce503.lib.motorcontrol.tuning.pidf.PIDFConfig;
import org.frogforce503.lib.swerve.SwervePathFollower;
import org.frogforce503.robot2025.constants.subsystem.*;
import org.frogforce503.robot2025.constants.tunerconstants.TunerConstantsPracticeBot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

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
     
}