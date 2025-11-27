package org.frogforce503.robot2025.config;

import org.frogforce503.lib.math.Range;
import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;
import org.frogforce503.robot2025.config.subsystem.*;
import org.frogforce503.robot2025.config.tunerconstants.TunerConstantsCompBot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

public class RobotHardwareCompBot extends RobotHardware {
    public RobotHardwareCompBot() {
        // PID for pathfollowing
        this.xPID = new PIDConfig(5.0, 0.0, 0.0);
        this.yPID = new PIDConfig(5.0, 0.0, 0.0);
        this.thetaPID = new PIDConfig(4.0, 0.0, 0.0);
    }

    @Override
    public ElevatorConfig getElevatorConfig() {
        return
            new ElevatorConfig(
                2,
                5.90,
                Units.inchesToMeters(1.74),
                false,
                RobotBase.isSimulation() ? 160 : 80,
                new PIDConfig(0.0, 0.0, 0.0),
                new FFConfig(0, 0, 0, 0),
                new Constraints(0, 0),
                new Range(Units.inchesToMeters(0), Units.inchesToMeters(30))); // 30 inches max height according to JVN Calc
    }

    @Override
    public ArmConfig getArmConfig() {
        return
            new ArmConfig(
                6,
                150,
                false,
                30,
                0.0,
                new PIDConfig(0.0, 0.0, 0.0),
                new FFConfig(0, 0, 0, 0),
                new Constraints(0, 0),
                new Range(Units.degreesToRadians(-90), Units.degreesToRadians(90)));
    }

    @Override
    public WristConfig getWristConfig() {
        return
            new WristConfig(
                7,
                40,
                true,
                40,
                0.0,
                new PIDConfig(0.0, 0.0, 0.0),
                new FFConfig(0, 0, 0, 0),
                new Range(Units.degreesToRadians(-90), Units.degreesToRadians(135)));
    }

    @Override
    public ClawConfig getClawConfig() {
        return
            new ClawConfig(
                5,
                8,
                25,
                true,
                false,
                35,
                new PIDConfig(0.0, 0.0, 0.0),
                new FFConfig(0, 0, 0, 0));
    }

    @Override
    public IntakePivotConfig getIntakePivotConfig() {
        return
            new IntakePivotConfig(
                3,
                60,
                false,
                40,
                0.0,
                new PIDConfig(0.0, 0.0, 0.0),
                new FFConfig(0, 0, 0, 0),
                new Constraints(0, 0),
                new Range(Units.degreesToRadians(-45), Units.degreesToRadians(90)));
    }

    @Override
    public IntakeRollerConfig getIntakeRollerConfig() {
        return
            new IntakeRollerConfig(
                true,
                4,
                1.6,
                true,
                80,
                new PIDConfig(0.0, 0.0, 0.0),
                new FFConfig(0, 0, 0, 0));
    }

    @Override
    public ClimberConfig getClimberConfig() {
        return
            new ClimberConfig(
                9,
                false,
                90);
    }

    @Override
    public SensorConfig getSensorsConfig() {
        return
            new SensorConfig(
                1,
                0,
                3,
                2);
    }

    @Override
    public LedsConfig getLedsConfig() {
        return new LedsConfig(11);
    }

    @Override
    public VisionConfig getVisionConfig() {
        final Transform3d FRONT_LEFT_CAMERA_TO_CENTER = new Transform3d(
            new Translation3d(Units.inchesToMeters(1.635), Units.inchesToMeters(11.241), Units.inchesToMeters(20.003)),
            new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(15), Units.degreesToRadians(-15)));

        final Transform3d FRONT_RIGHT_CAMERA_TO_CENTER = new Transform3d(
            new Translation3d(Units.inchesToMeters(1.8026), Units.inchesToMeters(-11.305), Units.inchesToMeters(31.721)),
            new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(30), Units.degreesToRadians(10)));
 
        final Transform3d ELEVATOR_BACK_CAMERA_TO_CENTER = new Transform3d(
            new Translation3d(Units.inchesToMeters(-8.670), Units.inchesToMeters(0), Units.inchesToMeters(41.074)),
            new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-20), Units.degreesToRadians(180)));

        final Transform3d LOWER_FRONT_RIGHT_CAMERA_TO_CENTER = new Transform3d(
            new Translation3d(Units.inchesToMeters(1.635), Units.inchesToMeters(-11.241), Units.inchesToMeters(20.003)),
            new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(15), Units.degreesToRadians(15)));

        return new VisionConfig(
            FRONT_LEFT_CAMERA_TO_CENTER,
            FRONT_RIGHT_CAMERA_TO_CENTER,
            ELEVATOR_BACK_CAMERA_TO_CENTER,
            LOWER_FRONT_RIGHT_CAMERA_TO_CENTER);
    }

    @Override
    public DriveConfig getDriveConfig() {
        return
            new DriveConfig(
                TunerConstantsCompBot.DrivetrainConstants,
                TunerConstantsCompBot.FrontLeft,
                TunerConstantsCompBot.FrontRight,
                TunerConstantsCompBot.BackLeft,
                TunerConstantsCompBot.BackRight);
    }
}