package org.frogforce503.robot.constants.hardware;

import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;
import org.frogforce503.robot.constants.hardware.subsystem_config.*;
import org.frogforce503.robot.constants.tuner.TunerConstantsCompBot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

public class RobotHardwareCompBot extends RobotHardware {
    public RobotHardwareCompBot() {
        // Create drive config
        this.driveConfig =
            new DriveConfig(
                TunerConstantsCompBot.DrivetrainConstants,
                TunerConstantsCompBot.FrontLeft,
                TunerConstantsCompBot.FrontRight,
                TunerConstantsCompBot.BackLeft,
                TunerConstantsCompBot.BackRight);

        // Create vision config
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

        this.visionConfig =
            new VisionConfig(
                FRONT_LEFT_CAMERA_TO_CENTER,
                FRONT_RIGHT_CAMERA_TO_CENTER,
                ELEVATOR_BACK_CAMERA_TO_CENTER,
                LOWER_FRONT_RIGHT_CAMERA_TO_CENTER);
            
        // Create superstructure configs
        this.elevatorConfig =
            new ElevatorConfig(
                2,
                5.90,
                Units.inchesToMeters(1.74),
                false,
                RobotBase.isSimulation() ? 160 : 80,
                new PIDConfig(6, 0, 0.55),
                new FFConfig(0.1, 1.46, 6.09, 0),
                new Constraints(Units.inchesToMeters(120), Units.inchesToMeters(240)),
                Units.inchesToMeters(0),
                Units.inchesToMeters(30)); // 30 inches max height according to JVN Calc

        this.armConfig =
            new ArmConfig(
                6,
                150,
                false,
                80,
                0.0,
                new PIDConfig(6, 0, 0),
                new FFConfig(0, 0.91, 10, 0),
                new Constraints(Units.degreesToRadians(480), Units.degreesToRadians(960)),
                Units.degreesToRadians(-90),
                Units.degreesToRadians(90));

        this.wristConfig =
            new WristConfig(
                7,
                40,
                true,
                40,
                0.0,
                new PIDConfig(2, 0, 0.06),
                new FFConfig(0, 3.356, 0.35, 0),
                new Constraints(Units.degreesToRadians(480), Units.degreesToRadians(960)),
                Units.degreesToRadians(-90),
                Units.degreesToRadians(135));

        this.clawConfig =
            new ClawConfig(
                5,
                8,
                1,
                true,
                false,
                80,
                new PIDConfig(0.0003, 0, 0),
                new FFConfig(0, 0, 0.01155, 0));

        this.intakePivotConfig =
            new IntakePivotConfig(
                3,
                60,
                false,
                40,
                0.0,
                new PIDConfig(0.5, 0, 0.1),
                new FFConfig(0, 2.106, 1.5, 0),
                new Constraints(Units.degreesToRadians(360), Units.degreesToRadians(720)),
                Units.degreesToRadians(-45),
                Units.degreesToRadians(90));

        this.intakeRollerConfig =
            new IntakeRollerConfig(
                true,
                4,
                1.6,
                true,
                80,
                new PIDConfig(0.00001, 0, 0),
                new FFConfig(0, 0, 0.0299, 0));

        // Create other configs
        this.climberConfig =
            new ClimberConfig(
                9,
                false,
                90);

        this.sensorConfig =
            new SensorConfig(
                1,
                3,
                0,
                2);

        this.ledsConfig =
            new LedsConfig(11);

        this.followerLinearPID = new PIDConfig(5.0, 0.0, 0.0);
        this.followerThetaPID = new PIDConfig(4.0, 0.0, 0.0);
    }
}