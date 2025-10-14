package org.frogforce503.robot2025.constants;

import org.frogforce503.lib.math.Range;
import org.frogforce503.lib.motorcontrol.tuning.pidf.PIDFConfig;
import org.frogforce503.lib.swerve.SwervePathFollower;
import org.frogforce503.robot2025.constants.subsystem.*;
import org.frogforce503.robot2025.constants.subsystem.ClawConfig.ClawMotorConfig;
import org.frogforce503.robot2025.constants.subsystem.IntakeConfig.PivotConfig;
import org.frogforce503.robot2025.constants.subsystem.IntakeConfig.RollerConfig;
import org.frogforce503.robot2025.constants.tunerconstants.TunerConstantsPracticeBot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public class RobotConstantsPracticeBot extends RobotConstants {
    public RobotConstantsPracticeBot() {
        // Sensors
        this.Sensors =
            SensorConfig.builder()
                .elevatorZeroSwitchID(1)
                .lowerBeamID(0)
                .upperBeamID(3)
                .winchSwitchID(2)
                .build();

        // Elevator
        this.Elevator =
            ElevatorConfig.builder()
                .elevatorID(2)
                .elevatorInverted(false)
                .kPIDF(new PIDFConfig())
                .kConstraints(new Constraints(1.0, 1.0))
                .range(new Range(0.0, 0.0))
                .build();

        // Arm
        this.Arm =
            ArmConfig.builder()
                .armID(6)
                .armInverted(false)
                .armOffset(0.0)
                .kPIDF(new PIDFConfig())
                .kConstraints(new Constraints(1.0, 1.0))
                .range(new Range(0.0, 0.0))
                .build();
        
        // Wrist
        this.Wrist =
            WristConfig.builder()
                .wristID(7)
                .wristInverted(false)
                .wristOffset(0.0)
                .kPIDF(new PIDFConfig())
                .range(new Range(0.0, 0.0))
                .build();

        // Claw
        this.Claw =
            ClawConfig.builder()
                .leftMotorConfig(
                    ClawMotorConfig.builder()
                        .motorID(5)
                        .motorInverted(false)
                        .kPIDF(new PIDFConfig())
                        .build())
                .rightMotorConfig(
                    ClawMotorConfig.builder()
                        .motorID(8)
                        .motorInverted(false)
                        .kPIDF(new PIDFConfig())
                        .build())
                .build();

        // Intake
        this.Intake =
            IntakeConfig.builder()
                .pivotConfig(
                    PivotConfig.builder()
                        .pivotID(3)
                        .pivotInverted(false)
                        .pivotOffset(0.0)
                        .kPIDF(new PIDFConfig())
                        .kConstraints(new Constraints(1.0, 1.0))
                        .range(new Range(0.0, 0.0))
                        .build())
                .rollerConfig(
                    RollerConfig.builder()
                        .rollerID(4)
                        .rollerInverted(false)
                        .rollerIsSparkFlex(false)
                        .kPIDF(new PIDFConfig())
                        .build())
                .build();

        // Climber
        this.Climber =
            ClimberConfig.builder()
                .winchID(9)
                .winchInverted(false)
                .kPIDF(new PIDFConfig())
                .build();

        this.Leds =
            LedsConfig.builder()
                .candleID(11)
                .build();

        /*
         * positive X axis points ahead, the positive Y axis points left, and the positive Z axis points up referenced from the floor. 
         * When viewed with each positive axis pointing toward you, counter-clockwise (CCW) is a positive value and clockwise (CW) is a negative value.
         * 
         * Positive X: Front of Robot
         * Positive Y: Left of Robot
         * Positive Z: Up
         * 
         * Positive Roll (Rotation about X): Robot rolls to its right
         * Positive Pitch (Rotation about Y): Robot points downwards
         * Positive Yaw (Rotation about Z): Robot rotates left
         */

        //Constants from Comp Bot CAD as of (02-19-2025)
        this.Vision =
            VisionConfig.builder()
                .FRONT_LEFT_CAMERA_TO_CENTER(
                    new Transform3d(
                        new Translation3d(Units.inchesToMeters(1.288), Units.inchesToMeters(11.525), Units.inchesToMeters(19.956)),
                        new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(15), Units.degreesToRadians(-15))))

                .UPPER_FRONT_RIGHT_CAMERA_TO_CENTER(
                    new Transform3d(
                        new Translation3d(Units.inchesToMeters(1.288), Units.inchesToMeters(-11.525), Units.inchesToMeters(31.721)),
                        new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(30), Units.degreesToRadians(10))))
        
                .ELEVATOR_BACK_CAMERA_TO_CENTER(
                    new Transform3d(
                        new Translation3d(Units.inchesToMeters(-8.670), Units.inchesToMeters(0), Units.inchesToMeters(41.074)),
                        new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-20), Units.degreesToRadians(180))))

                .LOWER_FRONT_RIGHT_CAMERA_TO_CENTER(
                    new Transform3d(
                        new Translation3d(Units.inchesToMeters(-5.330), Units.inchesToMeters(0), Units.inchesToMeters(41.074)),
                        new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-20), Units.degreesToRadians(0))))

                .OBJECT_DETECTION_CAMERA_TO_CENTER(
                    new Transform3d(
                        new Translation3d(Units.inchesToMeters(-8.585), Units.inchesToMeters(0), Units.inchesToMeters(16.532)),
                        new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-24), Units.degreesToRadians(180))))
                
                .build();


            // LOWER_FRONT_RIGHT_CAMERA_TO_CENTER = new Transform3d(
            //     new Translation3d(Units.inchesToMeters(1.546), Units.inchesToMeters(-11.35), Units.inchesToMeters(20.25)),
            //     new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(10)));

            this.Drive =
                DriveConfig.builder()
                    .steerGains(TunerConstantsPracticeBot.steerGains)
                    .driveGains(TunerConstantsPracticeBot.driveGains)
                    .kSteerClosedLoopOutput(TunerConstantsPracticeBot.kSteerClosedLoopOutput)
                    .kDriveClosedLoopOutput(TunerConstantsPracticeBot.kDriveClosedLoopOutput)
                    .kDriveMotorType(TunerConstantsPracticeBot.kDriveMotorType)
                    .kSteerMotorType(TunerConstantsPracticeBot.kSteerMotorType)
                    .kSteerFeedbackType(TunerConstantsPracticeBot.kSteerFeedbackType)
                    .kSlipCurrent(TunerConstantsPracticeBot.kSlipCurrent)
                    .driveInitialConfigs(TunerConstantsPracticeBot.driveInitialConfigs)
                    .steerInitialConfigs(TunerConstantsPracticeBot.steerInitialConfigs)
                    .encoderInitialConfigs(TunerConstantsPracticeBot.encoderInitialConfigs)
                    .pigeonConfigs(TunerConstantsPracticeBot.pigeonConfigs)
                    .kCANBus(TunerConstantsPracticeBot.kCANBus)
                    .kSpeedAt12Volts(TunerConstantsPracticeBot.kSpeedAt12Volts)
                    .kCoupleRatio(TunerConstantsPracticeBot.kCoupleRatio)
                    .kDriveGearRatio(TunerConstantsPracticeBot.kDriveGearRatio)
                    .kSteerGearRatio(TunerConstantsPracticeBot.kSteerGearRatio)
                    .kWheelRadius(TunerConstantsPracticeBot.kWheelRadius)
                    .kInvertLeftSide(TunerConstantsPracticeBot.kInvertLeftSide)
                    .kInvertRightSide(TunerConstantsPracticeBot.kInvertRightSide)
                    .kPigeonId(TunerConstantsPracticeBot.kPigeonId)
                    .kSteerInertia(TunerConstantsPracticeBot.kSteerInertia)
                    .kDriveInertia(TunerConstantsPracticeBot.kDriveInertia)
                    .kSteerFrictionVoltage(TunerConstantsPracticeBot.kSteerFrictionVoltage)
                    .kDriveFrictionVoltage(TunerConstantsPracticeBot.kDriveFrictionVoltage)
                    .DrivetrainConstants(TunerConstantsPracticeBot.DrivetrainConstants)
                    .kFrontLeftDriveMotorId(TunerConstantsPracticeBot.kFrontLeftDriveMotorId)
                    .kFrontLeftSteerMotorId(TunerConstantsPracticeBot.kFrontLeftSteerMotorId)
                    .kFrontLeftEncoderId(TunerConstantsPracticeBot.kFrontLeftEncoderId)
                    .kFrontLeftEncoderOffset(TunerConstantsPracticeBot.kFrontLeftEncoderOffset)
                    .kFrontLeftSteerMotorInverted(TunerConstantsPracticeBot.kFrontLeftSteerMotorInverted)
                    .kFrontLeftEncoderInverted(TunerConstantsPracticeBot.kFrontLeftEncoderInverted)
                    .kFrontLeftXPos(TunerConstantsPracticeBot.kFrontLeftXPos)
                    .kFrontLeftYPos(TunerConstantsPracticeBot.kFrontLeftYPos)
                    .kFrontRightDriveMotorId(TunerConstantsPracticeBot.kFrontRightDriveMotorId)
                    .kFrontRightSteerMotorId(TunerConstantsPracticeBot.kFrontRightSteerMotorId)
                    .kFrontRightEncoderId(TunerConstantsPracticeBot.kFrontRightEncoderId)
                    .kFrontRightEncoderOffset(TunerConstantsPracticeBot.kFrontRightEncoderOffset)
                    .kFrontRightSteerMotorInverted(TunerConstantsPracticeBot.kFrontRightSteerMotorInverted)
                    .kFrontRightEncoderInverted(TunerConstantsPracticeBot.kFrontRightEncoderInverted)
                    .kFrontRightXPos(TunerConstantsPracticeBot.kFrontRightXPos)
                    .kFrontRightYPos(TunerConstantsPracticeBot.kFrontRightYPos)
                    .kBackLeftDriveMotorId(TunerConstantsPracticeBot.kBackLeftDriveMotorId)
                    .kBackLeftSteerMotorId(TunerConstantsPracticeBot.kBackLeftSteerMotorId)
                    .kBackLeftEncoderId(TunerConstantsPracticeBot.kBackLeftEncoderId)
                    .kBackLeftEncoderOffset(TunerConstantsPracticeBot.kBackLeftEncoderOffset)
                    .kBackLeftSteerMotorInverted(TunerConstantsPracticeBot.kBackLeftSteerMotorInverted)
                    .kBackLeftEncoderInverted(TunerConstantsPracticeBot.kBackLeftEncoderInverted)
                    .kBackLeftXPos(TunerConstantsPracticeBot.kBackLeftXPos)
                    .kBackLeftYPos(TunerConstantsPracticeBot.kBackLeftYPos)
                    .kBackRightDriveMotorId(TunerConstantsPracticeBot.kBackRightDriveMotorId)
                    .kBackRightSteerMotorId(TunerConstantsPracticeBot.kBackRightSteerMotorId)
                    .kBackRightEncoderId(TunerConstantsPracticeBot.kBackRightEncoderId)
                    .kBackRightEncoderOffset(TunerConstantsPracticeBot.kBackRightEncoderOffset)
                    .kBackRightSteerMotorInverted(TunerConstantsPracticeBot.kBackRightSteerMotorInverted)
                    .kBackRightEncoderInverted(TunerConstantsPracticeBot.kBackRightEncoderInverted)
                    .kBackRightXPos(TunerConstantsPracticeBot.kBackRightXPos)
                    .kBackRightYPos(TunerConstantsPracticeBot.kBackRightYPos)
                    .FrontLeft(TunerConstantsPracticeBot.FrontLeft)
                    .FrontRight(TunerConstantsPracticeBot.FrontRight)
                    .BackLeft(TunerConstantsPracticeBot.BackLeft)
                    .BackRight(TunerConstantsPracticeBot.BackRight)
                    .build();

        this.swervePathFollower =
            new SwervePathFollower(
                new PIDController(0.95, 0.0, 0.0),
                new PIDController(0.905, 0.0, 0.0),
                new PIDController(5.0, 0.0, 0.0));

        this.kVehicleToFrontLeft = new Translation2d(this.Drive.FrontLeft().LocationX, this.Drive.FrontLeft().LocationY);
        this.kVehicleToFrontRight = new Translation2d(this.Drive.FrontRight().LocationX, this.Drive.FrontRight().LocationY);
        this.kVehicleToBackRight = new Translation2d(this.Drive.BackRight().LocationX, this.Drive.BackRight().LocationY);
        this.kVehicleToBackLeft = new Translation2d(this.Drive.BackLeft().LocationX, this.Drive.BackLeft().LocationY);

        this.kWheelbaseLength = kVehicleToFrontLeft.getDistance(kVehicleToBackLeft);
        this.kWheelbaseWidth = kVehicleToFrontLeft.getDistance(kVehicleToFrontRight);

        // Swerve Kinematics
        this.kinematics =
            new SwerveDriveKinematics(
                new Translation2d[] {
                    kVehicleToFrontLeft,
                    kVehicleToFrontRight,
                    kVehicleToBackLeft,
                    kVehicleToBackRight});
    }    
}