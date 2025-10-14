package org.frogforce503.robot2025.constants;

import org.frogforce503.lib.math.Range;
import org.frogforce503.lib.motorcontrol.tuning.pidf.PIDFConfig;
import org.frogforce503.lib.swerve.SwervePathFollower;
import org.frogforce503.robot2025.constants.subsystem.*;
import org.frogforce503.robot2025.constants.subsystem.ClawConfig.ClawMotorConfig;
import org.frogforce503.robot2025.constants.subsystem.IntakeConfig.PivotConfig;
import org.frogforce503.robot2025.constants.subsystem.IntakeConfig.RollerConfig;
import org.frogforce503.robot2025.constants.tunerconstants.TunerConstantsCompBot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public class RobotConstantsCompBot extends RobotConstants {
    public RobotConstantsCompBot() {
        this.Drive =
            DriveConfig.builder()
                .steerGains(TunerConstantsCompBot.steerGains)
                .driveGains(TunerConstantsCompBot.driveGains)
                .kSteerClosedLoopOutput(TunerConstantsCompBot.kSteerClosedLoopOutput)
                .kDriveClosedLoopOutput(TunerConstantsCompBot.kDriveClosedLoopOutput)
                .kDriveMotorType(TunerConstantsCompBot.kDriveMotorType)
                .kSteerMotorType(TunerConstantsCompBot.kSteerMotorType)
                .kSteerFeedbackType(TunerConstantsCompBot.kSteerFeedbackType)
                .kSlipCurrent(TunerConstantsCompBot.kSlipCurrent)
                .driveInitialConfigs(TunerConstantsCompBot.driveInitialConfigs)
                .steerInitialConfigs(TunerConstantsCompBot.steerInitialConfigs)
                .encoderInitialConfigs(TunerConstantsCompBot.encoderInitialConfigs)
                .pigeonConfigs(TunerConstantsCompBot.pigeonConfigs)
                .kCANBus(TunerConstantsCompBot.kCANBus)
                .kSpeedAt12Volts(TunerConstantsCompBot.kSpeedAt12Volts)
                .kCoupleRatio(TunerConstantsCompBot.kCoupleRatio)
                .kDriveGearRatio(TunerConstantsCompBot.kDriveGearRatio)
                .kSteerGearRatio(TunerConstantsCompBot.kSteerGearRatio)
                .kWheelRadius(TunerConstantsCompBot.kWheelRadius)
                .kInvertLeftSide(TunerConstantsCompBot.kInvertLeftSide)
                .kInvertRightSide(TunerConstantsCompBot.kInvertRightSide)
                .kPigeonId(TunerConstantsCompBot.kPigeonId)
                .kSteerInertia(TunerConstantsCompBot.kSteerInertia)
                .kDriveInertia(TunerConstantsCompBot.kDriveInertia)
                .kSteerFrictionVoltage(TunerConstantsCompBot.kSteerFrictionVoltage)
                .kDriveFrictionVoltage(TunerConstantsCompBot.kDriveFrictionVoltage)
                .DrivetrainConstants(TunerConstantsCompBot.DrivetrainConstants)
                .kFrontLeftDriveMotorId(TunerConstantsCompBot.kFrontLeftDriveMotorId)
                .kFrontLeftSteerMotorId(TunerConstantsCompBot.kFrontLeftSteerMotorId)
                .kFrontLeftEncoderId(TunerConstantsCompBot.kFrontLeftEncoderId)
                .kFrontLeftEncoderOffset(TunerConstantsCompBot.kFrontLeftEncoderOffset)
                .kFrontLeftSteerMotorInverted(TunerConstantsCompBot.kFrontLeftSteerMotorInverted)
                .kFrontLeftEncoderInverted(TunerConstantsCompBot.kFrontLeftEncoderInverted)
                .kFrontLeftXPos(TunerConstantsCompBot.kFrontLeftXPos)
                .kFrontLeftYPos(TunerConstantsCompBot.kFrontLeftYPos)
                .kFrontRightDriveMotorId(TunerConstantsCompBot.kFrontRightDriveMotorId)
                .kFrontRightSteerMotorId(TunerConstantsCompBot.kFrontRightSteerMotorId)
                .kFrontRightEncoderId(TunerConstantsCompBot.kFrontRightEncoderId)
                .kFrontRightEncoderOffset(TunerConstantsCompBot.kFrontRightEncoderOffset)
                .kFrontRightSteerMotorInverted(TunerConstantsCompBot.kFrontRightSteerMotorInverted)
                .kFrontRightEncoderInverted(TunerConstantsCompBot.kFrontRightEncoderInverted)
                .kFrontRightXPos(TunerConstantsCompBot.kFrontRightXPos)
                .kFrontRightYPos(TunerConstantsCompBot.kFrontRightYPos)
                .kBackLeftDriveMotorId(TunerConstantsCompBot.kBackLeftDriveMotorId)
                .kBackLeftSteerMotorId(TunerConstantsCompBot.kBackLeftSteerMotorId)
                .kBackLeftEncoderId(TunerConstantsCompBot.kBackLeftEncoderId)
                .kBackLeftEncoderOffset(TunerConstantsCompBot.kBackLeftEncoderOffset)
                .kBackLeftSteerMotorInverted(TunerConstantsCompBot.kBackLeftSteerMotorInverted)
                .kBackLeftEncoderInverted(TunerConstantsCompBot.kBackLeftEncoderInverted)
                .kBackLeftXPos(TunerConstantsCompBot.kBackLeftXPos)
                .kBackLeftYPos(TunerConstantsCompBot.kBackLeftYPos)
                .kBackRightDriveMotorId(TunerConstantsCompBot.kBackRightDriveMotorId)
                .kBackRightSteerMotorId(TunerConstantsCompBot.kBackRightSteerMotorId)
                .kBackRightEncoderId(TunerConstantsCompBot.kBackRightEncoderId)
                .kBackRightEncoderOffset(TunerConstantsCompBot.kBackRightEncoderOffset)
                .kBackRightSteerMotorInverted(TunerConstantsCompBot.kBackRightSteerMotorInverted)
                .kBackRightEncoderInverted(TunerConstantsCompBot.kBackRightEncoderInverted)
                .kBackRightXPos(TunerConstantsCompBot.kBackRightXPos)
                .kBackRightYPos(TunerConstantsCompBot.kBackRightYPos)
                .FrontLeft(TunerConstantsCompBot.FrontLeft)
                .FrontRight(TunerConstantsCompBot.FrontRight)
                .BackLeft(TunerConstantsCompBot.BackLeft)
                .BackRight(TunerConstantsCompBot.BackRight)
                .build();

        this.swervePathFollower =
            new SwervePathFollower(
                new PIDController(0.95, 0.0, 0.0),
                new PIDController(0.905, 0.0, 0.0),
                new PIDController(5.0, 0.0, 0.0));

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
                .kPIDF(new PIDFConfig(2.0, 0.0, 0.04, new ElevatorFeedforward(0.01, 0.0, 0.0, 0.002)))
                .kConstraints(new Constraints(5200.0, 600.0))
                .range(new Range(0.0, 33.0))
                .build();

        // Arm
        this.Arm =
            ArmConfig.builder()
                .armID(6)
                .armInverted(false)
                .armOffset(0.893)
                .kPIDF(new PIDFConfig(0.014, 0.0, 0.1, new ArmFeedforward(0.0, 0.75, 0.0, 0.0)))
                .kConstraints(new Constraints(1, 0.5))
                .range(new Range(0.0, 180.0))
                .build();
        
        // Wrist
        this.Wrist =
            WristConfig.builder()
                .wristID(7)
                .wristInverted(true)
                .wristOffset(0.187)
                .kPIDF(new PIDFConfig(0.014, 0.0, 0.0, new ArmFeedforward(0.0, 0.0, 0.0, 0.0)))
                .range(new Range(0.0, 300.0))
                .build();

        // Claw
        this.Claw =
            ClawConfig.builder()
                .leftMotorConfig(
                    ClawMotorConfig.builder()
                        .motorID(5)
                        .motorInverted(true)
                        .kPIDF(new PIDFConfig(0.0001, 0.0, 0.00005, 0.0, 0.0, 0.000096, 0.0))
                        .build())
                .rightMotorConfig(
                    ClawMotorConfig.builder()
                        .motorID(8)
                        .motorInverted(false)
                        .kPIDF(new PIDFConfig(0.0001, 0.0, 0.00005, 0.0, 0.0, 0.000096, 0.0))
                        .build())
                .build();

        // Intake
        this.Intake =
            IntakeConfig.builder()
                .pivotConfig(
                    PivotConfig.builder()
                        .pivotID(3)
                        .pivotInverted(false)
                        .pivotOffset(0.4)
                        .kPIDF(new PIDFConfig(0.01, 0.0, 0.0, new ArmFeedforward(0.0, 0.75, 0.0, 0.0)))
                        .kConstraints(new Constraints(1000, 500))
                        .range(new Range(35, 193))
                        .build())
                .rollerConfig(
                    RollerConfig.builder()
                        .rollerID(4)
                        .rollerInverted(true)
                        .rollerIsSparkFlex(true)
                        .kPIDF(new PIDFConfig(0.000001, 0.0, 0.0, 0.0, 0.0, 0.0001575, 0.0))
                        .build())
                .build();

        // Climber
        this.Climber =
            ClimberConfig.builder()
                .winchID(9)
                .winchInverted(false)
                .kPIDF(new PIDFConfig(0.0, 0.0, 0.0))
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
                        new Translation3d(Units.inchesToMeters(1.635), Units.inchesToMeters(11.241), Units.inchesToMeters(20.003)),
                        new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(15), Units.degreesToRadians(-15))))

                .UPPER_FRONT_RIGHT_CAMERA_TO_CENTER(
                    new Transform3d(
                        new Translation3d(Units.inchesToMeters(1.8026), Units.inchesToMeters(-11.305), Units.inchesToMeters(31.721)),
                        new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(30), Units.degreesToRadians(10))))
        
                .ELEVATOR_BACK_CAMERA_TO_CENTER(
                    new Transform3d(
                        new Translation3d(Units.inchesToMeters(-8.670), Units.inchesToMeters(0), Units.inchesToMeters(41.074)),
                        new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-20), Units.degreesToRadians(180))))

                .LOWER_FRONT_RIGHT_CAMERA_TO_CENTER(
                    new Transform3d(
                        new Translation3d(Units.inchesToMeters(1.635), Units.inchesToMeters(-11.241), Units.inchesToMeters(20.003)),
                        new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(15), Units.degreesToRadians(15))))

                .OBJECT_DETECTION_CAMERA_TO_CENTER(
                    new Transform3d(
                        new Translation3d(Units.inchesToMeters(-8.585), Units.inchesToMeters(0), Units.inchesToMeters(16.532)),
                        new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-24), Units.degreesToRadians(180))))
                
                 // LOWER_FRONT_RIGHT_CAMERA_TO_CENTER = new Transform3d(
                //     new Translation3d(Units.inchesToMeters(1.546), Units.inchesToMeters(-11.35), Units.inchesToMeters(20.25)),
                //     new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(10)));

                .build();

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