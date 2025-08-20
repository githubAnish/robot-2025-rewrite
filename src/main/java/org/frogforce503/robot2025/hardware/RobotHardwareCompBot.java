package org.frogforce503.robot2025.hardware;


import org.frogforce503.lib.auto.follower.AutoPIDController;
import org.frogforce503.lib.math.Range;
import org.frogforce503.lib.motorcontrol.tuning.pidf.PIDFConfig;
import org.frogforce503.robot2025.hardware.tunerconstants.TunerConstantsCompBot;

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

public class RobotHardwareCompBot extends RobotHardware {
    @Override
    public void initializeConstants() {
        // Sensors
        this.sensorConstants =
            SensorConstants.builder()
                .elevatorZeroSwitchID(1)
                .lowerBeamID(0)
                .upperBeamID(3)
                .winchSwitchID(2)
                .build();

        // Elevator
        this.elevatorConstants =
            ElevatorConstants.builder()
                .elevatorID(2)
                .elevatorInverted(false)
                .kPIDF(new PIDFConfig(2.0, 0.0, 0.04, new ElevatorFeedforward(0.01, 0.0, 0.0, 0.002)))
                .kConstraints(new Constraints(5200.0, 600.0))
                .range(new Range(0.0, 33.0))
                .build();

        // Arm
        this.armConstants =
            ArmConstants.builder()
                .armID(6)
                .armInverted(false)
                .armOffset(0.893)
                .kPIDF(new PIDFConfig(0.014, 0.0, 0.1, new ArmFeedforward(0.0, 0.75, 0.0, 0.0)))
                .kConstraints(new Constraints(1000, 500))
                .range(new Range(0.0, 180.0))
                .build();
        
        // Wrist
        this.wristConstants =
            WristConstants.builder()
                .wristID(7)
                .wristInverted(true)
                .wristOffset(0.187)
                .kPIDF(new PIDFConfig(0.014, 0.0, 0.0, new ArmFeedforward(0.0, 0.0, 0.0, 0.0)))
                .range(new Range(0.0, 300.0))
                .build();

        // Claw
        this.clawConstants =
            ClawConstants.builder()
                .leftMotorConstants(
                    ClawMotorConstants.builder()
                        .motorID(5)
                        .motorInverted(true)
                        .kPIDF(new PIDFConfig(0.0001, 0.0, 0.00005, 0.0, 0.0, 0.000096, 0.0))
                        .build())
                .rightMotorConstants(
                    ClawMotorConstants.builder()
                        .motorID(8)
                        .motorInverted(false)
                        .kPIDF(new PIDFConfig(0.0001, 0.0, 0.00005, 0.0, 0.0, 0.000096, 0.0))
                        .build())
                .build();

        // Intake
        this.intakeConstants =
            IntakeConstants.builder()
                .pivotConstants(
                    PivotConstants.builder()
                        .pivotID(3)
                        .pivotInverted(false)
                        .pivotOffset(0.4)
                        .kPIDF(new PIDFConfig(0.01, 0.0, 0.0, new ArmFeedforward(0.0, 0.75, 0.0, 0.0)))
                        .kConstraints(new Constraints(1000, 500))
                        .range(new Range(35, 193))
                        .build())
                .rollerConstants(
                    RollerConstants.builder()
                        .rollerID(4)
                        .rollerInverted(true)
                        .rollerIsSparkFlex(true)
                        .kPIDF(new PIDFConfig(0.000001, 0.0, 0.0, 0.0, 0.0, 0.0001575, 0.0))
                        .build())
                .build();

        // Climber
        this.climberConstants =
            ClimberConstants.builder()
                .winchID(9)
                .winchInverted(false)
                .kPIDF(new PIDFConfig(0.0, 0.0, 0.0))
                .build();

        candleID = 11;

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
        FRONT_LEFT_CAMERA_TO_CENTER = new Transform3d(
            new Translation3d(Units.inchesToMeters(1.635), Units.inchesToMeters(11.241), Units.inchesToMeters(20.003)),
            new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(15), Units.degreesToRadians(-15)));

        FRONT_RIGHT_CAMERA_TO_CENTER = new Transform3d(
            new Translation3d(Units.inchesToMeters(1.8026), Units.inchesToMeters(-11.305), Units.inchesToMeters(31.721)),
            new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(30), Units.degreesToRadians(10)));
 
        ELEVATOR_BACK_CAMERA_TO_CENTER = new Transform3d(
            new Translation3d(Units.inchesToMeters(-8.670), Units.inchesToMeters(0), Units.inchesToMeters(41.074)),
            new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-20), Units.degreesToRadians(180)));

        LOWER_FRONT_RIGHT_CAMERA_TO_CENTER = new Transform3d(
            new Translation3d(Units.inchesToMeters(1.635), Units.inchesToMeters(-11.241), Units.inchesToMeters(20.003)),
            new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(15), Units.degreesToRadians(15)));

        OBJECT_DETECTION_CAMERA_TO_CENTER = new Transform3d(
            new Translation3d(Units.inchesToMeters(-8.585), Units.inchesToMeters(0), Units.inchesToMeters(16.532)),
            new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-24), Units.degreesToRadians(180)));

            // LOWER_FRONT_RIGHT_CAMERA_TO_CENTER = new Transform3d(
            //     new Translation3d(Units.inchesToMeters(1.546), Units.inchesToMeters(-11.35), Units.inchesToMeters(20.25)),
            //     new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(10)));

        this.phoenixConstants = TunerConstantsCompBot.DrivetrainConstants;
        this.frontLeftConstants = TunerConstantsCompBot.FrontLeft;
        this.frontRightConstants = TunerConstantsCompBot.FrontRight;
        this.backLeftConstants = TunerConstantsCompBot.BackLeft;
        this.backRightConstants = TunerConstantsCompBot.BackRight;

        this.kFrontLeftEncoderOffset = TunerConstantsCompBot.kFrontLeftEncoderOffset;
        this.kFrontRightEncoderOffset = TunerConstantsCompBot.kFrontRightEncoderOffset;
        this.kBackLeftEncoderOffset = TunerConstantsCompBot.kBackLeftEncoderOffset;
        this.kBackRightEncoderOffset = TunerConstantsCompBot.kBackRightEncoderOffset;

        // Swerve Module Positions (relative to the center of the drive base)
        this.kVehicleToFrontLeft = new Translation2d(this.frontLeftConstants.LocationX, this.frontLeftConstants.LocationY);
        this.kVehicleToFrontRight = new Translation2d(this.frontRightConstants.LocationX, this.frontRightConstants.LocationY);
        this.kVehicleToBackRight = new Translation2d(this.backRightConstants.LocationX, this.backRightConstants.LocationY);
        this.kVehicleToBackLeft = new Translation2d(this.backLeftConstants.LocationX, this.backLeftConstants.LocationY);

        this.kWheelbaseLength = kVehicleToFrontLeft.getDistance(kVehicleToBackLeft); 
        this.kWheelbaseWidth = kVehicleToFrontLeft.getDistance(kVehicleToFrontRight);
        this.kWheelRadius = TunerConstantsCompBot.kWheelRadius;

        this.kinematics =
            new SwerveDriveKinematics(
                new Translation2d[] {
                    kVehicleToFrontLeft,
                    kVehicleToFrontRight,
                    kVehicleToBackLeft,
                    kVehicleToBackRight});

        this.autoPIDController =
            AutoPIDController.builder()
                .autoXController(
                    new PIDController(0.95, 0.0, 0.0))
                .autoYController(
                    new PIDController(0.905, 0.0, 0.0))
                .autoThetaController(
                    new PIDController(5.0, 0.0, 0.0))
                .build();
    }    
}