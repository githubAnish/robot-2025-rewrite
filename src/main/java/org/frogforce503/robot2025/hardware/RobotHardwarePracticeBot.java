package org.frogforce503.robot2025.hardware;


import org.frogforce503.lib.auto.follower.AutoPIDController;
import org.frogforce503.lib.math.Range;
import org.frogforce503.lib.motorcontrol.tuning.pidf.PIDFConfig;
import org.frogforce503.robot2025.hardware.tunerconstants.TunerConstantsPracticeBot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public class RobotHardwarePracticeBot extends RobotHardware {
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
                .kPIDF(new PIDFConfig())
                .kConstraints(new Constraints(1.0, 1.0))
                .range(new Range(0.0, 0.0))
                .build();

        // Arm
        this.armConstants =
            ArmConstants.builder()
                .armID(6)
                .armInverted(false)
                .armOffset(0.0)
                .kPIDF(new PIDFConfig())
                .kConstraints(new Constraints(1.0, 1.0))
                .range(new Range(0.0, 0.0))
                .build();
        
        // Wrist
        this.wristConstants =
            WristConstants.builder()
                .wristID(7)
                .wristInverted(false)
                .wristOffset(0.0)
                .kPIDF(new PIDFConfig())
                .range(new Range(0.0, 0.0))
                .build();

        // Claw
        this.clawConstants =
            ClawConstants.builder()
                .leftMotorConstants(
                    ClawMotorConstants.builder()
                        .motorID(5)
                        .motorInverted(false)
                        .kPIDF(new PIDFConfig())
                        .build())
                .rightMotorConstants(
                    ClawMotorConstants.builder()
                        .motorID(8)
                        .motorInverted(false)
                        .kPIDF(new PIDFConfig())
                        .build())
                .build();

        // Intake
        this.intakeConstants =
            IntakeConstants.builder()
                .pivotConstants(
                    PivotConstants.builder()
                        .pivotID(3)
                        .pivotInverted(false)
                        .pivotOffset(0.0)
                        .kPIDF(new PIDFConfig())
                        .kConstraints(new Constraints(1.0, 1.0))
                        .range(new Range(0.0, 0.0))
                        .build())
                .rollerConstants(
                    RollerConstants.builder()
                        .rollerID(4)
                        .rollerInverted(false)
                        .rollerIsSparkFlex(false)
                        .kPIDF(new PIDFConfig())
                        .build())
                .build();

        // Climber
        this.climberConstants =
            ClimberConstants.builder()
                .winchID(9)
                .winchInverted(false)
                .kPIDF(new PIDFConfig())
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
            new Translation3d(Units.inchesToMeters(1.288), Units.inchesToMeters(11.525), Units.inchesToMeters(19.956)),
            new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(15), Units.degreesToRadians(-15)));

        FRONT_RIGHT_CAMERA_TO_CENTER = new Transform3d(
            new Translation3d(Units.inchesToMeters(1.288), Units.inchesToMeters(-11.525), Units.inchesToMeters(31.721)),
            new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(30), Units.degreesToRadians(10)));
 
        ELEVATOR_BACK_CAMERA_TO_CENTER = new Transform3d(
            new Translation3d(Units.inchesToMeters(-8.670), Units.inchesToMeters(0), Units.inchesToMeters(41.074)),
            new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-20), Units.degreesToRadians(180)));

        LOWER_FRONT_RIGHT_CAMERA_TO_CENTER = new Transform3d(
            new Translation3d(Units.inchesToMeters(-5.330), Units.inchesToMeters(0), Units.inchesToMeters(41.074)),
            new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-20), Units.degreesToRadians(0)));

        OBJECT_DETECTION_CAMERA_TO_CENTER = new Transform3d(
            new Translation3d(Units.inchesToMeters(-8.585), Units.inchesToMeters(0), Units.inchesToMeters(16.532)),
            new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-24), Units.degreesToRadians(180)));


            // LOWER_FRONT_RIGHT_CAMERA_TO_CENTER = new Transform3d(
            //     new Translation3d(Units.inchesToMeters(1.546), Units.inchesToMeters(-11.35), Units.inchesToMeters(20.25)),
            //     new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(10)));

        this.phoenixConstants = TunerConstantsPracticeBot.DrivetrainConstants;
        this.frontLeftConstants = TunerConstantsPracticeBot.FrontLeft;
        this.frontRightConstants = TunerConstantsPracticeBot.FrontRight;
        this.backLeftConstants = TunerConstantsPracticeBot.BackLeft;
        this.backRightConstants = TunerConstantsPracticeBot.BackRight;

        this.kFrontLeftEncoderOffset = TunerConstantsPracticeBot.kFrontLeftEncoderOffset;
        this.kFrontRightEncoderOffset = TunerConstantsPracticeBot.kFrontRightEncoderOffset;
        this.kBackLeftEncoderOffset = TunerConstantsPracticeBot.kBackLeftEncoderOffset;
        this.kBackRightEncoderOffset = TunerConstantsPracticeBot.kBackRightEncoderOffset;

        // Swerve Module Positions (relative to the center of the drive base)
        this.kVehicleToFrontLeft = new Translation2d(this.frontLeftConstants.LocationX, this.frontLeftConstants.LocationY);
        this.kVehicleToFrontRight = new Translation2d(this.frontRightConstants.LocationX, this.frontRightConstants.LocationY);
        this.kVehicleToBackRight = new Translation2d(this.backRightConstants.LocationX, this.backRightConstants.LocationY);
        this.kVehicleToBackLeft = new Translation2d(this.backLeftConstants.LocationX, this.backLeftConstants.LocationY);

        this.kWheelbaseLength = kVehicleToFrontLeft.getDistance(kVehicleToBackLeft); 
        this.kWheelbaseWidth = kVehicleToFrontLeft.getDistance(kVehicleToFrontRight);
        this.kWheelRadius = TunerConstantsPracticeBot.kWheelRadius;

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
                    new PIDController(0.0, 0.0, 0.0))
                .autoYController(
                    new PIDController(0.0, 0.0, 0.0))
                .autoThetaController(
                    new PIDController(0.0, 0.0, 0.0))
                .build();
    }    
}