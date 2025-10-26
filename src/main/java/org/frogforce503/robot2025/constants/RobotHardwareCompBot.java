package org.frogforce503.robot2025.constants;

import org.frogforce503.lib.math.Range;
import org.frogforce503.lib.motorcontrol.tuning.pidf.PIDFConfig;
import org.frogforce503.robot2025.constants.subsystem.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public class RobotHardwareCompBot implements RobotHardware {
    @Override
    public ElevatorConfig getElevatorConfig() {
        return
            ElevatorConfig.builder()
                .elevatorID(2)
                .elevatorInverted(false)
                .kPIDF(new PIDFConfig(2.0, 0.0, 0.04, new ElevatorFeedforward(0.01, 0.0, 0.0, 0.002)))
                .kConstraints(new Constraints(5200.0, 600.0))
                .range(new Range(0.0, 33.0))
                .build();

    }

    @Override
    public ArmConfig getArmConfig() {
        return
            ArmConfig.builder()
                .armID(6)
                .armInverted(false)
                .armOffset(0.893)
                .kPIDF(new PIDFConfig(0.014, 0.0, 0.1, new ArmFeedforward(0.0, 0.75, 0.0, 0.0)))
                .kConstraints(new Constraints(1, 0.5))
                .range(new Range(0.0, 180.0))
                .build();
        
    }

    @Override
    public WristConfig getWristConfig() {
        return
            WristConfig.builder()
                .wristID(7)
                .wristInverted(true)
                .wristOffset(0.187)
                .kPIDF(new PIDFConfig(0.014, 0.0, 0.0, new ArmFeedforward(0.0, 0.0, 0.0, 0.0)))
                .range(new Range(0.0, 300.0))
                .build();
    }

    @Override
    public ClawConfig getClawConfig() {
        return
            ClawConfig.builder()
                .leftMotorID(5)
                .leftMotorInverted(true)
                .leftMotorPIDF(new PIDFConfig(0.0001, 0.0, 0.00005, 0.0, 0.0, 0.000096, 0.0))

                .rightMotorID(8)
                .rightMotorInverted(false)
                .rightMotorPIDF(new PIDFConfig(0.0001, 0.0, 0.00005, 0.0, 0.0, 0.000096, 0.0))

                .build();
    }

    @Override
    public IntakeConfig getIntakeConfig() {
        return
            IntakeConfig.builder()
                .pivotID(3)
                .pivotInverted(false)
                .pivotOffset(0.4)
                .pivotPIDF(new PIDFConfig(0.01, 0.0, 0.0, new ArmFeedforward(0.0, 0.75, 0.0, 0.0)))
                .pivotConstraints(new Constraints(1000, 500))
                .pivotRange(new Range(35, 193))

                .rollerID(4)
                .rollerInverted(true)
                .rollerIsSparkFlex(true)
                .rollerPIDF(new PIDFConfig(0.000001, 0.0, 0.0, 0.0, 0.0, 0.0001575, 0.0))
                
                .build();
    }

    @Override
    public ClimberConfig getClimberConfig() {
        return
            ClimberConfig.builder()
                .winchID(9)
                .winchInverted(false)
                .kPIDF(new PIDFConfig(0.0, 0.0, 0.0))
                .build();
    }

    @Override
    public SensorConfig getSensorsConfig() {
        return
            SensorConfig.builder()
                .elevatorZeroSwitchID(1)
                .lowerBeamID(0)
                .upperBeamID(3)
                .winchSwitchID(2)
                .build();
    }

    @Override
    public LedsConfig getLedsConfig() {
        return
            LedsConfig.builder()
                .candleID(11)
                .build();
    }

    @Override
    public VisionConfig getVisionConfig() {
        return
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

                .build();
    }

    @Override
    public DriveConfig getDriveConfig() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getDriveConfig'");
    }
}