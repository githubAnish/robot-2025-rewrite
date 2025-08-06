package org.frogforce503.robot2025.hardware;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;

import org.frogforce503.lib.auto.follower.AutoPIDController;
import org.frogforce503.lib.math.Range;
import org.frogforce503.lib.motorcontrol.tuning.pidf.PIDFConfig;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Distance;
import lombok.Builder;

public abstract class RobotHardware {
    public abstract void initializeConstants();

    // Camera positions relative to center of robot (insert below) as Transform3d
    public Transform3d FRONT_LEFT_CAMERA_TO_CENTER;
    public Transform3d FRONT_RIGHT_CAMERA_TO_CENTER;
    public Transform3d ELEVATOR_BACK_CAMERA_TO_CENTER;
    public Transform3d LOWER_FRONT_RIGHT_CAMERA_TO_CENTER;
    public Transform3d OBJECT_DETECTION_CAMERA_TO_CENTER;

    // Sensor Constants
    public SensorConstants sensorConstants;

    @Builder
    public record SensorConstants(
        int elevatorZeroSwitchID,
        int lowerBeamID,
        int upperBeamID,
        int winchSwitchID) {}
    
    // Subsystem-Based Constants
    public ElevatorConstants elevatorConstants;

    @Builder
    public record ElevatorConstants(
        int elevatorID,
        boolean elevatorInverted,
        PIDFConfig kPIDF,
        Constraints kConstraints,
        Range range) {}

    public ArmConstants armConstants;

    @Builder
    public record ArmConstants(
        int armID,
        boolean armInverted,
        double armOffset,
        PIDFConfig kPIDF,
        Constraints kConstraints,
        Range range) {}

    public WristConstants wristConstants;

    @Builder
    public record WristConstants(
        int wristID,
        boolean wristInverted,
        double wristOffset,
        PIDFConfig kPIDF,
        Range range) {}

    public ClawConstants clawConstants;

    @Builder
    public record ClawConstants(
        ClawMotorConstants leftMotorConstants,
        ClawMotorConstants rightMotorConstants) {}

    @Builder
    public record ClawMotorConstants(
        int motorID,
        boolean motorInverted,
        PIDFConfig kPIDF) {}

    public IntakeConstants intakeConstants;

    @Builder
    public record IntakeConstants(
        PivotConstants pivotConstants,
        RollerConstants rollerConstants) {}

    @Builder
    public record PivotConstants(
        int pivotID,
        boolean pivotInverted,
        double pivotOffset,
        PIDFConfig kPIDF,
        Constraints kConstraints,
        Range range) {}

    @Builder
    public record RollerConstants(
        int rollerID,
        boolean rollerInverted,
        boolean rollerIsSparkFlex,
        PIDFConfig kPIDF) {}

    public ClimberConstants climberConstants;

    @Builder
    public record ClimberConstants(
        int winchID,
        boolean winchInverted,
        PIDFConfig kPIDF) {}

    public int candleID;

    // Swerve Kinematics
    public SwerveDriveKinematics kinematics;

    // CTRE Drivetrain Constants
    public SwerveDrivetrainConstants phoenixConstants;
    public SwerveModuleConstants<?, ?, ?> frontLeftConstants;
    public SwerveModuleConstants<?, ?, ?> backLeftConstants;
    public SwerveModuleConstants<?, ?, ?> frontRightConstants;
    public SwerveModuleConstants<?, ?, ?> backRightConstants;
    
    // Swerve Calculations Constants (measurements are in inches)
    public double kWheelbaseLength;
    public double kWheelbaseWidth;
    public Distance kWheelRadius;

    // Swerve Module Positions (relative to the center of the drive base)
    public Translation2d kVehicleToFrontRight;
    public Translation2d kVehicleToBackRight;
    public Translation2d kVehicleToFrontLeft;
    public Translation2d kVehicleToBackLeft;

    // Auto Following PID
    public AutoPIDController autoPIDController;

    /**
     * @return the MAC address of the robot
     */
    public static String getMACAddress() {
        try {
            Enumeration<NetworkInterface> nwInterface = NetworkInterface.getNetworkInterfaces();
            StringBuilder ret = new StringBuilder();
            while (nwInterface.hasMoreElements()) {
                NetworkInterface nis = nwInterface.nextElement();
                if (nis != null) {
                    byte[] mac = nis.getHardwareAddress();
                    if (mac != null) {
                        for (int i = 0; i < mac.length; i++) {
                            ret.append(String.format("%02X%s", mac[i], (i < mac.length - 1) ? "-" : ""));
                        }
                        return ret.toString();
                    } else {
                        System.out.println("Address doesn't exist or is not accessible");
                    }
                } else {
                    System.out.println("Network Interface for the specified address is not found.");
                }
            }
        } catch (SocketException e) {
            e.printStackTrace();
        } catch (NullPointerException e) {
            e.printStackTrace();
        }
        return "";
    }
}