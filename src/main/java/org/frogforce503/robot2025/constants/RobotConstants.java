package org.frogforce503.robot2025.constants;

import org.frogforce503.lib.swerve.SwervePathFollower;
import org.frogforce503.robot2025.constants.subsystem.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public abstract class RobotConstants {
    public DriveConfig Drive;
    public VisionConfig Vision;

    public ElevatorConfig Elevator;
    public ArmConfig Arm;
    public WristConfig Wrist;
    public ClawConfig Claw;
    public IntakeConfig Intake;
    public ClimberConfig Climber;

    public SensorConfig Sensors;
    public LedsConfig Leds;
    
    // Swerve Module Positions (relative to the center of the drive base)
    public Translation2d kVehicleToFrontLeft;
    public Translation2d kVehicleToFrontRight;
    public Translation2d kVehicleToBackRight;
    public Translation2d kVehicleToBackLeft;

    public double kWheelbaseLength;
    public double kWheelbaseWidth;

    // Swerve Kinematics
    public SwerveDriveKinematics kinematics;

    // Path Follower
    public SwervePathFollower swervePathFollower;
}