package org.frogforce503.robot2025.subsystems.drive;

import org.frogforce503.robot2025.Robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public final class DriveConstants {
    public static final double dt = 0.02;

    public static final double driveBaseRadius = Math.hypot(Robot.bot.kWheelbaseLength / 2, Robot.bot.kWheelbaseWidth / 2);
    
    public static final Translation2d CENTER_OF_ROTATION = Translation2d.kZero;

    public static final double SLOW_TRANSLATION_METERS_PER_SECOND = 1.5;
    public static final double FAST_TRANSLATION_METERS_PER_SECOND = Robot.bot.frontLeftConstants.SpeedAt12Volts;
    public static final double FAST_ROTATION_RADIANS_PER_SECOND = Units.degreesToRadians(720);
    public static final double SUPER_FAST_ROTATION_RADIANS_PER_SECOND = FAST_ROTATION_RADIANS_PER_SECOND * 2.25; 

    public static final double SLOW_ROTATION_RADIANS_PER_SECOND = Units.degreesToRadians(720);

    public static final double MAX_ACCELERATION_METERS_PER_SEC_PER_SEC = FAST_TRANSLATION_METERS_PER_SECOND * 1.2;
    public static final double MAX_ANGULAR_ACCLERATION_RAD_PER_SEC_PER_SEC = 5 * Math.PI / 2.0;

    public static final String[] moduleNames = { "FrontLeft", "FrontRight", "BackLeft", "BackRight" };

    public static final Matrix<N3, N1> stdDevs = VecBuilder.fill(0.8, 0.8, Units.degreesToRadians(30));
}