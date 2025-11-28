package org.frogforce503.robot2025.subsystems.drive;

import org.frogforce503.lib.math.MathUtils;
import org.frogforce503.lib.swerve.SwervePathFollower;
import org.frogforce503.robot2025.Robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class DriveConstants {
    private static final Translation2d frontLeftModuleTranslation = new Translation2d(Robot.bot.getDriveConfig().frontLeft().LocationX, Robot.bot.getDriveConfig().frontLeft().LocationY);
    private static final Translation2d frontRightModuleTranslation = new Translation2d(Robot.bot.getDriveConfig().frontRight().LocationX, Robot.bot.getDriveConfig().frontRight().LocationY);
    private static final Translation2d backLeftModuleTranslation = new Translation2d(Robot.bot.getDriveConfig().backLeft().LocationX, Robot.bot.getDriveConfig().backLeft().LocationY);
    private static final Translation2d backRightModuleTranslation = new Translation2d(Robot.bot.getDriveConfig().backRight().LocationX, Robot.bot.getDriveConfig().backRight().LocationY);

    public static final double trackWidthX = frontLeftModuleTranslation.getDistance(backLeftModuleTranslation);
    public static final double trackWidthY = frontLeftModuleTranslation.getDistance(frontRightModuleTranslation);

    public static final double driveBaseRadius =
        MathUtils.max(
            frontLeftModuleTranslation.getNorm(),
            frontRightModuleTranslation.getNorm(),
            backLeftModuleTranslation.getNorm(),
            backRightModuleTranslation.getNorm());
    
    public static final Translation2d CENTER_OF_ROTATION = Translation2d.kZero;

    public static final double maxLinearSpeed = Robot.bot.getDriveConfig().frontLeft().SpeedAt12Volts;
    public static final double maxOmega = maxLinearSpeed / driveBaseRadius;
    public static final double slowModeLinearSpeed = 1.5;
    public static final double slowModeOmega = Math.PI;

    public static final SwerveDriveKinematics kinematics =
        new SwerveDriveKinematics(
            new Translation2d[] {
                frontLeftModuleTranslation,
                frontRightModuleTranslation,
                backLeftModuleTranslation,
                backRightModuleTranslation});

    public static final SwervePathFollower pathFollower =
        new SwervePathFollower(
            Robot.bot.xPID.toPIDController(),
            Robot.bot.yPID.toPIDController(),
            Robot.bot.thetaPID.toPIDController());
}