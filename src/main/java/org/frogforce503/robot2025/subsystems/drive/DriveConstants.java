package org.frogforce503.robot2025.subsystems.drive;

import org.frogforce503.lib.math.MathUtils;
import org.frogforce503.lib.util.ErrorUtil;
import org.frogforce503.robot2025.Robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class DriveConstants {
    public static final double odometryFrequency = 250;

    public static final double dt = 0.02;

    public static final double driveBaseRadius = Math.hypot(Robot.bot.kWheelbaseLength / 2, Robot.bot.kWheelbaseWidth / 2);
    
    public static final Translation2d CENTER_OF_ROTATION = Translation2d.kZero;

    public static final double SLOW_TRANSLATION_METERS_PER_SECOND = 1.5;
    public static final double FAST_TRANSLATION_METERS_PER_SECOND = Robot.bot.frontLeftConstants.SpeedAt12Volts;
    public static final double FAST_ROTATION_RADIANS_PER_SECOND = Units.degreesToRadians(720);
    public static final double SUPER_FAST_ROTATION_RADIANS_PER_SECOND = FAST_ROTATION_RADIANS_PER_SECOND * 2.25; 

    public static final double SLOW_ROTATION_RADIANS_PER_SECOND = Units.degreesToRadians(360);

    public static final double MAX_ACCELERATION_METERS_PER_SEC_PER_SEC = FAST_TRANSLATION_METERS_PER_SECOND * 1.2;
    public static final double MAX_ANGULAR_ACCLERATION_RAD_PER_SEC_PER_SEC = 5 * Math.PI / 2.0;

    public enum ModuleName {
        FrontLeft(0, new Rotation2d(Robot.bot.kFrontLeftEncoderOffset)),
        FrontRight(1, new Rotation2d(Robot.bot.kFrontRightEncoderOffset)),
        BackLeft(2, new Rotation2d(Robot.bot.kBackLeftEncoderOffset)),
        BackRight(3, new Rotation2d(Robot.bot.kBackRightEncoderOffset));

        public final int moduleIndex;
        public final Rotation2d encoderOffset;

        private ModuleName(int moduleIndex, Rotation2d encoderOffset) {
            this.moduleIndex = moduleIndex;
            this.encoderOffset = encoderOffset;
        }

        public static ModuleName fromIndex(int index) {
            if (!MathUtils.inRange(index, 0, 3)) {
                throw new IllegalArgumentException("Module index must be between 0 and 3, inclusive." + ErrorUtil.attachJavaClassName(ModuleName.class));
            }

            return values()[index];
        }
    }
}