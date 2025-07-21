package org.frogforce503.robot2025.subsystems.drive;

import org.frogforce503.robot2025.Robot;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class SwerveVisualizer {
    private MechanismLigament2d[] m_moduleSpeeds;
    private MechanismLigament2d robotDirection;

    private final double maxSpeed;
    
    // Display
    private final double visualSize = 3;
    private final double fillPercent = 0.75;

    private final Color8Bit stationaryColor = new Color8Bit(Color.kWhite);
    private final Color8Bit speedColor = new Color8Bit(Color.kLightGreen);
    private final Color8Bit maxColor = new Color8Bit(Color.kGreen);

    // FL, FR, BL, BR
    final Color8Bit[] moduleColors =
        new Color8Bit[] {
            new Color8Bit(Color.kGreen),
            new Color8Bit(Color.kBlue),
            new Color8Bit(Color.kRed),
            new Color8Bit(Color.kYellow)};

    public SwerveVisualizer(double maxSpeed) {
        this.maxSpeed = maxSpeed;
    
        /* Mechanisms to represent the swerve module states */
        Mechanism2d visual =
            new Mechanism2d(
                visualSize,
                visualSize,
                new Color8Bit(Color.kBlack));

        double smallestDimension = Math.min(Robot.bot.kWheelbaseLength, Robot.bot.kWheelbaseWidth);
        double width = (Robot.bot.kWheelbaseWidth / smallestDimension) * fillPercent * visualSize;
        double length = (Robot.bot.kWheelbaseLength / smallestDimension) * fillPercent * visualSize;

        Translation2d bottomLeft = new Translation2d((visualSize - width) / 2, (visualSize - length) / 2);
        Translation2d topRight = new Translation2d((visualSize + width) / 2, (visualSize + length) / 2);

        MechanismLigament2d speedVectorDisplay =
            new MechanismLigament2d("Speed", 0.5, 0, 10, speedColor);

        m_moduleSpeeds =
            new MechanismLigament2d[] {
                visual
                    .getRoot("FL_Speed", bottomLeft.getX(), topRight.getY())
                    .append(speedVectorDisplay),
                visual
                    .getRoot("FR_Speed", topRight.getX(), topRight.getY())
                    .append(speedVectorDisplay),
                visual
                    .getRoot("BL_Speed", bottomLeft.getX(), bottomLeft.getY())
                    .append(speedVectorDisplay),
                visual
                    .getRoot("BR_Speed", topRight.getX(), bottomLeft.getY())
                    .append(speedVectorDisplay)};

        robotDirection =
            visual
                .getRoot("compass", visualSize / 2, visualSize / 2)
                .append(new MechanismLigament2d("Angle", 0.5, 0, 25, new Color8Bit(Color.kYellow)));

        SmartDashboard.putData("Swerve/Visualizer", visual);
    }

    public void update(SwerveModuleState[] moduleStates, Rotation2d angle) {
        for (int i = 0; i < 4; ++i) {
            boolean moving = moduleStates[i].speedMetersPerSecond > Units.inchesToMeters(3);
            m_moduleSpeeds[i].setColor(moduleColors[i]);
            m_moduleSpeeds[i].setColor(moving ? (moduleStates[i].speedMetersPerSecond >= maxSpeed - 0.2 ? maxColor : speedColor) : stationaryColor);
            m_moduleSpeeds[i].setAngle(moduleStates[i].angle.plus(new Rotation2d(Math.PI / 2)));
            m_moduleSpeeds[i].setLength(moduleStates[i].speedMetersPerSecond / (2 * maxSpeed) + 0.05);
        }
        
        robotDirection.setAngle(angle.plus(new Rotation2d(Math.PI / 2)));
    }

    public void displayModulePoses(Translation2d currentTranslation, Rotation2d currentRotation) {
        Translation2d fl = Robot.bot.kVehicleToFrontLeft.rotateBy(currentRotation).plus(currentTranslation);
        Translation2d fr = Robot.bot.kVehicleToFrontRight.rotateBy(currentRotation).plus(currentTranslation);
        Translation2d bl = Robot.bot.kVehicleToBackLeft.rotateBy(currentRotation).plus(currentTranslation);
        Translation2d br = Robot.bot.kVehicleToBackRight.rotateBy(currentRotation).plus(currentTranslation);

        Rotation3d angle = new Rotation3d(0, -Math.PI/2, currentRotation.getRadians());

        Logger.recordOutput("Swerve/State/ModulePoses",
            new Pose3d[] {
                new Pose3d(fl.getX(), fl.getY(), 0.5, angle),
                new Pose3d(fr.getX(), fr.getY(), 0.5, angle),
                new Pose3d(bl.getX(), bl.getY(), 0.5, angle),
                new Pose3d(br.getX(), br.getY(), 0.5, angle)});

        Translation2d upperLeft =
            Robot.bot.kVehicleToFrontLeft
                .plus(
                    new Translation2d(Units.inchesToMeters(10), 0))
                .rotateBy(currentRotation)
                .plus(currentTranslation);

        Translation2d upperRight =
            Robot.bot.kVehicleToFrontRight
                .plus(
                    new Translation2d(Units.inchesToMeters(10), 0))
                .rotateBy(currentRotation)
                .plus(currentTranslation);

        Logger.recordOutput("Swerve/State/CornerPoses",
            new Pose3d[] {
                new Pose3d(upperLeft.getX(), upperLeft.getY(), 0.5, angle),
                new Pose3d(upperRight.getX(), upperRight.getY(), 0.5, angle)});
    }
}
