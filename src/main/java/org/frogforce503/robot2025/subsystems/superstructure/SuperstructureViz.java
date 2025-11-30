package org.frogforce503.robot2025.subsystems.superstructure;

import java.util.function.Supplier;

import org.frogforce503.robot2025.subsystems.superstructure.arm.ArmConstants;
import org.frogforce503.robot2025.subsystems.superstructure.intakepivot.IntakePivotConstants;
import org.frogforce503.robot2025.subsystems.superstructure.wrist.WristConstants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class SuperstructureViz {
    // Requirements
    private final Superstructure superstructure;
    private final Supplier<Pose2d> robotPoseSupplier;

    // Constants
    private final double distElevatorBaseToIntakePivotInches = 14.5;
    private final Translation2d elevator2dOrigin = new Translation2d(Units.inchesToMeters(15), Units.inchesToMeters(15)); // Base of Elevator
    private final Translation2d intakePivot2dOrigin = new Translation2d(Units.inchesToMeters(15 + distElevatorBaseToIntakePivotInches), Units.inchesToMeters(15)); // Base of Intake Pivot

    private final double elevator2dInitialLengthMeters = Units.inchesToMeters(28.0);
    private final double arm2dLengthMeters = Units.inchesToMeters(18.0);
    private final double wrist2dLengthMeters = Units.inchesToMeters(4.5);
    private final double intakePivot2dLengthMeters = Units.inchesToMeters(14.75);

    private final double arm2dAngleOffsetRad = Units.degreesToRadians(-85);
    private final double wrist2dAngleOffsetRad = Units.degreesToRadians(180);

    // State
    private final LoggedNetworkBoolean hasCoral = new LoggedNetworkBoolean("SuperstructureViz/HasCoral");

    // 2D Visualization
    private final LoggedMechanism2d mechanism2d =
        new LoggedMechanism2d(
            Units.inchesToMeters(50),
            Units.inchesToMeters(80));

    private final LoggedMechanismLigament2d elevatorLigament;
    private final LoggedMechanismLigament2d armLigament;
    private final LoggedMechanismLigament2d wristLigament;
    
    private final LoggedMechanismLigament2d intakePivotLigament;
    
    public SuperstructureViz(Superstructure superstructure, Supplier<Pose2d> robotPoseSupplier) {
        this.superstructure = superstructure;
        this.robotPoseSupplier = robotPoseSupplier;

        // Setup 2D Viz
        LoggedMechanismRoot2d mainRoot =
            mechanism2d.getRoot(
                "Main Root", elevator2dOrigin.getX(), elevator2dOrigin.getY());
            
        elevatorLigament =
            mainRoot.append(
                new LoggedMechanismLigament2d(
                    "Elevator",
                    elevator2dInitialLengthMeters, // Initial minimum length
                    90.0, // Fixed vertical angle
                    4.0, // Line width
                    new Color8Bit(Color.kFirstBlue)));
                    
        armLigament =
            elevatorLigament.append(
                new LoggedMechanismLigament2d(
                    "Arm",
                    arm2dLengthMeters,
                    Units.radiansToDegrees(ArmConstants.START), // Initial relative angle (set in update)
                    4.0,
                    new Color8Bit(Color.kFirstRed)));

        wristLigament =
            armLigament.append(
                new LoggedMechanismLigament2d(
                    "Wrist",
                    wrist2dLengthMeters,
                    Units.radiansToDegrees(WristConstants.START), // Initial relative angle (set in update)
                    4.0,
                    new Color8Bit(Color.kGreen)));

        LoggedMechanismRoot2d intakeRoot =
            mechanism2d.getRoot(
                "Intake Root", intakePivot2dOrigin.getX(), intakePivot2dOrigin.getY());
                
        intakePivotLigament = 
            intakeRoot.append(
                new LoggedMechanismLigament2d(
                    "Intake Pivot",
                    intakePivot2dLengthMeters,
                    Units.radiansToDegrees(IntakePivotConstants.START), // Initial absolute angle (set in update)
                    4.0,
                    new Color8Bit(Color.kYellow)));

        // Setup 3D Viz
    }

    public void setHasCoral(boolean choice) {
        this.hasCoral.set(choice);
    }

    public void update(double elevatorHeightMeters, double armAngleRad, double wristAngleRad, double intakePivotAngleRad) {
        Pose3d drivePose3d = new Pose3d(robotPoseSupplier.get());

        update2dViz(elevatorHeightMeters, armAngleRad, wristAngleRad, intakePivotAngleRad);
        update3dViz(elevatorHeightMeters, armAngleRad, wristAngleRad, intakePivotAngleRad);
    }

    private void update2dViz(double elevatorHeightMeters, double armAngleRad, double wristAngleRad, double intakePivotAngleRad) {
        elevatorLigament.setLength(elevator2dInitialLengthMeters + elevatorHeightMeters);
        armLigament.setAngle(Units.radiansToDegrees(armAngleRad + arm2dAngleOffsetRad));
        wristLigament.setAngle(Units.radiansToDegrees(wristAngleRad + wrist2dAngleOffsetRad));

        intakePivotLigament.setAngle(Units.radiansToDegrees(intakePivotAngleRad));

        Logger.recordOutput("SuperstructureViz/2D", mechanism2d);
    }

    private void update3dViz(double elevatorHeightMeters, double armAngleRad, double wristAngleRad, double intakePivotAngleRad) {
        Logger.recordOutput("SuperstructureViz/3D/Components", Pose3d.kZero, Pose3d.kZero, Pose3d.kZero, Pose3d.kZero);
    }
}