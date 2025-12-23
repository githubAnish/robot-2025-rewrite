package org.frogforce503.robot2025.subsystems.superstructure;

import java.util.Map;
import java.util.function.Supplier;

import org.frogforce503.lib.math.GeomUtil;
import org.frogforce503.robot2025.subsystems.superstructure.arm.ArmConstants;
import org.frogforce503.robot2025.subsystems.superstructure.intakepivot.IntakePivotConstants;
import org.frogforce503.robot2025.subsystems.superstructure.wrist.WristConstants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class SuperstructureViz {
    // Requirements
    private final Superstructure superstructure;
    private final Supplier<Pose2d> robotPoseSupplier;

    // 2D Viz Constants
    private final double distElevatorBaseToIntakePivotInches = 14.5;
    private final Translation2d elevator2dOrigin = new Translation2d(Units.inchesToMeters(15), Units.inchesToMeters(15)); // Base of Elevator
    private final Translation2d intakePivot2dOrigin = new Translation2d(Units.inchesToMeters(15 + distElevatorBaseToIntakePivotInches), Units.inchesToMeters(15)); // Base of Intake Pivot

    private final double elevator2dInitialLengthMeters = Units.inchesToMeters(28.0);
    private final double arm2dLengthMeters = Units.inchesToMeters(18.0);
    private final double wrist2dLengthMeters = Units.inchesToMeters(4.5);
    private final double intakePivot2dLengthMeters = Units.inchesToMeters(10);

    private final double arm2dAngleOffsetRad = Units.degreesToRadians(-85);
    private final double wrist2dAngleOffsetRad = Units.degreesToRadians(87);

    // 2D Viz Mechanisms
    private final LoggedMechanism2d mechanism2d =
        new LoggedMechanism2d(
            Units.inchesToMeters(50),
            Units.inchesToMeters(80));

    private final LoggedMechanismLigament2d elevatorLigament;
    private final LoggedMechanismLigament2d armLigament;
    private final LoggedMechanismLigament2d wristLigament;
    
    private final LoggedMechanismLigament2d intakePivotLigament;

    // 3D Viz Constants
    private final Transform3d robotToElevator = Transform3d.kZero;
    private final Transform3d elevatorCarriageToArm = new Transform3d(-0.0125, 0, 1.013, Rotation3d.kZero);
    private final Transform3d armToWrist = new Transform3d(-0.0235, 0, -0.508, Rotation3d.kZero);
    private final Transform3d robotToIntakePivot = new Transform3d(0.28, 0, 0.22, Rotation3d.kZero);
    private final Transform3d wristToCoral = new Transform3d(0, 0, 0, Rotation3d.kZero);
    private final Transform3d wristToAlgae = new Transform3d(0.25, 0, 0, Rotation3d.kZero);
    private final Transform3d intakePivotToAlgae = new Transform3d(0.12, 0, 0.2, Rotation3d.kZero);

    // 3D Viz Poses
    private Pose3d elevatorPose = Pose3d.kZero;
    private Pose3d armPose = Pose3d.kZero;
    private Pose3d wristPose = Pose3d.kZero;
    private Pose3d intakePivotPose = Pose3d.kZero;
    private Pose3d coralClawPose = Pose3d.kZero;
    private Pose3d algaeClawPose = Pose3d.kZero;
    private Pose3d algaeIntakePose = Pose3d.kZero;
    
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
    }

    public void update(
        double elevatorHeightMeters,
        double armAngleRad,
        double wristAngleRad,
        double intakePivotAngleRad,
        boolean hasCoral,
        boolean hasAlgaeInClaw,
        boolean hasAlgaeInIntake
    ) {
        Pose3d drivePose3d = new Pose3d(robotPoseSupplier.get());

        update2dViz(drivePose3d, elevatorHeightMeters, armAngleRad, wristAngleRad, intakePivotAngleRad, hasCoral, hasAlgaeInClaw, hasAlgaeInIntake);
        update3dViz(drivePose3d, elevatorHeightMeters, armAngleRad, wristAngleRad, intakePivotAngleRad, hasCoral, hasAlgaeInClaw, hasAlgaeInIntake);
    }

    private void update2dViz(
        Pose3d drivePose3d,
        double elevatorHeightMeters,
        double armAngleRad,
        double wristAngleRad,
        double intakePivotAngleRad,
        boolean hasCoral,
        boolean hasAlgaeInClaw,
        boolean hasAlgaeInIntake
    ) {
        // Convert global wrist angle to wrist angle relative to arm for viz
        double wristVisualAngleDeg = Units.radiansToDegrees(-WristKinematics.getWristAngleRelativeToArm(armAngleRad, wristAngleRad) + wrist2dAngleOffsetRad);

        // Update Ligaments
        elevatorLigament.setLength(elevator2dInitialLengthMeters + elevatorHeightMeters);
        armLigament.setAngle(Units.radiansToDegrees(arm2dAngleOffsetRad + armAngleRad));
        wristLigament.setAngle(wristVisualAngleDeg);
        intakePivotLigament.setAngle(Units.radiansToDegrees(intakePivotAngleRad));

        Logger.recordOutput("SuperstructureViz/2D", mechanism2d);
    }

    private void update3dViz(
        Pose3d drivePose3d,
        double elevatorHeightMeters,
        double armAngleRad,
        double wristAngleRad,
        double intakePivotAngleRad,
        boolean hasCoral,
        boolean hasAlgaeInClaw,
        boolean hasAlgaeInIntake
    ) {
        elevatorPose =
            Pose3d.kZero
                .plus(robotToElevator)
                .plus(new Transform3d(0, 0, elevatorHeightMeters, Rotation3d.kZero));

        armPose =
            elevatorPose
                .plus(elevatorCarriageToArm)
                .plus(new Transform3d(Translation3d.kZero, new Rotation3d(0, -armAngleRad - Math.PI/2, 0)));

        // Convert global wrist angle to wrist angle relative to arm for viz
        wristPose =
            armPose
                .plus(armToWrist)
                .plus(new Transform3d(Translation3d.kZero, new Rotation3d(0, WristKinematics.getWristAngleRelativeToArm(armAngleRad, wristAngleRad), 0)));

        intakePivotPose =
            Pose3d.kZero
                .plus(robotToIntakePivot)
                .plus(new Transform3d(Translation3d.kZero, new Rotation3d(0, -intakePivotAngleRad + Math.PI/2, 0)));
            
        if (hasCoral) {
            coralClawPose = drivePose3d.plus(GeomUtil.toTransform3d(wristPose.plus(wristToCoral)));
        }

        if (hasAlgaeInClaw) {
            algaeClawPose = drivePose3d.plus(GeomUtil.toTransform3d(wristPose.plus(wristToAlgae)));
        }

        if (hasAlgaeInIntake) {
            algaeIntakePose = drivePose3d.plus(GeomUtil.toTransform3d(intakePivotPose.plus(intakePivotToAlgae)));
        }

        Logger.recordOutput("SuperstructureViz/3D/Components", elevatorPose, armPose, wristPose, intakePivotPose);
        Logger.recordOutput("SuperstructureViz/3D/CoralClawPose", hasCoral ? new Pose3d[] { coralClawPose } : new Pose3d[] {});
        Logger.recordOutput("SuperstructureViz/3D/AlgaeClawPose", hasAlgaeInClaw ? new Pose3d[] { algaeClawPose } : new Pose3d[] {});
        Logger.recordOutput("SuperstructureViz/3D/AlgaeIntakePose", hasAlgaeInIntake ? new Pose3d[] { algaeIntakePose } : new Pose3d[] {});
    }

    /**
     * <p>
     * Computes the wrist angle relative to the arm.
     * Used for viz where the wrist must be expressed
     * in the arm’s reference frame instead of the global frame.
     * </p>
     * 
     * <p><b>
     * This is because the FF2025 robot has a 4-bar linkage with the arm and wrist, where the wrist angle is global (relative to ground),
     * while viz requires the wrist angle to be relative to the arm.
     * </b></p>
     * 
     * <p>
     * This solution was used instead of direct kinematics calculations because the relationship is nonlinear (and it's harder to do 4-bar kinematics).
     * </p>
     */
    public class WristKinematics {
        /**
         * Maps the arm angle (degrees, global frame) to the angle of wrist relative to the arm (degrees)
         * that corresponds to a zero global wrist angle.
         *
         * Values are interpolated for smooth motion.
         */
        private static final InterpolatingDoubleTreeMap lookupTable =
            InterpolatingDoubleTreeMap.ofEntries(
                Map.entry(-90.0, 0.0),
                Map.entry(-45.0, 45.0),
                Map.entry(0.0, 90.0),
                Map.entry(45.0, 135.0),
                Map.entry(90.0, 180.0)
            );

        /**
         * Returns the wrist angle relative to the arm.
         *
         * @param armAngleRad Arm angle in radians (global frame)
         * @param wristGlobalAngleRad Wrist angle in radians (global frame)
         * @return Wrist angle relative to the arm, in radians
         */
        public static double getWristAngleRelativeToArm(double armAngleRad, double wristGlobalAngleRad) {
            return
                Units.degreesToRadians(lookupTable.get(Units.radiansToDegrees(armAngleRad))) -
                wristGlobalAngleRad;
        }
    }
}