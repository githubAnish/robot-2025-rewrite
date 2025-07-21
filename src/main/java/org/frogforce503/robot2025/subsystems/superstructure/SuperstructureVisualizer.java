package org.frogforce503.robot2025.subsystems.superstructure;

import java.util.function.Supplier;

import org.frogforce503.robot2025.subsystems.superstructure.claw.Claw;
import org.frogforce503.robot2025.subsystems.superstructure.claw.Claw.ClawGoal;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

public class SuperstructureVisualizer {
    private final Claw claw;
    private final Supplier<Pose2d> robotPoseSupplier;
    
    private final String name;

    private Pose3d armPose, intakePose, movingStagePose, gearboxPose;

    private final Translation3d armPos = new Translation3d(0, 0.00, 1);
    private final Translation3d intakePos = new Translation3d(0.305, 0, .23);

    private LoggedNetworkBoolean simulationHasAlgae;
    private LoggedNetworkBoolean simulationHasCoral;
    
    public SuperstructureVisualizer(String name, Claw claw, Supplier<Pose2d> robotPoseSupplier) {
        this.claw = claw;
        this.robotPoseSupplier = robotPoseSupplier;

        this.name = name;

        simulationHasAlgae = new LoggedNetworkBoolean("Simulation/" + name + "/Simulation Has Algae");
        simulationHasCoral = new LoggedNetworkBoolean("Simulation/" + name + "/Simulation Has Coral");
        
        armPose = new Pose3d();
        intakePose = new Pose3d();
        movingStagePose = new Pose3d(new Translation3d(), new Rotation3d(0, 0, Math.PI/2));
        gearboxPose = new Pose3d();
    }

    public void updateOnlyIfInSimulation(double elevatorPosition, double armPosition, double wristPosition, double intakePosition) {
        if (RobotBase.isSimulation()) {
            update(elevatorPosition, armPosition, wristPosition, intakePosition);
        }
    }

    public void update(double elevatorPosition, double armPosition, double wristPosition, double intakePosition) {
        Pose3d drivePose3d = new Pose3d(robotPoseSupplier.get());

        // Main Robot Poses
        var stagePose = movingStagePose.plus(
            new Transform3d(
                new Translation3d(0, 0, Units.degreesToRadians(elevatorPosition)),
                new Rotation3d()));
                
        gearboxPose = stagePose;

        Rotation3d armRoot =
            new Rotation3d(Units.degreesToRadians(-armPosition) + Math.PI/2, 0, Math.PI/2);

        armPose =
            new Pose3d(
                armPos.plus(
                    new Translation3d(0, 0, Units.degreesToRadians(elevatorPosition))),
                    armRoot);

        var wristRotation =
            new Rotation3d(Units.degreesToRadians(wristPosition) + Math.PI, 0.0, 0.0);

        var clawPose =
            armPose
                .transformBy(
                    new Transform3d(
                        new Translation3d(0, -0.51, 0),
                        wristRotation));

        Rotation3d intakeRoot =
            new Rotation3d(Units.degreesToRadians(-intakePosition) + Units.degreesToRadians(180 + 60), 0, Math.PI/2);

        intakePose = new Pose3d(intakePos, intakeRoot);

        Logger.recordOutput("Simulation/" + name + "/Mechanism", armPose, clawPose, intakePose, stagePose, gearboxPose);

        // Gamepiece Held
        handleClawGoalLogic(claw::getCurrentGoal);

        if (simulationHasAlgae.get()) {
            Pose3d heldAlgaePose =
                drivePose3d
                    .transformBy(
                        new Transform3d(
                            clawPose.getTranslation()
                                .plus(
                                    new Translation3d(
                                        Units.inchesToMeters(7.5),
                                        new Rotation3d(0.0, clawPose.getRotation().getX(), 0.0))),
                            new Rotation3d()));

            Logger.recordOutput("Simulation/" + name + "/HeldAlgae", heldAlgaePose);
        }

        if (simulationHasCoral.get()) {
            Pose3d heldCoralPose =
                drivePose3d
                    .transformBy(
                        new Transform3d(
                            clawPose.getTranslation(),
                            new Rotation3d(0.0, clawPose.getRotation().getX() + Math.PI/2, 0.0)));

            Logger.recordOutput("Simulation/" + name + "/HeldCoral", heldCoralPose);
        }
    }

    public void setupAuto() {
        simulationHasCoral.set(true);
    }

    private void handleClawGoalLogic(Supplier<ClawGoal> goalSupplier) {
        switch (goalSupplier.get()) {
            case INTAKE_CORAL -> simulationHasCoral.set(true);
            case EJECT_CORAL -> simulationHasCoral.set(false);
            case EJECT_ALGAE -> simulationHasAlgae.set(false);
            default -> {}
        }
    }
}