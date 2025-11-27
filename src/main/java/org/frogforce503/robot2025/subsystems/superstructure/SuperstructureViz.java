package org.frogforce503.robot2025.subsystems.superstructure;

import java.util.function.Supplier;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public class SuperstructureViz {
    private final Superstructure superstructure;
    private final Supplier<Pose2d> robotPoseSupplier;

    private final LoggedNetworkBoolean hasCoral = new LoggedNetworkBoolean("SuperstructureViz/HasCoral");
    
    public SuperstructureViz(Superstructure superstructure, Supplier<Pose2d> robotPoseSupplier) {
        this.superstructure = superstructure;
        this.robotPoseSupplier = robotPoseSupplier;
    }

    public void setHasCoral(boolean choice) {
        this.hasCoral.set(choice);
    }

    public void update(double elevatorHeightMeters, double armAngleRad, double wristAngleRad, double intakePivotAngleRad) {
        Pose3d drivePose3d = new Pose3d(robotPoseSupplier.get());
    }
}