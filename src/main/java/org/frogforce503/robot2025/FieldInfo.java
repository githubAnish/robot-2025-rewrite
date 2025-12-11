package org.frogforce503.robot2025;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lombok.Setter;

/** Class containing all field-related information. */
public final class FieldInfo {
    // Requirements
    private static final Field2d field2d = new Field2d();

    // State
    @Setter private static Alliance allianceOverride;

    private FieldInfo() {}

    static {
        SmartDashboard.putData("Field", field2d);
    }

    /** Returns current alliance. */
    public static Alliance getAlliance() {
        return
            RobotBase.isSimulation() || DriverStation.getAlliance().isEmpty() // if in sim or alliance not known
                ? allianceOverride
                : DriverStation.getAlliance().get();
    }

    /** Returns if currently on red alliance. */
    public static boolean isRed() {
        return getAlliance() == Alliance.Red;
    }

    /** Sets the robot pose on Field2d. */
    public static void setRobotPose(Pose2d robotPose) {
        field2d.setRobotPose(robotPose);
    }

    /** Gets or creates a field object on Field2d. */
    public static FieldObject2d getObject(String name) {
        return field2d.getObject(name);
    }
}