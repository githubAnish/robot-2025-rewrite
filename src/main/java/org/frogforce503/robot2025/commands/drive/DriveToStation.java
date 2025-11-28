// package org.frogforce503.robot2025.commands.drive;

// import java.util.function.Supplier;

// import org.frogforce503.lib.io.JoystickInputs;
// import org.frogforce503.lib.reefscape.Station;
// import org.frogforce503.lib.util.ProximityUtil;
// import org.frogforce503.robot2025.FieldInfo;
// import org.frogforce503.robot2025.subsystems.drive.Drive;

// import edu.wpi.first.math.geometry.Pose2d;

// public class DriveToStation extends DriveToPose {
//     public DriveToStation(Drive drive, FieldInfo field, JoystickInputs inputs, Supplier<Pose2d> robotPose, Station station) {
//         super(
//             drive,
//             field,
//             robotPose,
//             station.getTarget(field));
//     }

//     public DriveToStation(Drive drive, FieldInfo field, JoystickInputs inputs, Supplier<Pose2d> robotPose) {
//         this(
//             drive,
//             field,
//             inputs,
//             robotPose,
//             ProximityUtil.getClosestStation(drive, field));
//     }

//     /** Initializes the robot pose to the global pose of the robot. */
//     public DriveToStation(Drive drive, FieldInfo field, JoystickInputs inputs) {
//         this(
//             drive,
//             field,
//             inputs,
//             drive::getCurrentPose);
//     }
// }