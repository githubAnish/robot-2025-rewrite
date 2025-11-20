// package org.frogforce503.robot2025.commands.drive;

// import java.util.function.Supplier;

// import org.frogforce503.lib.math.GeomUtil;
// import org.frogforce503.robot2025.FieldInfo;
// import org.frogforce503.lib.reefscape.ReefSide;
// import org.frogforce503.lib.util.ProximityUtil;
// import org.frogforce503.robot2025.subsystems.drive.Drive;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.util.Units;

// public class DriveToReefAlgae extends DriveToPose {
//     public DriveToReefAlgae(Drive drive, FieldInfo field, Supplier<Pose2d> robotPose, ReefSide side) {
//         super(
//             drive,
//             field,
//             robotPose,
//             () ->
//                 side
//                     .getTarget(field)
//                     .get()
//                     .plus(
//                         GeomUtil.toTransform2d(Units.inchesToMeters(4), 0.0)));
//     }

//     public DriveToReefAlgae(Drive drive, FieldInfo field, Supplier<Pose2d> robotPose) {
//         this(
//             drive,
//             field,
//             robotPose,
//             ProximityUtil.getClosestReefSide(drive, field));
//     }

//     /** Initializes the robot pose to the global pose of the robot. */
//     public DriveToReefAlgae(Drive drive, FieldInfo field) {
//         this(
//             drive,
//             field,
//             drive::getCurrentPose);
//     }
// }