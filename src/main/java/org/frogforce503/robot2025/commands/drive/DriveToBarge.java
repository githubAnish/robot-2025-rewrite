// package org.frogforce503.robot2025.commands.drive;

// import java.util.function.Supplier;

// import org.frogforce503.lib.io.JoystickInputs;
// import org.frogforce503.lib.reefscape.Barge;
// import org.frogforce503.lib.util.ProximityUtil;
// import org.frogforce503.robot2025.FieldInfo;
// import org.frogforce503.robot2025.subsystems.drive.Drive;

// import edu.wpi.first.math.geometry.Pose2d;

// public class DriveToBarge extends DriveToPose {
//     public DriveToBarge(Drive drive, FieldInfo field, JoystickInputs inputs, Supplier<Pose2d> robotPose, Barge barge) {
//         super(
//             drive,
//             field,
//             robotPose,
//             barge.getTarget(drive, field),
//             inputs);
//     }

//     public DriveToBarge(Drive drive, FieldInfo field, JoystickInputs inputs, Supplier<Pose2d> robotPose) {
//         this(
//             drive,
//             field,
//             inputs,
//             robotPose,
//             ProximityUtil.getClosestBarge(drive, field));
//     }

//     /** Initializes the robot pose to the global pose of the robot. */
//     public DriveToBarge(Drive drive, FieldInfo field, JoystickInputs inputs) {
//         this(
//             drive,
//             field,
//             inputs,
//             drive::getCurrentPose);
//     }
// }