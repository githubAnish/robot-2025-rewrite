// package org.frogforce503.robot2025.commands.algae_score_barge;

// import java.util.function.Supplier;

// import org.frogforce503.lib.lineup.goals.AlgaeLoc;
// import org.frogforce503.lib.util.Util;
// import org.frogforce503.robot2025.RobotContainer;
// import org.frogforce503.robot2025.subsystems.drive.Drive;
// import org.littletonrobotics.junction.Logger;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj2.command.Command;

// public class AlignToNet extends Command {

//     private final Supplier<Pose2d> robotPose;
//     private final Supplier<Pose2d> target;

//     private ProfiledPIDController xController =
//         new ProfiledPIDController(
//             3.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.7, Drive.FAST_TRANSLATION_METERS_PER_SECOND * 0.7));

//     private PIDController thetaController =
//         new PIDController(4.0, 0.0, 0.0);

//     private double xErrorAbs = 0.0;
//     private double thetaErrorAbs = 0.0;
//     private boolean running = false;

//     public AlignToNet(Supplier<Pose2d> robotPose) {
//         this.robotPose = robotPose;
//         this.target = AlgaeLoc.NET.target;

//         // Set tolerances
//         xController.setTolerance(0.01);
//         thetaController.setTolerance(Math.toRadians(1));

//         // Enable continuous input for theta controller
//         thetaController.enableContinuousInput(-Math.PI, Math.PI);

//         addRequirements(RobotContainer.drive);
//     }

//     @Override
//     public void initialize() {}

//     @Override
//     public void execute() {
//         running = true;

//         // Get current pose and target pose
//         Pose2d currentPose = robotPose.get();
//         Pose2d targetPose = target.get();

//         // Calculate errors
//         Translation2d delta = currentPose.getTranslation().minus(targetPose.getTranslation());

//         xErrorAbs = delta.getX();
//         thetaErrorAbs = Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());

//         // Calculate speeds
//         double speed = xController.calculate(xErrorAbs, 0);

//         Translation2d movement = new Translation2d(speed, RobotContainer.drive.getTeleopInput().getFirst().getY());

//         double omega = thetaController.calculate(thetaErrorAbs, 0);

//         // Apply Speeds
//         RobotContainer.drive.acceptSwerveCommand(
//             new SwerveCommand.FieldCentric()
//                 .withVelocityX(movement.getX())
//                 .withVelocityY(movement.getY())
//                 .withRotationalRate(omega)
//         );

//         Logger.recordOutput("AlignToNet/Is Finished", isFinished());
//         Logger.recordOutput("AlignToNet/X Error", delta.getX());
//         Logger.recordOutput("AlignToNet/Y Error", delta.getY());
//         Logger.recordOutput("AlignToNet/Theta Error", thetaErrorAbs);
//         Logger.recordOutput("AlignToNet/Commanded Pose", targetPose);
//     }

//     @Override
//     /** Don't finish the command so aligning & controlling movement in the Y direction can occur simultaneously */
//     public boolean isFinished() {
//         return false;
//     }

//     @Override
//     public void end(boolean interrupted) {
//         RobotContainer.drive.stop();
//         running = false;
//     }
    
//     /** Checks if the robot is stopped at the final pose. */
//     public boolean atGoal() {
//         return running && xController.atGoal() && thetaController.atSetpoint();
//     }

//     /** Checks if the robot pose is within the allowed drive and theta tolerances. */
//     public boolean withinTolerance(double xTolerance, Rotation2d thetaTolerance) {
//         return running
//         && Math.abs(xErrorAbs) < xTolerance
//         && Math.abs(thetaErrorAbs) < thetaTolerance.getRadians();
//     }
// }