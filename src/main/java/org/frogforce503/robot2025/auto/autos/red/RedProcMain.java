// package org.frogforce503.robot2025.auto.red;

// import org.frogforce503.lib.auto.AutoMode;

// import org.frogforce503.lib.lineup.goals.Branch;
// import org.frogforce503.lib.trajectory.SwervePathBuilder;
// import org.frogforce503.lib.trajectory.routeTypes.ChoreoRoute;
// import org.frogforce503.robot2025.RobotContainer;
// import org.frogforce503.robot2025.commands.AutoIntakeCommands;
// import org.frogforce503.robot2025.commands.AutoScoreCommands;
// import org.frogforce503.robot2025.fields.FieldInfo;
// import org.frogforce503.robot2025.planners.MovementPlanner.Presets;
// import org.frogforce503.robot2025.subsystems.drive.Drive;
// import org.frogforce503.robot2025.subsystems.superstructure.Superstructure;

// import choreo.auto.AutoFactory;
// import choreo.auto.AutoRoutine;
// import choreo.auto.AutoTrajectory;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;

// public class RedProcMain extends AutoMode {
//     private AutoRoutine routine;
//     private AutoTrajectory scoreE;
//     private AutoTrajectory E_TO_HP;
//     private AutoTrajectory scoBlue;
//     private AutoTrajectory D_TO_HP;
//     private AutoTrajectory scoreC;
//     private AutoTrajectory C_TO_HP;
//     private AutoTrajectory scoreB;

//     public RedProcMain(
//         Drive drive,
//         FieldInfo field,
//         Superstructure superstructure,
//         AutoFactory autoFactory,
//         AutoIntakeCommands autoIntakeCommands,
//         AutoScoreCommands autoScoreCommands
//     ) {
//         super(drive, field, superstructure);
        
//         routine = autoFactory.newRoutine("Routine");
//         scoreE = routine.trajectory( "Red-Proc-Score_E");
//         E_TO_HP = routine.trajectory( "Red-Proc-E_To_HP");
//         scoBlue = routine.trajectory( "Red-Proc-Score_D");
//         D_TO_HP = routine.trajectory( "Red-Proc-D_To_HP");
//         scoreC = routine.trajectory( "Red-Proc-Score_C");
//         C_TO_HP = routine.trajectory( "Red-Proc-C_To_HP");
//         scoreB = routine.trajectory( "Red-Proc-Score_B");



//         routine.active().onTrue(
//             Commands.sequence(

//                 //First Path
//                 scoreE.cmd().alongWith(Commands.sequence(
//                     RobotContainer.intake.idle_auton(),
//                     // Commands.waitUntil(RobotContainer.intake::atGoal),
//                     RobotContainer.elevator.holdCoral(),
//                     Commands.waitSeconds(0.1),
//                     RobotContainer.selectPreset(Presets.L4),
//                     RobotContainer.arm.setToPreset(), 
//                     RobotContainer.wrist.setToPreset()
//                 )),
//                 RobotContainer.lineupplanner.autoScore(Branch.LEFT).withTimeout(1.3)
//                 .alongWith(RobotContainer.elevator.setToPreset()),
//                 Commands.waitUntil(RobotContainer.elevator::atGoal),
//                 RobotContainer.ejectCoral(),

//                 //Second Path
//                 E_TO_HP.cmd().alongWith(
//                    Commands.waitSeconds(0.5).andThen(
//                     Commands.sequence(
//                         RobotContainer.intake.scoringClearance(),//.until(RobotContainer.intake::atGoal),
//                         RobotContainer.idle_auton(),
//                         RobotContainer.intake.idle_auton()  
//                     ).withTimeout(0.5)
//                     )
//                 ),
//                 Commands.waitUntil(RobotContainer.claw::lowerSensorGot).alongWith(RobotContainer.drive.brake()).deadlineFor(RobotContainer.coral_intake_auton()),

//                 //Third Path
//                 scoBlue.cmd().alongWith(Commands.sequence(
//                     RobotContainer.coral_intake_auton().until(RobotContainer.claw::hasCoral),
//                     RobotContainer.intake.idle_auton(),
//                     // Commands.waitUntil(RobotContainer.intake::atGoal),
//                     RobotContainer.elevator.holdCoral(),
//                     RobotContainer.selectPreset(Presets.L4),
//                     RobotContainer.arm.setToPreset(), 
//                     RobotContainer.wrist.setToPreset()
//                 )),
//                 RobotContainer.lineupplanner.autoScore(Branch.RIGHT).withTimeout(1.0)//1.3 -> 2.0
//                 .alongWith(RobotContainer.elevator.setToPreset()),
//                 Commands.waitUntil(RobotContainer.elevator::atGoal),
//                 RobotContainer.ejectCoral(),

//                 // Fourth Path
//                 D_TO_HP.cmd().alongWith(
//                     Commands.waitSeconds(0.5).andThen(
//                      Commands.sequence(
//                          RobotContainer.intake.scoringClearance(),//.until(RobotContainer.intake::atGoal),
//                          RobotContainer.idle_auton(),
//                          RobotContainer.intake.idle_auton()  
//                      ).withTimeout(0.5)
//                      )
//                  ),
//                  Commands.waitUntil(RobotContainer.claw::lowerSensorGot).alongWith(RobotContainer.drive.brake()).deadlineFor(RobotContainer.coral_intake_auton()),


//                 // Fifth Path
//                 scoreC.cmd().alongWith(Commands.sequence(
//                     RobotContainer.coral_intake_auton().until(RobotContainer.claw::hasCoral),
//                     RobotContainer.intake.idle_auton(),
//                     // Commands.waitUntil(RobotContainer.intake::atGoal),
//                     RobotContainer.elevator.holdCoral(),
//                     RobotContainer.selectPreset(Presets.L4),
//                     RobotContainer.arm.setToPreset(), 
//                     RobotContainer.wrist.setToPreset()
//                 )),
//                 RobotContainer.lineupplanner.autoScore(Branch.LEFT).withTimeout(1.0)
//                 .alongWith(RobotContainer.elevator.setToPreset()),
//                 Commands.waitUntil(RobotContainer.elevator::atGoal),
//                 RobotContainer.ejectCoral(),


//                 // Sixth Path
//                 C_TO_HP.cmd().alongWith(
//                     Commands.waitSeconds(0.5).andThen(
//                      Commands.sequence(
//                          RobotContainer.intake.scoringClearance(),//.until(RobotContainer.intake::atGoal),
//                          RobotContainer.idle_auton(),
//                          RobotContainer.intake.idle_auton()  
//                      ).withTimeout(0.5)
//                      )
//                  ),
                 
//                  Commands.waitUntil(RobotContainer.claw::lowerSensorGot).alongWith(RobotContainer.drive.brake()),

//                  // Seventh Path
//                  scoreB.cmd().alongWith(Commands.sequence(
//                     RobotContainer.coral_intake_auton().until(RobotContainer.claw::hasCoral),
//                     RobotContainer.intake.idle_auton(),
//                     // Commands.waitUntil(RobotContainer.intake::atGoal),
//                     RobotContainer.elevator.holdCoral(),
//                     RobotContainer.selectPreset(Presets.L4),
//                     RobotContainer.arm.setToPreset(), 
//                     RobotContainer.wrist.setToPreset()
//                 )),
//                 RobotContainer.lineupplanner.autoScore(Branch.RIGHT).withTimeout(1.3)
//                 .alongWith(RobotContainer.elevator.setToPreset()),
//                 Commands.waitUntil(RobotContainer.elevator::atGoal),
//                 RobotContainer.ejectCoral().alongWith(RobotContainer.drive.brake())
//             ));


            
//    }

//    @Override
//    public Command routine() {
//         return Commands.sequence(
//             routine.cmd()
//         );
//    }

//    @Override
//    public ChoreoRoute getRoute() {
//         return new ChoreoRoute(scoreE, E_TO_HP, scoBlue, D_TO_HP, scoreC, C_TO_HP, scoreB);
//    }
// }