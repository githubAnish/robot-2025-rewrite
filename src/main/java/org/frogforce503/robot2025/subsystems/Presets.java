package org.frogforce503.robot2025.subsystems;

public enum Presets {
    // Default
    NONE(),

    // Coral
    L1(8, 38, 95),//
    L2(8, 73, 80),
    L3(7, 119, 82),
    L4(33, 145, 57.3),

    // Algae
    ALGAE_INTAKE_REEF_LOW(10, 85, 76, -4000, 0, 0),
    ALGAE_INTAKE_REEF_HIGH(33, 85, 76, -4000, 0, 0),

    PROCESSOR(10, 3, 0, 0, 180, -3000),
    NET(33, 205, 230),
    AUTON_NET(33, 205, 220);

    public double elevatorPos, armAng, wristAng, clawRollerSpeed, intakePivotPos, intakeRollerSpeed;

    private Presets(double elevatorPos, double armAng, double wristAng, double clawRollerSpeed, double intakePivotPos, double intakeRollerSpeed) {
        this.elevatorPos = elevatorPos;
        this.armAng = armAng;
        this.wristAng = wristAng;
        this.clawRollerSpeed = clawRollerSpeed;
        this.intakePivotPos = intakePivotPos;
        this.intakeRollerSpeed = intakeRollerSpeed;
    }

    private Presets(double elevatorPos, double armAng, double wristAng) {
        this(elevatorPos, armAng, wristAng, 0, 0, 0);
    }

    private Presets() {
        this(0, 0, 0, 0, 0, 0);
    }
}

// public Command coral_intake() {
//     return Commands.sequence(
//         intake.lowClearance(),
//         Commands.parallel(
//             arm.coral_intake(),
//             claw.intakeCoral(),
//             elevator.off(),
//             wrist.idle()
//         ),
//         Commands.waitUntil(this::lowerSensorGot),
//         Commands.either(
//             claw.intakeCoral().alongWith(Commands.runOnce(() -> hasCoral = false)),
//             claw.off().alongWith(Commands.runOnce(() -> hasCoral = true)),
//             this::lowerSensorGot
//         ).repeatedly().withName("check intake")
//         // Commands.waitUntil(Logic.not(claw::lowerSensorGot)),
//         // claw.off(),
//         // Commands.runOnce(() -> claw.hasCoral = true)
//     )
//     .unless(() -> hasCoral);
// }

// public Command coral_intake_auton() {
//     return Commands.sequence(
//         intake.idle().until(intake::atGoal),
//         Commands.parallel(
//             arm.idle().until(arm::atGoal),
//             claw.intakeCoral().until(claw::atGoal),
//             elevator.idle().until(elevator::atGoal),
//             wrist.idle().until(wrist::atGoal)
//         ),
//         Commands.waitUntil(claw::lowerSensorGot),
//         Commands.waitUntil(Logic.not(claw::lowerSensorGot)),
//         claw.off()
//     );
// }

// public Command holdCoral() {
//     return intake.idle().until(intake::atGoal);
// }

// public Command preScore() {
//     return intake.scoringClearance()
//     .andThen(Commands.waitUntil(intake::atGoal))
//     .andThen(
//         Commands.parallel(
//             arm.setToPreset(),
//             Commands.either(
//                 wrist.setToPreset(),
//                 wrist.toZero(),
//                 () -> PresetPlanner.isCurrentPresetEqualsTo(Presets.L1)
//             ),
//             elevator.setToPreset().onlyIf(() -> PresetPlanner.isCurrentPresetEqualsTo(Presets.L1))
//         )
//     )
//     .andThen(Commands.waitUntil(arm::atGoal))
//     .andThen(intake.idle().until(intake::atGoal));
// }

// // public Command score() {
// //     return Commands.sequence(
// //         arm.setToPreset().until(arm::atGoal).alongWith(wrist.setToPreset().until(wrist::atGoal)),
// //         elevator.setToPreset().until(elevator::atGoal)
// //     );
// // }

// public Command ejectCoral() {
//     return claw.ejectCoral().until(() -> !claw.hasCoral());
// }

// public Command releaseCoral() {
//     return Commands.sequence(
//         ejectCoral().withTimeout(0.5),
//         Commands.waitSeconds(0.25),
//         intake.scoringClearance(),
//         Commands.waitUntil(intake::atGoal),
//         idle(),
//         Commands.waitUntil(arm::atGoal),
//         intake.idle()
//     );
// }

// public Command algae_intake_ground() {
//     return intake.intake().until(intake::hasAlgae).onlyIf(() -> !intake.hasAlgae());
// }

// public Command algae_intake_ground_release() {
//     return Commands.sequence(
//         intake.hold().until(intake::atGoal)
//     );
// }

// public Command algae_intake_reef() {
//     return Commands.sequence(
//         intake.scoringClearance(),
//         Commands.waitUntil(intake::atGoal),
//         elevator.setToPreset().until(elevator::atGoal),
//         Commands.parallel(
//             arm.setToPreset().until(arm::atGoal),
//             wrist.setToPreset().until(wrist::atGoal),
//             claw.intakeAlgae().until(claw::atGoal)
//         ),
//         Commands.waitUntil(arm::atGoal),
//         intake.idle()
//     );
// }

// public Command algae_hold() {
//     return intake.hold_claw().until(intake::atGoal)
//     .andThen(elevator.holdAlgae().until(elevator::atGoal))
//     .andThen(Commands.waitUntil(elevator::atGoal))
//     .andThen(
//         Commands.parallel(
//             arm.holdAlgae().until(arm::atGoal),
//             wrist.holdAlgae().until(wrist::atGoal),
//             claw.holdAlgae().until(claw::atGoal)
//         )
//     );
// }

// public Command algae_intake_into_claw() {
//     return Commands.sequence(
//         elevator.handoff().until(elevator::atGoal),
//         Commands.parallel(
//             arm.handoff().until(arm::atGoal),
//             wrist.handoff().until(wrist::atGoal),
//             claw.intakeAlgae().until(claw::atGoal)
//         ),
//         intake.intake()
//     );
// }

// public Command algae_intake_into_claw_release() {
//     return intake.handoff()
//     .andThen(Commands.waitUntil(intake::pivotAtGoal))
//     .andThen(
//         Commands.parallel(
//             arm.handoff_release().until(arm::atGoal),
//             wrist.handoff_release().until(wrist::atGoal)
//         )
//     )
//     // .andThen(Commands.waitUntil(Logic.and(wrist::atGoal)))
//     .andThen(elevator.handoff_release().until(elevator::atGoal))
//     .andThen(Commands.waitUntil(elevator::atGoal))
//     .andThen(Commands.waitSeconds(0.2))
//     .andThen(intake.handoff_release())
//     .andThen(Commands.waitUntil(claw::hasAlgae))
//     .andThen(Commands.waitSeconds(0.2))
//     .andThen(algae_hold());
// }

// public Command scoreProc() {
//     return Commands.sequence(
//         intake.setToPreset().until(intake::atGoal)
//     );
// }

// public Command ejectAlgae() {
//     return claw.ejectAlgae().until(() -> !claw.hasAlgae());
// }

// public Command releaseAlgae() {
//     return Commands.sequence(
//         // Commands.either(
//         //     Commands.waitUntil(Logic.and(elevator::atGoal, () -> arm.getPosition() > 175))
//         //     // .andThen(Commands.waitSeconds(0.2))
//         //     .andThen(wrist.net_release())
//         //     .andThen(Commands.waitUntil(() -> wrist.getPosition() > 155)),
//         //     Commands.none(),
//         //     () -> PresetPlanner.isCurrentPresetEqualsTo(Presets.NET)
//         // ),
//         wrist.to90().onlyIf(() -> PresetPlanner.isCurrentPresetEqualsTo(Presets.PROCESSOR)),
//         ejectAlgae().withTimeout(0.5),
//         Commands.waitSeconds(0.25),
//         Commands.either(
//             modeSwitch(),
//             Commands.sequence(
//                 intake.scoringClearance().until(intake::atGoal),
//                 idle(),
//                 Commands.waitUntil(arm::atGoal),
//                 intake.idle().until(intake::atGoal)
//             ),
//             () -> PresetPlanner.isCurrentPresetEqualsTo(Presets.ALGAE_INTAKE_REEF_HIGH, Presets.ALGAE_INTAKE_REEF_LOW)
//         )
//     );
// }

// public Command releaseAlgaeAuton() {
//     return Commands.sequence(
//         // Commands.either(
//         //     Commands.waitUntil(Logic.and(elevator::atGoal, () -> arm.getPosition() > 175))
//         //     // .andThen(Commands.waitSeconds(0.2))
//         //     .andThen(wrist.net_release())
//         //     .andThen(Commands.waitUntil(() -> wrist.getPosition() > 155)),
//         //     Commands.none(),
//         //     () -> PresetP+lanner.isCurrentPresetEqualsTo(Presets.NET)
//         // ),
//         wrist.to90().onlyIf(() -> PresetPlanner.isCurrentPresetEqualsTo(Presets.PROCESSOR)),
//         ejectAlgae().withTimeout(0.5),
//         Commands.waitSeconds(0.25),
//         Commands.either(
//             modeSwitch(),
//             Commands.sequence(
//                 intake.scoringClearance().until(intake::atGoal),
//                 idle(),
//                 Commands.waitUntil(() -> arm.getPosition() < 24),
//                 intake.idle().until(intake::atGoal)
//             ),
//             () -> PresetPlanner.isCurrentPresetEqualsTo(Presets.ALGAE_INTAKE_REEF_HIGH, Presets.ALGAE_INTAKE_REEF_LOW)
//         )
//     );
// }

// public Command climberOff() {
//     return intake.off().alongWith(climber.off());
// }

// public Command autoScore(Branch branch) {
//     return Commands.parallel(
//         score(),
//         DriveCommands.driveTo(branch).unless(() -> LoggerUtil.getBool("AdvantageKit/RealOutputs/AGILE/DISABLE LINEUP").get())
//     );
// }

// public Command autoPluck(Branch branch) {
//     return Commands.sequence(
//         Commands.parallel(
//             algae_intake_reef(),
//             DriveCommands.driveToAlgae()
//         ),
//         Commands.waitUntil(claw::hasAlgae),
//         DriveCommands.algaeBackup(),
//         algae_hold()
//     );
// }

// public Command modeSwitch() {
//     return Commands.sequence(
//         elevator.switch_mode(),
//         Commands.waitUntil(elevator::atGoal),
//         intake.scoringClearance(),
//         Commands.waitUntil(intake::atGoal),
//         Commands.parallel(
//             arm.idle().until(arm::atGoal),
//             claw.idle().until(claw::atGoal),
//             wrist.idle().until(wrist::atGoal)
//         ),
//         Commands.waitUntil(arm::atGoal),
//         elevator.idle().until(elevator::atGoal),
//         intake.idle()
//     );
// }

// /** AUTONS ONLY */
// public Command score(Presets preset) {
//     return Commands.sequence(
//         selectPreset(preset),
//         Commands.parallel(
//             arm.setToPreset(),
//             elevator.setToPreset(),
//             wrist.setToPreset()
//         )
//     );
// }

// // Base Commands
// public Command selectPreset(Presets preset) {
//     return Commands.runOnce(() -> PresetPlanner.setPreset(preset));
// }

// public Command idle() {
//     return Commands.sequence(
//         elevator.idle(),
//         // Commands.waitUntil(elevator::atGoal),
//         arm.down().alongWith(claw.idle(), wrist.idle())
//     );
//     // .andThen(Commands.runOnce(() -> wrist.seedPosition()));
// }

// public Command idle_auton() {
//     return Commands.parallel(
//         arm.down().until(arm::atGoal),
//         claw.idle().until(claw::atGoal),
//         elevator.idle().until(elevator::atGoal),
//         wrist.idle().until(wrist::atGoal)
//     );
//     // .andThen(Commands.runOnce(() -> wrist.seedPosition()));
// }