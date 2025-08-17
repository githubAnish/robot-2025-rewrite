package org.frogforce503.robot2025.auto.blue;

import org.frogforce503.lib.auto.AutoMode;
import org.frogforce503.lib.auto.route.ChoreoRoute;
import org.frogforce503.robot2025.commands.AutoIntakeCommands;
import org.frogforce503.robot2025.commands.AutoScoreCommands;
import org.frogforce503.robot2025.commands.coral_score_reef.Branch;
import org.frogforce503.robot2025.fields.FieldInfo;
import org.frogforce503.robot2025.subsystems.drive.Drive;
import org.frogforce503.robot2025.subsystems.superstructure.Superstructure;
import org.frogforce503.robot2025.subsystems.superstructure.Superstructure.Mode;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class BlueBabyAuton extends AutoMode {
    private AutoRoutine routine;
    private AutoTrajectory scoreH;
    private AutoTrajectory backUp;
    
    public BlueBabyAuton(
        Drive drive,
        FieldInfo field,
        Superstructure superstructure,
        AutoFactory autoFactory,
        AutoIntakeCommands autoIntakeCommands,
        AutoScoreCommands autoScoreCommands
    ) {
        super(drive, field, superstructure);

        routine = autoFactory.newRoutine("Routine");
        scoreH = routine.trajectory("Blue-Baby-Score_H");
        backUp = routine.trajectory("Blue-Baby-Back_Up");

        routine
            .active()
            .onTrue(
                Commands.sequence(
                    scoreH.cmd(),
                    autoScoreCommands
                        .coralAutoScore(() -> Branch.RIGHT, () -> Mode.L4)
                        .withTimeout(4.5),
                    superstructure.ejectCoral(),
                    Commands.waitSeconds(0.5),
                    backUp
                        .cmd()
                        .alongWith(
                            superstructure
                                .home()
                                .withTimeout(0.5)),
                    Commands.runOnce(drive::stop),
                    setSuperstructureMode(() -> Mode.CORAL_INTAKE)
            ));
    }
    
    @Override
    public Command routine() {
        return routine.cmd();
    }

    @Override
    public ChoreoRoute getRoute() {
        return new ChoreoRoute(scoreH, backUp);
    }
}