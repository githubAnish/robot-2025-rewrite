package org.frogforce503.robot2025.auto.test;

import org.frogforce503.lib.auto.AutoMode;
import org.frogforce503.lib.auto.route.ChoreoRoute;
import org.frogforce503.robot2025.fields.FieldInfo;
import org.frogforce503.robot2025.subsystems.drive.Drive;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;

/** Dummy auto using Choreo. This auto is purely to show the format of how to create a auto using Choreo, so don't try to run this auto, as it will crash. */
public class ChoreoDummyAuto extends AutoMode {
    private final AutoRoutine routine;
    private final AutoTrajectory trajectory;

    public ChoreoDummyAuto(Drive drive, FieldInfo field, AutoFactory factory) {
        super(drive, field);

        this.routine = factory.newRoutine("Routine");

        this.trajectory = routine.trajectory("ChoreoTestPath"); // ChoreoTestPath.traj does not exist, so don't try to load this auto.
    }

    @Override
    public Command routine() {
        return routine.cmd();
    }

    @Override
    public ChoreoRoute getRoute() {
        return new ChoreoRoute(trajectory);
    }
}