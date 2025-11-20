package org.frogforce503.robot2025.subsystems.superstructure.intake;

import org.frogforce503.lib.subsystem.FFSubsystemBase;
import org.frogforce503.lib.util.LoggedTracer;
import org.frogforce503.robot2025.subsystems.superstructure.intake.IntakeConstants.IntakeGoal;
import org.frogforce503.robot2025.subsystems.superstructure.intake.pivot.IntakePivot;
import org.frogforce503.robot2025.subsystems.superstructure.intake.pivot.PivotIO;
import org.frogforce503.robot2025.subsystems.superstructure.intake.roller.IntakeRoller;
import org.frogforce503.robot2025.subsystems.superstructure.intake.roller.RollerIO;

public class Intake extends FFSubsystemBase {
    private final IntakePivot pivot;
    private final IntakeRoller roller;

    public Intake(PivotIO pivotIO, RollerIO rollerIO) {
        this.pivot = new IntakePivot(pivotIO);
        this.roller = new IntakeRoller(rollerIO);
    }

    @Override
    public void periodic() {
        super.periodic();

        pivot.periodic();
        roller.periodic();
        
        // Record cycle time
        LoggedTracer.record("Intake");
    }

    public double getPivotAngle() {
        return pivot.getAngle();
    }

    public double getRollerVelocity() {
        return roller.getVelocity();
    }

    @Override
	protected void setBrakeMode(boolean enabled) {
		pivot.setBrakeMode(enabled);
        roller.setBrakeMode(enabled);
	}

	@Override
	public void stop() {
		pivot.stop();
        roller.stop();
	}

    public void runOpenLoop(double pivotOutput, double rollerOutput) {
        pivot.runOpenLoop(pivotOutput);
        roller.runOpenLoop(rollerOutput);
    }

    public void setGoal(IntakeGoal goal) {
        pivot.setAngle(goal.pivotAngle());
        roller.setVelocity(goal.rollerVelocity());
    }

    public boolean isPivotAtAngle(double setpointAngle, double tolerance) {
        return pivot.isAtAngle(setpointAngle, tolerance);
    }

    public boolean isRollerAtVoltage(double setpointVolts, double tolerance) {
        return roller.isAtVelocity(setpointVolts, tolerance);
    }

    public boolean algaeCurrentThresholdForHoldMet() {
        return roller.algaeCurrentThresholdForHoldMet();
    }
}