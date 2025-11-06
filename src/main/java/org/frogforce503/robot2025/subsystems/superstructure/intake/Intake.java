package org.frogforce503.robot2025.subsystems.superstructure.intake;

import org.frogforce503.lib.subsystem.FFSubsystemBase;
import org.frogforce503.lib.util.LoggedTracer;
import org.frogforce503.robot2025.subsystems.superstructure.intake.pivot.IntakePivot;
import org.frogforce503.robot2025.subsystems.superstructure.intake.pivot.PivotIO;
import org.frogforce503.robot2025.subsystems.superstructure.intake.roller.IntakeRoller;
import org.frogforce503.robot2025.subsystems.superstructure.intake.roller.RollerIO;
import org.littletonrobotics.junction.Logger;

import lombok.Getter;
import lombok.Setter;

public class Intake extends FFSubsystemBase {
    private final IntakePivot pivot;
    private final IntakeRoller roller;

    @Setter @Getter private boolean hasAlgae = false;

    public Intake(PivotIO pivotIO, RollerIO rollerIO) {
        this.pivot = new IntakePivot(pivotIO);
        this.roller = new IntakeRoller(rollerIO);
    }

    @Override
    public void periodic() {
        super.periodic();

        pivot.periodic();
        roller.periodic();

        Logger.recordOutput("Claw/HasAlgae", hasAlgae);
        
        // Record cycle time
        LoggedTracer.record("Intake");
    }

    public double getPivotAngle() {
        return pivot.getAngle();
    }

    public double getRollerVolts() {
        return roller.getVolts();
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

    public boolean isPivotAtAngle(double setpointAngle, double tolerance) {
        return pivot.isPivotAtAngle(setpointAngle, tolerance);
    }

    public boolean isPivotAtAngle(double setpointAngle) {
        return pivot.isPivotAtAngle(setpointAngle);
    }

    public boolean isRollerAtVoltage(double setpointVolts, double tolerance) {
        return roller.isRollerAtVoltage(setpointVolts, tolerance);
    }

    public boolean isRollerAtVoltage(double setpointVolts) {
        return roller.isRollerAtVoltage(setpointVolts);
    }

    public void setGoal(IntakeGoal goal) {
        pivot.setGoal(goal);
        roller.setGoal(goal);
    }

    public boolean algaeCurrentThresholdForHoldMet() {
        return roller.algaeCurrentThresholdForHoldMet();
    }
}