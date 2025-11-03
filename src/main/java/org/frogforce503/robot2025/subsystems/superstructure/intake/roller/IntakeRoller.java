package org.frogforce503.robot2025.subsystems.superstructure.intake.roller;

import org.frogforce503.robot2025.subsystems.superstructure.intake.IntakeGoal;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.RobotBase;

import org.frogforce503.lib.subsystem.FFSubsystemBase;
import org.frogforce503.lib.util.LoggedTracer;

public class IntakeRoller extends FFSubsystemBase {
    private final RollerIO io;
    private final RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();

    // Constants
    private final double tolerance = 0.5;
    private final Debouncer algaeIntakeDebouncer = new Debouncer(0.5, DebounceType.kRising);

    // Control
    private double targetRollerVolts = IntakeGoal.START.getRollerVolts();

    private boolean shouldRunVolts = false;
    private boolean atGoal = false;

    public IntakeRoller(RollerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        super.periodic();
        
        io.updateInputs(inputs);
        Logger.processInputs("IntakeRoller", inputs);

        // Update profile
        if (shouldRunVolts) {
            atGoal = isRollerAtVoltage(targetRollerVolts);
            io.runVolts(targetRollerVolts);

            // Log state
            Logger.recordOutput("IntakeRoller/AtGoal", atGoal);
        } else {
            // Reset setpoint
            targetRollerVolts = 0.0;
      
            // Clear logs
            Logger.recordOutput("IntakeRoller/AtGoal", true);
        }

        Logger.recordOutput("IntakeRoller/CurrentVolts", getVolts());

        // Record cycle time
        LoggedTracer.record("IntakeRoller");
    }

    public double getVolts() {
        return inputs.data.appliedVolts();
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        io.setBrakeMode(enabled);
    }

    @Override
    public void stop() {
        io.stop();
    }

    public void runOpenLoop(double pivotOutput, double rollerOutput) {
        shouldRunVolts = false;
        io.runOpenLoop(rollerOutput);
    }

    public boolean isRollerAtVoltage(double setpointVolts, double tolerance) {
        return MathUtil.isNear(setpointVolts, getVolts(), tolerance);
    }

    public boolean isRollerAtVoltage(double setpointVolts) {
        return isRollerAtVoltage(setpointVolts, tolerance);
    }

    public void setGoal(IntakeGoal goal) {
        shouldRunVolts = true;
        this.targetRollerVolts = goal.getRollerVolts();
    }

    public boolean algaeCurrentThresholdForHoldMet() {
        if (RobotBase.isSimulation()) {
            return true;
        }

        return
            algaeIntakeDebouncer.calculate(
                inputs.data.statorCurrentAmps() > 15);
    }
}