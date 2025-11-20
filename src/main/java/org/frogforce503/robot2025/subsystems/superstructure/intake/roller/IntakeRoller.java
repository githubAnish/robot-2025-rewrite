package org.frogforce503.robot2025.subsystems.superstructure.intake.roller;

import org.frogforce503.robot2025.Robot;
import org.frogforce503.robot2025.subsystems.superstructure.intake.IntakeConstants;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.RobotBase;
import lombok.Setter;

import org.frogforce503.lib.util.LoggedTracer;

public class IntakeRoller {
    private final RollerIO io;
    private final RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();

    // Constants
    @Setter private SimpleMotorFeedforward feedforward = Robot.bot.getIntakeConfig().rollerFF().getSimpleMotorFF();
    private final Debouncer algaeIntakeDebouncer = new Debouncer(0.5, DebounceType.kRising);

    // Control
    private double targetVelocity = IntakeConstants.START.rollerVelocity();

    private boolean shouldRunVelocity = false;
    private boolean atGoal = false;

    public IntakeRoller(RollerIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("IntakeRoller", inputs);

        // Update profile
        if (shouldRunVelocity) {
            atGoal = isAtVelocity(targetVelocity, IntakeConstants.kRollerTolerance);
            io.runVelocity(targetVelocity, feedforward.calculate(targetVelocity));

            // Log state
            Logger.recordOutput("IntakeRoller/SetpointVelocity", targetVelocity);
            Logger.recordOutput("IntakeRoller/AtGoal", atGoal);
        } else {
            // Reset setpoint
            targetVelocity = 0.0;
      
            // Clear logs
            Logger.recordOutput("IntakeRoller/AtGoal", true);
        }

        Logger.recordOutput("IntakeRoller/CurrentVelocity", getVelocity());

        // Record cycle time
        LoggedTracer.record("IntakeRoller");
    }

    public double getVelocity() {
        return inputs.data.velocity();
    }

    public void setPID(double kP, double kI, double kD) {
        io.setPID(kP, kI, kD);
    }

    public void setBrakeMode(boolean enabled) {
        io.setBrakeMode(enabled);
    }

    public void stop() {
        io.stop();
    }

    public void runOpenLoop(double output) {
        shouldRunVelocity = false;
        io.runOpenLoop(output);
    }

    public void setVelocity(double velocity) {
        shouldRunVelocity = true;
        this.targetVelocity = velocity;
    }

    public boolean isAtVelocity(double velocity, double tolerance) {
        return MathUtil.isNear(velocity, getVelocity(), tolerance);
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