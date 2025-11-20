package org.frogforce503.robot2025.subsystems.superstructure.intake.pivot;

import org.frogforce503.robot2025.Constants;
import org.frogforce503.robot2025.Robot;
import org.frogforce503.robot2025.subsystems.superstructure.intake.IntakeConstants;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import lombok.Getter;
import lombok.Setter;

import org.frogforce503.lib.math.Range;
import org.frogforce503.lib.util.LoggedTracer;

public class IntakePivot {
    private final PivotIO io;
    private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

    // Constants
    private final double horizontalAngle = Robot.bot.getIntakeConfig().pivotHorizontalAngle();
    private final Range range = Robot.bot.getIntakeConfig().pivotRange();
    @Setter private ArmFeedforward feedforward = Robot.bot.getIntakeConfig().pivotFF().getArmFF();
    @Setter private Constraints constraints = Robot.bot.getIntakeConfig().pivotConstraints();

    // Control
    private double targetAngle = IntakeConstants.START.pivotAngle();

    private boolean shouldRunProfile = false;
    private TrapezoidProfile profile;
    @Getter private State setpoint = new State();
    private boolean atGoal = false;

    public IntakePivot(PivotIO pivotIO) {
        this.io = pivotIO;

        profile =
            new TrapezoidProfile(
                Robot.bot.getIntakeConfig().pivotConstraints());
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("IntakePivot", inputs);

        // Update profile
        if (shouldRunProfile) {
            var goalState =
                new State(
                    range.clamp(targetAngle),
                    0.0);

            double previousVelocity = setpoint.velocity;

            setpoint =
                profile
                    .calculate(Constants.loopPeriodSecs, setpoint, goalState);

            if (!range.contains(setpoint.position)) {
                setpoint =
                    new State(
                        range.clamp(setpoint.position),
                        0.0);
            }

            atGoal = isAtAngle(goalState.position, IntakeConstants.kPivotTolerance);

            if (atGoal) {
                stop();
            } else {
                double accel = (setpoint.velocity - previousVelocity) / Constants.loopPeriodSecs;
                io.runPosition(setpoint.position, feedforward.calculate(Math.toRadians(setpoint.position - horizontalAngle), setpoint.velocity, accel));
            }

            // Log state
            Logger.recordOutput("IntakePivot/Profile/SetpointPosition", setpoint.position);
            Logger.recordOutput("IntakePivot/Profile/SetpointVelocity", setpoint.velocity);
            Logger.recordOutput("IntakePivot/Profile/GoalPosition", goalState.position);
            Logger.recordOutput("IntakePivot/AtGoal", atGoal);
        } else {
            // Reset setpoint
            setpoint = new State(getAngle(), 0.0);
      
            // Clear logs
            Logger.recordOutput("IntakePivot/Profile/SetpointPosition", 0.0);
            Logger.recordOutput("IntakePivot/Profile/SetpointVelocity", 0.0);
            Logger.recordOutput("IntakePivot/Profile/GoalPosition", 0.0);
            Logger.recordOutput("IntakePivot/AtGoal", true);
        }

        Logger.recordOutput("IntakePivot/CurrentPosition", getAngle());

        // Record cycle time
        LoggedTracer.record("IntakePivot");
    }

    public double getAngle() {
        return inputs.data.position();
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
        shouldRunProfile = false;
        io.runOpenLoop(output);
    }

    public void setAngle(double angle) {
        this.shouldRunProfile = true;
        this.targetAngle = angle;
    }

    public boolean isAtAngle(double setpointAngle, double tolerance) {
        return MathUtil.isNear(setpointAngle, getAngle(), tolerance);
    }
}