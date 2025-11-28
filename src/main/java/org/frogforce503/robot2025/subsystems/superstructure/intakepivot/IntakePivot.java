package org.frogforce503.robot2025.subsystems.superstructure.intakepivot;

import org.frogforce503.robot2025.Constants;
import org.frogforce503.robot2025.Robot;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.RobotState;
import lombok.Getter;
import lombok.Setter;

import org.frogforce503.lib.math.Range;
import org.frogforce503.lib.subsystem.FFSubsystemBase;
import org.frogforce503.lib.util.LoggedTracer;

public class IntakePivot extends FFSubsystemBase {
    private final IntakePivotIO io;
    private final IntakePivotIOInputsAutoLogged inputs = new IntakePivotIOInputsAutoLogged();

    // Constants
    private final Range motionRange = Robot.bot.getIntakePivotConfig().motionRange();
    @Setter private ArmFeedforward feedforward = Robot.bot.getIntakePivotConfig().kFF().getArmFF();

    // Control
    private double targetAngleRad = IntakePivotConstants.START;

    private boolean shouldRunProfile = false;
    @Setter private TrapezoidProfile profile;
    @Getter private State setpoint = new State();
    private boolean atGoal = false;

    public IntakePivot(IntakePivotIO io) {
        this.io = io;
        profile = new TrapezoidProfile(Robot.bot.getIntakePivotConfig().kConstraints());
    }

    @Override
    public void periodic() {
        super.periodic();

        io.updateInputs(inputs);
        Logger.processInputs("IntakePivot", inputs);

        // Update profile
        if (shouldRunProfile && RobotState.isEnabled()) {
            var goalState =
                new State(
                    motionRange.clamp(targetAngleRad),
                    0.0);

            double previousVelocity = setpoint.velocity;

            setpoint =
                profile
                    .calculate(Constants.loopPeriodSecs, setpoint, goalState);

            if (!motionRange.contains(setpoint.position)) {
                setpoint =
                    new State(
                        motionRange.clamp(setpoint.position),
                        0.0);
            }

            atGoal = isAtAngle(goalState.position, IntakePivotConstants.kPivotTolerance);

            if (atGoal) {
                stop();
            } else {
                double accel = (setpoint.velocity - previousVelocity) / Constants.loopPeriodSecs;
                io.runPosition(setpoint.position, feedforward.calculate(setpoint.position, setpoint.velocity, accel));
            }

            // Log state
            Logger.recordOutput("IntakePivot/Profile/SetpointPositionRad", setpoint.position);
            Logger.recordOutput("IntakePivot/Profile/SetpointVelocityRadPerSec", setpoint.velocity);
            Logger.recordOutput("IntakePivot/Profile/GoalPositionRad", goalState.position);
            Logger.recordOutput("IntakePivot/AtGoal", atGoal);
        } else {
            // Reset setpoint
            setpoint = new State(getAngleRad(), 0.0);
      
            // Clear logs
            Logger.recordOutput("IntakePivot/Profile/SetpointPositionRad", 0.0);
            Logger.recordOutput("IntakePivot/Profile/SetpointVelocityRadPerSec", 0.0);
            Logger.recordOutput("IntakePivot/Profile/GoalPositionRad", 0.0);
            Logger.recordOutput("IntakePivot/AtGoal", true);
        }

        Logger.recordOutput("IntakePivot/CurrentPositionRad", getAngleRad());

        // Record cycle time
        LoggedTracer.record("IntakePivot");
    }

    public double getAngleRad() {
        return inputs.data.positionRad();
    }

    // Actions
    public void setPID(double kP, double kI, double kD) {
        io.setPID(kP, kI, kD);
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        io.setBrakeMode(enabled);
    }

    @Override
    public void stop() {
        io.stop();
    }

    public void runOpenLoop(double output) {
        this.shouldRunProfile = false;
        io.runOpenLoop(output);
    }

    public void setAngle(double angleRad) {
        this.shouldRunProfile = true;
        this.targetAngleRad = angleRad;
    }

    public boolean isAtAngle(double angleRad, double tolerance) {
        return MathUtil.isNear(angleRad, getAngleRad(), tolerance);
    }
}