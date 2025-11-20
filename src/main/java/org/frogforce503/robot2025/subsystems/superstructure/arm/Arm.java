package org.frogforce503.robot2025.subsystems.superstructure.arm;

import org.frogforce503.robot2025.Constants;
import org.frogforce503.robot2025.Robot;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import lombok.Getter;
import lombok.Setter;

import org.frogforce503.lib.math.Range;
import org.frogforce503.lib.subsystem.FFSubsystemBase;
import org.frogforce503.lib.util.LoggedTracer;

public class Arm extends FFSubsystemBase {
    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    // Constants
    private final double horizontalAngle = Robot.bot.getArmConfig().horizontalAngle();
    private final Range range = Robot.bot.getArmConfig().range();
    @Setter private ArmFeedforward feedforward = Robot.bot.getArmConfig().kFF().getArmFF();
    @Setter private Constraints constraints = Robot.bot.getArmConfig().kConstraints();

    // Control
    private double targetAngle = ArmConstants.START;

    private boolean shouldRunProfile = false;
    private TrapezoidProfile profile;
    @Getter private State setpoint = new State();
    private boolean atGoal = false;

    public Arm(ArmIO io) {
        this.io = io;
        profile = new TrapezoidProfile(constraints);
    }

    @Override
    public void periodic() {
        super.periodic();

        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);

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

            atGoal = isAtAngle(goalState.position, ArmConstants.kTolerance);

            if (atGoal) {
                stop();
            } else {
                double accel = (setpoint.velocity - previousVelocity) / Constants.loopPeriodSecs;
                io.runPosition(setpoint.position, feedforward.calculate(Math.toRadians(setpoint.position - horizontalAngle), setpoint.velocity, accel));
            }

            // Log state
            Logger.recordOutput("Arm/Profile/SetpointPosition", setpoint.position);
            Logger.recordOutput("Arm/Profile/SetpointVelocity", setpoint.velocity);
            Logger.recordOutput("Arm/Profile/GoalPosition", goalState.position);
            Logger.recordOutput("Arm/AtGoal", atGoal);
        } else {
            // Reset setpoint
            setpoint = new State(getAngle(), 0.0);
      
            // Clear logs
            Logger.recordOutput("Arm/Profile/SetpointPosition", 0.0);
            Logger.recordOutput("Arm/Profile/SetpointVelocity", 0.0);
            Logger.recordOutput("Arm/Profile/GoalPosition", 0.0);
            Logger.recordOutput("Arm/AtGoal", true);
        }

        Logger.recordOutput("Arm/CurrentPosition", getAngle());

        // Record cycle time
        LoggedTracer.record("Arm");
    }

    public double getAngle() {
        return inputs.data.position();
    }

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

    public void setAngle(double angle) {
        this.shouldRunProfile = true;
        this.targetAngle = angle;
    }

    public boolean isAtAngle(double angle, double tolerance) {
        return MathUtil.isNear(angle, getAngle(), tolerance);
    }
}