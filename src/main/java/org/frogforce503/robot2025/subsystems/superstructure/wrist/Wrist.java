package org.frogforce503.robot2025.subsystems.superstructure.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;

import org.frogforce503.lib.math.Range;
import org.frogforce503.lib.subsystem.FFSubsystemBase;
import org.frogforce503.lib.util.LoggedTracer;
import org.frogforce503.robot2025.Robot;
import org.littletonrobotics.junction.Logger;

public class Wrist extends FFSubsystemBase {
    private final WristIO io;
    private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

    // Constants
    private final double horizontalAngle = Robot.bot.getWristConfig().horizontalAngle();
    private final Range range = Robot.bot.getWristConfig().range();
    private ArmFeedforward feedforward = Robot.bot.getWristConfig().kFF().getArmFF();

    // Control
    private double targetAngle = WristConstants.START;
    
    private boolean shouldRunPosition = false;
    private boolean atGoal = false;

    public Wrist(WristIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        super.periodic();

        io.updateInputs(inputs);
        Logger.processInputs("Wrist", inputs);

        // Run position mode unless requested to stop
        if (shouldRunPosition) {
            var goalState = range.clamp(targetAngle);

            atGoal = isAtAngle(goalState, WristConstants.kTolerance);

            if (atGoal) {
                stop();
            } else {
                io.runPosition(goalState, feedforward.calculate(Math.toRadians(goalState - horizontalAngle), 0.0));
            }

            // Log state
            Logger.recordOutput("Wrist/GoalPosition", goalState);
            Logger.recordOutput("Wrist/AtGoal", atGoal);
        } else {
            // Reset setpoint
            targetAngle = 0.0;
      
            // Clear logs
            Logger.recordOutput("Wrist/GoalPosition", 0.0);
            Logger.recordOutput("Wrist/AtGoal", true);
        }

        Logger.recordOutput("Wrist/CurrentPosition", getRelativeAngle());

        // Record cycle time
        LoggedTracer.record("Wrist");
    }

    public double getRelativeAngle() {
        return inputs.data.relativePosition();
    }

    public double getAbsoluteAngle() {
        return inputs.data.absolutePosition();
    }

    public void setEncoderPosition(double position) {
        io.setEncoderPosition(position);
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
        this.shouldRunPosition = false;
        io.runOpenLoop(output);
    }

    public void setAngle(double angle) {
        this.shouldRunPosition = true;
        this.targetAngle = angle;
    }

    public boolean isAtAngle(double setpointAngle, double tolerance) {
        return MathUtil.isNear(setpointAngle, getRelativeAngle(), tolerance);
    }
}