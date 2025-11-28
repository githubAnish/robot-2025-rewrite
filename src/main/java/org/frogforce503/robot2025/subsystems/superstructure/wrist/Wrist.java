package org.frogforce503.robot2025.subsystems.superstructure.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import lombok.Setter;

import org.frogforce503.lib.math.Range;
import org.frogforce503.lib.subsystem.FFSubsystemBase;
import org.frogforce503.lib.util.LoggedTracer;
import org.frogforce503.robot2025.Robot;
import org.littletonrobotics.junction.Logger;

public class Wrist extends FFSubsystemBase {
    private final WristIO io;
    private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

    // Constants
    private final Range motionRange = Robot.bot.getWristConfig().motionRange();
    @Setter private ArmFeedforward feedforward = Robot.bot.getWristConfig().kFF().getArmFF();

    // Control
    private double targetAngleRad = WristConstants.START;
    
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
            var goalState = motionRange.clamp(targetAngleRad);

            atGoal = isAtAngle(goalState, WristConstants.kTolerance);

            if (atGoal) {
                stop();
            } else {
                io.runPosition(goalState, feedforward.calculate(goalState, 0.0));
            }

            // Log state
            Logger.recordOutput("Wrist/GoalPositionRad", goalState);
            Logger.recordOutput("Wrist/AtGoal", atGoal);
        } else {
            // Reset setpoint
            targetAngleRad = 0.0;
      
            // Clear logs
            Logger.recordOutput("Wrist/GoalPositionRad", 0.0);
            Logger.recordOutput("Wrist/AtGoal", true);
        }

        Logger.recordOutput("Wrist/CurrentPositionRad", getRelativeAngleRad());
        Logger.recordOutput("Wrist/AbsolutePositionRad", getAbsoluteAngleRad());

        // Record cycle time
        LoggedTracer.record("Wrist");
    }

    public double getRelativeAngleRad() {
        return inputs.data.relativePositionRad();
    }

    public double getAbsoluteAngleRad() {
        return inputs.data.absolutePositionRad();
    }

    // Actions
    public void setEncoderPosition(double position) {
        io.setRelativeEncoderPosition(position);
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

    public void setAngle(double angleRad) {
        this.shouldRunPosition = true;
        this.targetAngleRad = angleRad;
    }

    public boolean isAtAngle(double angleRad, double tolerance) {
        return MathUtil.isNear(angleRad, getRelativeAngleRad(), tolerance);
    }
}