package org.frogforce503.robot2025.subsystems.superstructure.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.util.function.BooleanConsumer;

import org.frogforce503.lib.math.Range;
import org.frogforce503.lib.motorcontrol.tuning.TuningService;
import org.frogforce503.lib.motorcontrol.tuning.pidf.PIDFConfig;
import org.frogforce503.lib.motorcontrol.tuning.pidf.PIDFTuningService;
import org.frogforce503.lib.subsystem.FFSubsystemBase;
import org.frogforce503.lib.util.LoggedTracer;
import org.frogforce503.robot2025.Robot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class Wrist extends FFSubsystemBase {
    private final WristIO io;
    private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

    // Constants
    private final double parallelToGroundAngle = 90;
    private final Range range = Robot.bot.getWristConfig().range();
    private ArmFeedforward feedforward = Robot.bot.getWristConfig().kPIDF().toArmFF();
    private final double tolerance = 1.0;

    // Control
    private double targetAngle = WristGoal.START.getAngle();
    
    private boolean shouldRunPosition = false;
    private boolean atGoal = false;

    // Tuning
    private LoggedNetworkBoolean tuningEnabled =
        new LoggedNetworkBoolean("Tuning/Wrist/Tuning?", false);

    private TuningService<PIDFConfig> pidfTuningService =
        new PIDFTuningService("Wrist", Robot.bot.getWristConfig().kPIDF());

    public Wrist(WristIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        super.periodic();

        io.updateInputs(inputs);
        Logger.processInputs("Wrist", inputs);

        // Update tunable numbers
        tuningExecutor().accept(tuningEnabled.get());

        // Run position mode unless requested to stop
        if (shouldRunPosition) {
            var goalState = range.clamp(targetAngle);

            atGoal = isAtAngle(goalState);

            if (atGoal) {
                stop();
            } else {
                io.runPosition(goalState, feedforward.calculate(Math.toRadians(goalState - parallelToGroundAngle), 0.0));
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

    @Override
    protected BooleanConsumer tuningExecutor() {
        return tuningEnabled -> {
            pidfTuningService.setTuning(tuningEnabled);
            
            if (tuningEnabled) {
                PIDFConfig newPIDFConfig = pidfTuningService.getUpdatedConfig();

                io.setPID(
                    newPIDFConfig.kP(),
                    newPIDFConfig.kI(),
                    newPIDFConfig.kD());

                feedforward = newPIDFConfig.toArmFF();
            }
        };
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

    public boolean isAtAngle(double setpointAngle, double tolerance) {
        return MathUtil.isNear(setpointAngle, getRelativeAngle(), tolerance);
    }

    public boolean isAtAngle(double setpointAngle) {
        return isAtAngle(setpointAngle, tolerance);
    }

    public void setGoal(WristGoal goal) {
        this.shouldRunPosition = true;
        this.targetAngle = goal.getAngle();
    }
}