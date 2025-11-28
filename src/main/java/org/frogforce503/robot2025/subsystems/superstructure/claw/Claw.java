package org.frogforce503.robot2025.subsystems.superstructure.claw;

import org.frogforce503.lib.subsystem.FFSubsystemBase;
import org.frogforce503.lib.util.LoggedTracer;
import org.frogforce503.robot2025.Robot;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import lombok.Setter;

public class Claw extends FFSubsystemBase {
    private final ClawIO io;
    private final ClawIOInputsAutoLogged inputs = new ClawIOInputsAutoLogged();

    // Constants
    @Setter private SimpleMotorFeedforward feedforward = Robot.bot.getClawConfig().kFF().getSimpleMotorFF();
    private final Debouncer coralFilter = new Debouncer(0.1);
    private final Debouncer algaeFilter = new Debouncer(0.25, DebounceType.kRising);

    // Control
    private double targetLeftVelocityRPM = ClawConstants.START;
    private double targetRightVelocityRPM = ClawConstants.START;

    private boolean shouldRunVelocity = false;
    private boolean atGoal = false;

    public Claw(ClawIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        super.periodic();

        io.updateInputs(inputs);
        Logger.processInputs("Claw", inputs);

        // Run velocity mode unless requested to stop
        if (shouldRunVelocity && RobotState.isEnabled()) {
            atGoal = isAtVelocity(targetLeftVelocityRPM, targetRightVelocityRPM, ClawConstants.kTolerance);
            io.runVelocity(targetLeftVelocityRPM, targetRightVelocityRPM, feedforward.calculate((targetLeftVelocityRPM + targetRightVelocityRPM) / 2.0));

            // Log state
            Logger.recordOutput("Claw/LeftSetpointVelocityRPM", targetLeftVelocityRPM);
            Logger.recordOutput("Claw/RightSetpointVelocityRPM", targetRightVelocityRPM);
            Logger.recordOutput("Claw/AtGoal", atGoal);
        } else {
            // Reset setpoint
            targetLeftVelocityRPM = 0.0;
            targetRightVelocityRPM = 0.0;

            // Clear logs
            Logger.recordOutput("Claw/LeftSetpointVelocityRPM", 0.0);
            Logger.recordOutput("Claw/RightSetpointVelocityRPM", 0.0);
            Logger.recordOutput("Claw/AtGoal", true);
        }

        Logger.recordOutput("Claw/LeftCurrentVelocityRPM", getLeftVelocityRPM());
        Logger.recordOutput("Claw/RightCurrentVelocityRPM", getRightVelocityRPM());

        // Record cycle time
        LoggedTracer.record("Claw");
    }

    public double getLeftVelocityRPM() {
        return inputs.leftMotorData.velocityRPM();
    }

    public double getRightVelocityRPM() {
        return inputs.rightMotorData.velocityRPM();
    }

    public boolean coralCurrentThresholdForIntookMet() {
        if (RobotBase.isSimulation()) {
            return true;
        }

        return coralFilter.calculate(
            inputs.leftMotorData.statorCurrentAmps() > 10 ||
            inputs.rightMotorData.statorCurrentAmps() > 10);
    }

    public boolean algaeCurrentThresholdForHoldMet() {
        if (RobotBase.isSimulation()) {
            return true;
        }

        return algaeFilter.calculate(
            inputs.leftMotorData.statorCurrentAmps() > 15 ||
            inputs.rightMotorData.statorCurrentAmps() > 15);
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

    public void runOpenLoop(double leftOutput, double rightOutput) {
        this.shouldRunVelocity = false;
        io.runOpenLoop(leftOutput, rightOutput);
    }

    public void setVelocity(double leftVelocityRPM, double rightVelocityRPM) {
        this.shouldRunVelocity = true;
        this.targetLeftVelocityRPM = leftVelocityRPM;
        this.targetRightVelocityRPM = rightVelocityRPM;
    }

    public void setVelocity(double velocityRPM) {
        setVelocity(velocityRPM, velocityRPM);
    }

    public boolean isAtVelocity(double leftVelocityRPM, double rightVelocityRPM, double tolerance) {
        return
            MathUtil.isNear(leftVelocityRPM, getLeftVelocityRPM(), tolerance) &&
            MathUtil.isNear(rightVelocityRPM, getRightVelocityRPM(), tolerance);
    }
}