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
    @Setter private SimpleMotorFeedforward feedforward;
    private final Debouncer coralDebouncer = new Debouncer(0.1);
    private final Debouncer algaeDebouncer = new Debouncer(0.25);

    // Control
    private double targetLeftVelocityRadPerSec = ClawConstants.START;
    private double targetRightVelocityRadPerSec = ClawConstants.START;

    private boolean shouldRunVelocity = false;
    private boolean atGoal = false;

    public Claw(ClawIO io) {
        this.io = io;

        feedforward = Robot.bot.getClawConfig().kFF().getSimpleMotorFF();
    }

    @Override
    public void periodic() {
        super.periodic();

        io.updateInputs(inputs);
        Logger.processInputs("Claw", inputs);

        // Run velocity mode unless requested to stop
        if (shouldRunVelocity && RobotState.isEnabled()) {
            atGoal = isAtVelocity(targetLeftVelocityRadPerSec, targetRightVelocityRadPerSec, ClawConstants.kTolerance);
            io.runVelocity(targetLeftVelocityRadPerSec, targetRightVelocityRadPerSec, feedforward.calculate(targetLeftVelocityRadPerSec), feedforward.calculate(targetRightVelocityRadPerSec));

            // Log state
            Logger.recordOutput("Claw/LeftSetpointVelocityRadPerSec", targetLeftVelocityRadPerSec);
            Logger.recordOutput("Claw/RightSetpointVelocityRadPerSec", targetRightVelocityRadPerSec);
            Logger.recordOutput("Claw/AtGoal", atGoal);
        } else {
            // Reset setpoint
            targetLeftVelocityRadPerSec = 0.0;
            targetRightVelocityRadPerSec = 0.0;

            // Clear logs
            Logger.recordOutput("Claw/LeftSetpointVelocityRadPerSec", 0.0);
            Logger.recordOutput("Claw/RightSetpointVelocityRadPerSec", 0.0);
            Logger.recordOutput("Claw/AtGoal", true);
        }

        Logger.recordOutput("Claw/LeftCurrentVelocityRadPerSec", getLeftVelocityRadPerSec());
        Logger.recordOutput("Claw/RightCurrentVelocityRadPerSec", getRightVelocityRadPerSec());

        // Record cycle time
        LoggedTracer.record("Claw");
    }

    public double getLeftVelocityRadPerSec() {
        return inputs.leftData.velocityRadPerSec();
    }

    public double getRightVelocityRadPerSec() {
        return inputs.rightData.velocityRadPerSec();
    }

    public boolean coralCurrentThresholdForIntookMet() {
        return coralDebouncer.calculate(
            inputs.leftData.statorCurrentAmps() > 10 ||
            inputs.rightData.statorCurrentAmps() > 10);
    }

    public boolean algaeCurrentThresholdForHoldMet() {
        return algaeDebouncer.calculate(
            inputs.leftData.statorCurrentAmps() > 15 ||
            inputs.rightData.statorCurrentAmps() > 15);
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

    public void runVolts(double leftVolts, double rightVolts) {
        this.shouldRunVelocity = false;
        io.runVolts(leftVolts, rightVolts);
    }

    public void setVelocity(double leftVelocityRadPerSec, double rightVelocityRadPerSec) {
        this.shouldRunVelocity = true;
        this.targetLeftVelocityRadPerSec = leftVelocityRadPerSec;
        this.targetRightVelocityRadPerSec = rightVelocityRadPerSec;
    }

    public void setVelocity(double velocityRadPerSec) {
        setVelocity(velocityRadPerSec, velocityRadPerSec);
    }

    public boolean isAtVelocity(double leftVelocityRadPerSec, double rightVelocityRadPerSec, double tolerance) {
        return
            MathUtil.isNear(leftVelocityRadPerSec, getLeftVelocityRadPerSec(), tolerance) &&
            MathUtil.isNear(rightVelocityRadPerSec, getRightVelocityRadPerSec(), tolerance);
    }
}