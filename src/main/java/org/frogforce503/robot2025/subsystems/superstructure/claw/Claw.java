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
import lombok.Setter;

public class Claw extends FFSubsystemBase {
    private final ClawIO io;
    private final ClawIOInputsAutoLogged inputs = new ClawIOInputsAutoLogged();

    // Constants
    @Setter private SimpleMotorFeedforward feedforward = Robot.bot.getClawConfig().kFF().getSimpleMotorFF();
    private final Debouncer coralFilter = new Debouncer(0.1);
    private final Debouncer algaeFilter = new Debouncer(0.25, DebounceType.kRising);

    // Control
    private double targetLeftVelocity = ClawConstants.START;
    private double targetRightVelocity = ClawConstants.START;

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
        if (shouldRunVelocity) {
            atGoal = isAtVelocity(targetLeftVelocity, targetRightVelocity, ClawConstants.kTolerance);
            io.runVelocity(targetLeftVelocity, targetRightVelocity, feedforward.calculate(targetLeftVelocity));

            // Log state
            Logger.recordOutput("Claw/LeftSetpointVelocity", targetLeftVelocity);
            Logger.recordOutput("Claw/RightSetpointVelocity", targetRightVelocity);
            Logger.recordOutput("Claw/AtGoal", atGoal);
        } else {
            // Reset setpoint
            targetLeftVelocity = 0.0;
            targetRightVelocity = 0.0;

            // Clear logs
            Logger.recordOutput("Claw/AtGoal", true);
        }

        Logger.recordOutput("Claw/LeftCurrentVelocity", getLeftVelocity());
        Logger.recordOutput("Claw/RightCurrentVelocity", getRightVelocity());

        // Record cycle time
        LoggedTracer.record("Claw");
    }

    public double getLeftVelocity() {
        return inputs.leftMotorData.velocity();
    }

    public double getRightVelocity() {
        return inputs.rightMotorData.velocity();
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

    public void runOpenLoop(double leftOutput, double rightOutput) {
        this.shouldRunVelocity = false;
        io.runOpenLoop(leftOutput, rightOutput);
    }

    public void setVelocity(double leftVelocity, double rightVelocity) {
        this.shouldRunVelocity = true;
        this.targetLeftVelocity = leftVelocity;
        this.targetRightVelocity = rightVelocity;
    }

    public void setVelocity(double velocity) {
        setVelocity(velocity, velocity);
    }

    public boolean isAtVelocity(double leftVelocity, double rightVelocity, double tolerance) {
        return
            MathUtil.isNear(leftVelocity, getLeftVelocity(), tolerance) &&
            MathUtil.isNear(rightVelocity, getRightVelocity(), tolerance);
    }

    public boolean coralCurrentThresholdForIntookMet() {
        if (RobotBase.isSimulation()) {
            return true;
        }

        return
            coralFilter.calculate(
                inputs.leftMotorData.statorCurrentAmps() > 10 ||
                inputs.rightMotorData.statorCurrentAmps() > 10);
    }

    public boolean algaeCurrentThresholdForHoldMet() {
        if (RobotBase.isSimulation()) {
            return true;
        }

        return
            algaeFilter.calculate(
                inputs.leftMotorData.statorCurrentAmps() > 15 ||
                inputs.rightMotorData.statorCurrentAmps() > 15);
    }
}