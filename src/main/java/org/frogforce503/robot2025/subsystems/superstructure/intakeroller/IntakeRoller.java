package org.frogforce503.robot2025.subsystems.superstructure.intakeroller;

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

public class IntakeRoller extends FFSubsystemBase {
    private final IntakeRollerIO io;
    private final IntakeRollerIOInputsAutoLogged inputs = new IntakeRollerIOInputsAutoLogged();

    // Constants
    @Setter private SimpleMotorFeedforward feedforward = Robot.bot.getIntakeRollerConfig().kFF().getSimpleMotorFF();
    private final Debouncer algaeIntakeDebouncer = new Debouncer(0.5, DebounceType.kRising);

    // Control
    private double targetVelocityRPM = IntakeRollerConstants.START;

    private boolean shouldRunVelocity = false;
    private boolean atGoal = false;

    public IntakeRoller(IntakeRollerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        super.periodic();

        io.updateInputs(inputs);
        Logger.processInputs("IntakeRoller", inputs);

        // Run velocity mode unless requested to stop
        if (shouldRunVelocity && RobotState.isEnabled()) {
            atGoal = isAtVelocity(targetVelocityRPM, IntakeRollerConstants.kRollerTolerance);
            io.runVelocity(targetVelocityRPM, feedforward.calculate(targetVelocityRPM));

            // Log state
            Logger.recordOutput("IntakeRoller/SetpointVelocityRPM", targetVelocityRPM);
            Logger.recordOutput("IntakeRoller/AtGoal", atGoal);
        } else {
            // Reset setpoint
            targetVelocityRPM = 0.0;

            // Clear logs
            Logger.recordOutput("IntakeRoller/SetpointVelocityRPM", 0.0);
            Logger.recordOutput("IntakeRoller/AtGoal", true);
        }

        Logger.recordOutput("IntakeRoller/CurrentVelocityRPM", getVelocityRPM());

        // Record cycle time
        LoggedTracer.record("IntakeRoller");
    }

    public double getVelocityRPM() {
        return inputs.data.velocityRPM();
    }

    public boolean algaeCurrentThresholdForHoldMet() {
        if (RobotBase.isSimulation()) {
            return true;
        }

        return algaeIntakeDebouncer.calculate(
            inputs.data.statorCurrentAmps() > 15);
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
        this.shouldRunVelocity = false;
        io.runOpenLoop(output);
    }

    public void setVelocity(double velocityRPM) {
        this.shouldRunVelocity = true;
        this.targetVelocityRPM = velocityRPM;
    }

    public boolean isAtVelocity(double velocityRPM, double tolerance) {
        return MathUtil.isNear(velocityRPM, getVelocityRPM(), tolerance);
    }
}