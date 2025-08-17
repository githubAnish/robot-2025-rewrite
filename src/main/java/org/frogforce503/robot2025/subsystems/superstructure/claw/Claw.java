package org.frogforce503.robot2025.subsystems.superstructure.claw;

import org.frogforce503.lib.subsystem.FFSubsystemBase;
import org.frogforce503.lib.util.LoggedTracer;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import lombok.Getter;

public class Claw extends FFSubsystemBase {
    private final ClawIO io;
    private final ClawIOInputsAutoLogged inputs = new ClawIOInputsAutoLogged();

    private final Debouncer coralFilter = new Debouncer(0.1);
    private final Debouncer algaeFilter = new Debouncer(0.25, DebounceType.kRising);

    // Overrides
    private LoggedNetworkBoolean coastOverride =
        new LoggedNetworkBoolean("Coast Mode/Claw", false);

    private boolean requestVelocityControl = true;

    // Alerts
    private final Alert coastModeWhileRunning =
        new Alert("Claw/Warnings", "Claw is in coast mode while running!", Alert.AlertType.kWarning);

    public enum ClawGoal {
        OFF(0),

        INTAKE_CORAL(0),
        INTAKE_ALGAE(0),

        HOLD_ALGAE(0),
        
        EJECT_CORAL(0),
        EJECT_CORAL_FOR_L1(0, 0),

        EJECT_ALGAE(0),
        SLOW_EJECT_ALGAE(0),

        HANDOFF_INTAKE_TO_CLAW(0);
        
        private double velocityLeft, velocityRight;

        private ClawGoal(double velocityLeft, double velocityRight) {
            this.velocityLeft = velocityLeft;
            this.velocityRight = velocityRight;
        }

        private ClawGoal(double volts) {
            this(volts, volts);
        }
    }

    @Getter private ClawGoal currentGoal = ClawGoal.OFF;

    public Claw(ClawIO clawIO) {
        this.io = clawIO;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Claw", inputs);

        coastModeWhileRunning
            .set(coastOverride.get() && !RobotState.isDisabled());

        // Set coast mode
        if (RobotState.isDisabled()) {
            setBrakeMode(!coastOverride.get());
        }

        if (requestVelocityControl) {
            io.runVolts(currentGoal.velocityLeft, currentGoal.velocityRight);
        }

        Logger.recordOutput("Claw/Goal", currentGoal.name());
        Logger.recordOutput("Claw/Reached Goal", atGoal());

        // Record cycle time
        LoggedTracer.record("Claw");
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

    @Override
    public boolean atGoal() {
        return
            MathUtil.isNear(currentGoal.velocityLeft, inputs.leftMotorData.velocity(), 50) &&
            MathUtil.isNear(currentGoal.velocityRight, inputs.rightMotorData.velocity(), 50);
    }

    public void setBrakeMode(boolean enabled) {
        io.setBrakeMode(enabled);
    }

    @Override
    public Command stop() {
        return Commands.sequence(
            runOnce(() -> requestVelocityControl = false),
            runOnce(io::stop)
        );
    }

    @Override
    public Command runManual(double output) {
        return Commands.sequence(
            runOnce(() -> requestVelocityControl = false),
            run(() -> io.runOpenLoop(output, output))
        );
    }

    public Command runGoal(ClawGoal goal) {
        return Commands.sequence(
            runOnce(() -> requestVelocityControl = true),
            runOnce(() -> currentGoal = goal),
            Commands.waitUntil(this::atGoal)
        );
    }
}