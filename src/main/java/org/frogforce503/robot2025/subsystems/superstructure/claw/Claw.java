package org.frogforce503.robot2025.subsystems.superstructure.claw;

import org.frogforce503.lib.motorcontrol.tuning.TuningService;
import org.frogforce503.lib.motorcontrol.tuning.pidf.PIDFConfig;
import org.frogforce503.lib.motorcontrol.tuning.pidf.PIDFTuningService;
import org.frogforce503.lib.subsystem.FFSubsystemBase;
import org.frogforce503.lib.util.LoggedTracer;
import org.frogforce503.robot2025.Robot;
import org.frogforce503.robot2025.subsystems.superstructure.sensors.CoralSensorIO;
import org.frogforce503.robot2025.subsystems.superstructure.sensors.CoralSensorIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.RobotBase;
import lombok.Getter;
import lombok.Setter;

public class Claw extends FFSubsystemBase {
    private final ClawIO clawIO;
    private final ClawIOInputsAutoLogged clawInputs = new ClawIOInputsAutoLogged();

    private final CoralSensorIO coralSensorIO;
    private final CoralSensorIOInputsAutoLogged coralSensorInputs = new CoralSensorIOInputsAutoLogged();

    // Constants
    private final double tolerance = 25.0;
    private final Debouncer coralFilter = new Debouncer(0.1);
    private final Debouncer algaeFilter = new Debouncer(0.25, DebounceType.kRising);

    // Control
    private double targetLeftVelocity = ClawGoal.START.getLeftVelocity();
    private double targetRightVelocity = ClawGoal.START.getRightVelocity();

    private boolean shouldRunVelocity = false;
    private boolean atGoal = false;

    // Storage
    @Setter @Getter private boolean hasCoral = false;
    @Setter @Getter private boolean hasAlgae = false;

    // Tuning
    private LoggedNetworkBoolean tuningEnabled =
        new LoggedNetworkBoolean("Tuning/Claw/Tuning?", false);

    private TuningService<PIDFConfig> pidfTuningService =
        new PIDFTuningService("Claw", Robot.bot.getClawConfig().kPIDF());

    public Claw(ClawIO clawIO, CoralSensorIO coralSensorIO) {
        this.clawIO = clawIO;
        this.coralSensorIO = coralSensorIO;
    }

    @Override
    public void periodic() {
        super.periodic();

        clawIO.updateInputs(clawInputs);
        Logger.processInputs("Claw", clawInputs);

        coralSensorIO.updateInputs(coralSensorInputs);
        Logger.processInputs("CoralSensors", coralSensorInputs);

        // Update tunable numbers
        tuningExecutor().accept(tuningEnabled.get());

        // Run velocity mode unless requested to stop
        if (shouldRunVelocity) {
            atGoal = isAtVelocity(targetLeftVelocity, targetRightVelocity);
            clawIO.runVolts(targetLeftVelocity, targetRightVelocity);

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

        Logger.recordOutput("Claw/Claw/HasCoral", hasCoral);
        Logger.recordOutput("Claw/HasAlgae", hasAlgae);

        // Record cycle time
        LoggedTracer.record("Claw");
    }

    @Override
    public BooleanConsumer tuningExecutor() {
        return tuningEnabled -> {
            pidfTuningService.setTuning(tuningEnabled);
            
            if (tuningEnabled) {
                PIDFConfig newPIDFConfig = pidfTuningService.getUpdatedConfig();

                clawIO.setPID(
                    newPIDFConfig.kP(),
                    newPIDFConfig.kI(),
                    newPIDFConfig.kD());
            }
        };
    }

    public double getLeftVelocity() {
        return clawInputs.leftMotorData.velocity();
    }

    public double getRightVelocity() {
        return clawInputs.rightMotorData.velocity();
    }

    public boolean upperSensorTriggered() {
        return coralSensorInputs.data.upperTripped();
    }

    public boolean lowerSensorTriggered() {
        return coralSensorInputs.data.lowerTripped();
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        clawIO.setBrakeMode(enabled);
    }

    @Override
    public void stop() {
        clawIO.stop();
    }

    public void runOpenLoop(double leftOutput, double rightOutput) {
        shouldRunVelocity = false;
        clawIO.runOpenLoop(leftOutput, rightOutput);
    }

    public boolean isAtVelocity(double leftSetpointVelocity, double rightSetpointVelocity, double tolerance) {
        return
            MathUtil.isNear(leftSetpointVelocity, getLeftVelocity(), tolerance) &&
            MathUtil.isNear(rightSetpointVelocity, getRightVelocity(), tolerance);
    }

    public boolean isAtVelocity(double leftSetpointVelocity, double rightSetpointVelocity) {
        return isAtVelocity(leftSetpointVelocity, rightSetpointVelocity, tolerance);
    }

    public void setGoal(ClawGoal goal) {
        this.shouldRunVelocity = true;
        this.targetLeftVelocity = goal.getLeftVelocity();
        this.targetRightVelocity = goal.getRightVelocity();
    }

    public boolean coralCurrentThresholdForIntookMet() {
        if (RobotBase.isSimulation()) {
            return true;
        }

        return
            coralFilter.calculate(
                clawInputs.leftMotorData.statorCurrentAmps() > 10 ||
                clawInputs.rightMotorData.statorCurrentAmps() > 10);
    }

    public boolean algaeCurrentThresholdForHoldMet() {
        if (RobotBase.isSimulation()) {
            return true;
        }

        return
            algaeFilter.calculate(
                clawInputs.leftMotorData.statorCurrentAmps() > 15 ||
                clawInputs.rightMotorData.statorCurrentAmps() > 15);
    }
}