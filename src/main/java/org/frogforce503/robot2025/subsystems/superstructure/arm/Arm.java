package org.frogforce503.robot2025.subsystems.superstructure.arm;

import org.frogforce503.robot2025.Constants;
import org.frogforce503.robot2025.Robot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.util.function.BooleanConsumer;
import lombok.Getter;

import org.frogforce503.lib.motorcontrol.tuning.TuningService;
import org.frogforce503.lib.motorcontrol.tuning.pidf.PIDFConfig;
import org.frogforce503.lib.motorcontrol.tuning.pidf.PIDFTuningService;
import org.frogforce503.lib.motorcontrol.tuning.speed.SpeedConstraintsTuningService;
import org.frogforce503.lib.math.Range;
import org.frogforce503.lib.subsystem.FFSubsystemBase;
import org.frogforce503.lib.util.LoggedTracer;

public class Arm extends FFSubsystemBase {
    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    // Constants
    private final double parallelToGroundAngleDeg = 88.5;
    private final Range range = Robot.bot.getArmConfig().range();
    private ArmFeedforward feedforward = Robot.bot.getArmConfig().kPIDF().toArmFF();
    private final double tolerance = 0.5;

    // Control
    private double targetAngle = ArmGoal.START.getAngle();

    private boolean shouldRunProfile = false;
    private TrapezoidProfile profile;
    @Getter private State setpoint = new State();
    private boolean atGoal = false;

    // Tuning
    private LoggedNetworkBoolean tuningEnabled =
        new LoggedNetworkBoolean("Tuning/Arm/Tuning?", false);

    private TuningService<PIDFConfig> pidfTuningService =
        new PIDFTuningService("Arm", Robot.bot.getArmConfig().kPIDF());

    private TuningService<Constraints> speedTuningService =
        new SpeedConstraintsTuningService("Arm", Robot.bot.getArmConfig().kConstraints());

    public Arm(ArmIO io) {
        this.io = io;

        profile =
            new TrapezoidProfile(
                Robot.bot.getArmConfig().kConstraints());
    }

    @Override
    public void periodic() {
        super.periodic();

        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);

        // Update tunable numbers
        tuningExecutor().accept(tuningEnabled.get());

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

            atGoal = isAtAngle(goalState.position);

            if (atGoal) {
                stop();
            } else {
                double accel = (setpoint.velocity - previousVelocity) / Constants.loopPeriodSecs;
                io.runPosition(setpoint.position, feedforward.calculate(Math.toRadians(setpoint.position - parallelToGroundAngleDeg), setpoint.velocity, accel));
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

    @Override
    protected BooleanConsumer tuningExecutor() {
        return tuningEnabled -> {
            pidfTuningService.setTuning(tuningEnabled);
            speedTuningService.setTuning(tuningEnabled);
            
            if (tuningEnabled) {
                PIDFConfig newPIDFConfig = pidfTuningService.getUpdatedConfig();
                Constraints newSpeedConfig = speedTuningService.getUpdatedConfig();

                io.setPID(
                    newPIDFConfig.kP(),
                    newPIDFConfig.kI(),
                    newPIDFConfig.kD());

                feedforward = newPIDFConfig.toArmFF();

                profile =
                    new TrapezoidProfile(
                        new Constraints(
                            newSpeedConfig.maxVelocity,
                            newSpeedConfig.maxAcceleration));
            }
        };
    }

    public double getAngle() {
        return inputs.data.position();
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

    public boolean isAtAngle(double setpointAngle, double tolerance) {
        return MathUtil.isNear(setpointAngle, getAngle(), tolerance);
    }

    public boolean isAtAngle(double setpointAngle) {
        return isAtAngle(setpointAngle, tolerance);
    }

    public void setGoal(ArmGoal goal) {
        this.shouldRunProfile = true;
        this.targetAngle = goal.getAngle();
    }
}