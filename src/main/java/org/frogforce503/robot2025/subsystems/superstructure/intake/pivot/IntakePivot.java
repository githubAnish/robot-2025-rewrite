package org.frogforce503.robot2025.subsystems.superstructure.intake.pivot;

import org.frogforce503.robot2025.Constants;
import org.frogforce503.robot2025.Robot;
import org.frogforce503.robot2025.subsystems.superstructure.intake.IntakeGoal;
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

public class IntakePivot extends FFSubsystemBase {
    private final PivotIO io;
    private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

    // Constants
    private final double parallelToGroundAngleDeg = 107;
    private final Range range = Robot.bot.getIntakeConfig().pivotRange();
    private ArmFeedforward feedforward = Robot.bot.getIntakeConfig().pivotPIDF().toArmFF();
    private final double tolerance = 1.0;

    // Control
    private double targetPivotAngle = IntakeGoal.START.getPivotAngle();

    private boolean shouldRunProfile = false;
    private TrapezoidProfile profile;
    @Getter private State setpoint = new State();
    private boolean atGoal = false;

    // Tuning
    private LoggedNetworkBoolean tuningEnabled =
        new LoggedNetworkBoolean("Tuning/IntakePivot/Tuning?", false);
    
    private TuningService<PIDFConfig> pidfTuningService =
        new PIDFTuningService("IntakePivot", Robot.bot.getIntakeConfig().pivotPIDF());

    private TuningService<Constraints> speedTuningService =
        new SpeedConstraintsTuningService("IntakePivot", Robot.bot.getIntakeConfig().pivotConstraints());

    public IntakePivot(PivotIO pivotIO) {
        this.io = pivotIO;

        profile =
            new TrapezoidProfile(
                Robot.bot.getIntakeConfig().pivotConstraints());
    }

    @Override
    public void periodic() {
        super.periodic();

        io.updateInputs(inputs);
        Logger.processInputs("IntakePivot", inputs);

        // Update tunable numbers
        tuningExecutor().accept(tuningEnabled.get());

        // Update profile
        if (shouldRunProfile) {
            var goalState =
                new State(
                    range.clamp(targetPivotAngle),
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

            atGoal = isPivotAtAngle(goalState.position);

            if (atGoal) {
                stop();
            } else {
                double accel = (setpoint.velocity - previousVelocity) / Constants.loopPeriodSecs;
                io.runPosition(setpoint.position, feedforward.calculate(Math.toRadians(setpoint.position - parallelToGroundAngleDeg), setpoint.velocity, accel));
            }

            // Log state
            Logger.recordOutput("IntakePivot/Profile/SetpointPosition", setpoint.position);
            Logger.recordOutput("IntakePivot/Profile/SetpointVelocity", setpoint.velocity);
            Logger.recordOutput("IntakePivot/Profile/GoalPosition", goalState.position);
            Logger.recordOutput("IntakePivot/AtGoal", atGoal);
        } else {
            // Reset setpoint
            setpoint = new State(getAngle(), 0.0);
      
            // Clear logs
            Logger.recordOutput("IntakePivot/Profile/SetpointPosition", 0.0);
            Logger.recordOutput("IntakePivot/Profile/SetpointVelocity", 0.0);
            Logger.recordOutput("IntakePivot/Profile/GoalPosition", 0.0);
            Logger.recordOutput("IntakePivot/AtGoal", true);
        }

        Logger.recordOutput("IntakePivot/CurrentPosition", getAngle());

        // Record cycle time
        LoggedTracer.record("IntakePivot");
    }

    @Override
    public BooleanConsumer tuningExecutor() {
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

    public void runOpenLoop(double pivotOutput, double rollerOutput) {
        shouldRunProfile = false;
        io.runOpenLoop(pivotOutput);
    }

    public boolean isPivotAtAngle(double setpointAngle, double tolerance) {
        return MathUtil.isNear(setpointAngle, getAngle(), tolerance);
    }

    public boolean isPivotAtAngle(double setpointAngle) {
        return isPivotAtAngle(setpointAngle, tolerance);
    }

    public void setGoal(IntakeGoal goal) {
        this.shouldRunProfile = true;
        this.targetPivotAngle = goal.getPivotAngle();
    }
}