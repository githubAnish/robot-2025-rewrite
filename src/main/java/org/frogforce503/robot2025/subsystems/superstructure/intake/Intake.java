package org.frogforce503.robot2025.subsystems.superstructure.intake;

import org.frogforce503.robot2025.Constants;
import org.frogforce503.robot2025.Robot;
import org.frogforce503.robot2025.subsystems.superstructure.intake.pivot.PivotIO;
import org.frogforce503.robot2025.subsystems.superstructure.intake.pivot.PivotIOInputsAutoLogged;
import org.frogforce503.robot2025.subsystems.superstructure.intake.roller.RollerIO;
import org.frogforce503.robot2025.subsystems.superstructure.intake.roller.RollerIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import lombok.Getter;

import org.frogforce503.lib.motorcontrol.tuning.TuningService;
import org.frogforce503.lib.motorcontrol.tuning.pidf.PIDFConfig;
import org.frogforce503.lib.motorcontrol.tuning.pidf.PIDFTuningService;
import org.frogforce503.lib.motorcontrol.tuning.speed.SpeedConstraintsTuningService;
import org.frogforce503.lib.math.Range;
import org.frogforce503.lib.subsystem.FFSubsystemBase;
import org.frogforce503.lib.util.LoggedTracer;

public class Intake extends FFSubsystemBase {
    private final PivotIO pivotIO;
    private final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();

    private final RollerIO rollerIO;
    private final RollerIOInputsAutoLogged rollerInputs = new RollerIOInputsAutoLogged();

    private final Debouncer algaeFilter = new Debouncer(0.5, DebounceType.kRising);

    // Constants
    private final Range range = Robot.bot.intakeConstants.pivotConstants().range();
    private ArmFeedforward feedforward = Robot.bot.intakeConstants.pivotConstants().kPIDF().toArmFeedforward();
    private final double parallelToGroundAngle = 107;

    // Control
    private TrapezoidProfile profile;
    @Getter private State setpoint = new State();

    // Tuning
    private TuningService<PIDFConfig> pidfTuningService =
        new PIDFTuningService("Intake", Robot.bot.intakeConstants.pivotConstants().kPIDF());

    private TuningService<Constraints> speedTuningService =
        new SpeedConstraintsTuningService("Intake", Robot.bot.intakeConstants.pivotConstants().kConstraints());

    // Overrides
    private LoggedNetworkBoolean tuningEnabled =
        new LoggedNetworkBoolean("Tuning/Intake/Tuning?", false);

    private LoggedNetworkBoolean coastOverride =
        new LoggedNetworkBoolean("Coast Mode/Intake", false);

    private boolean requestPositionControl = true;

    // Alerts
    private final Alert coastModeWhileRunning =
        new Alert("Intake is in coast mode while running!", AlertType.kError);

    @Getter private IntakeGoal currentGoal = IntakeGoal.IDLE;

    public Intake(PivotIO pivotIO, RollerIO rollerIO) {
        this.pivotIO = pivotIO;
        this.rollerIO = rollerIO;

        profile =
            new TrapezoidProfile(
                Robot.bot.intakeConstants.pivotConstants().kConstraints());
    }

    @Override
    public void periodic() {
        pivotIO.updateInputs(pivotInputs);
        Logger.processInputs("Intake/Pivot", pivotInputs);
        
        rollerIO.updateInputs(rollerInputs);
        Logger.processInputs("Intake/Roller", rollerInputs);

        coastModeWhileRunning
            .set(coastOverride.get() && !RobotState.isDisabled());

        // Update tunable numbers
        tuningExecutor().accept(tuningEnabled.get());

        // Set coast mode
        if (RobotState.isDisabled()) {
            setBrakeMode(!coastOverride.get());
        }

        // Run position mode unless requested to stop
        if (requestPositionControl) {
            var goalState =
                new State(
                    range.clamp(currentGoal.pivotPosition),
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

            double accel = (setpoint.velocity - previousVelocity) / Constants.loopPeriodSecs;

            pivotIO.runPosition(setpoint.position, feedforward.calculate(Math.toRadians(currentGoal.pivotPosition - parallelToGroundAngle), setpoint.velocity, accel));
            rollerIO.runVolts(currentGoal.rollerVolts);

            // Log state
            Logger.recordOutput("Intake/Profile/SetpointPosition", setpoint.position);
            Logger.recordOutput("Intake/Profile/SetpointVelocity", setpoint.velocity);
            Logger.recordOutput("Intake/Profile/GoalPosition", goalState.position);
        }

        Logger.recordOutput("Intake/Goal", currentGoal.name());
        Logger.recordOutput("Intake/Reached Goal", atGoal());

        // Record cycle time
        LoggedTracer.record("Intake");
    }

    public boolean algaeCurrentThresholdForHoldMet() {
        if (RobotBase.isSimulation()) {
            return true;
        }

        return algaeFilter.calculate(
            rollerInputs.data.statorCurrentAmps() > 15);
    }

    public double getPivotPosition() {
        return pivotInputs.data.position();
    }

    @Override
    public BooleanConsumer tuningExecutor() {
        return tuningEnabled -> {
            pidfTuningService.setTuning(tuningEnabled);
            speedTuningService.setTuning(tuningEnabled);
            
            if (tuningEnabled) {
                PIDFConfig newPIDFConfig = pidfTuningService.getUpdatedConfig();
                Constraints newSpeedConfig = speedTuningService.getUpdatedConfig();

                pivotIO.setPID(
                    newPIDFConfig.kP(),
                    newPIDFConfig.kI(),
                    newPIDFConfig.kD());

                feedforward = new ArmFeedforward(newPIDFConfig.kS(), newPIDFConfig.kG(), newPIDFConfig.kV(), newPIDFConfig.kA());

                profile =
                    new TrapezoidProfile(
                        new Constraints(
                            newSpeedConfig.maxVelocity,
                            newSpeedConfig.maxAcceleration));
            }
        };
    }

    @Override
    public boolean atGoal() {
        return
            MathUtil.isNear(currentGoal.pivotPosition, pivotInputs.data.position(), 2) &&
            MathUtil.isNear(currentGoal.rollerVolts, rollerInputs.data.appliedVolts(), 1);
    }

    public void setBrakeMode(boolean enabled) {
        pivotIO.setBrakeMode(enabled);
        rollerIO.setBrakeMode(enabled);
    }

    @Override
    public Command stop() {
        return Commands.sequence(
            runOnce(() -> requestPositionControl = false),
            runOnce(pivotIO::stop),
            runOnce(rollerIO::stop)
        );
    }

    @Override
    public Command runManual(double output) {
        return Commands.sequence(
            runOnce(() -> requestPositionControl = false),
            run(() -> pivotIO.runOpenLoop(output))
        );
    }

    public Command runGoal(IntakeGoal goal) {
        return Commands.sequence(
            runOnce(() -> requestPositionControl = true),
            runOnce(() -> currentGoal = goal),
            Commands.waitUntil(this::atGoal)
        );
    }
}