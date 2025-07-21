package org.frogforce503.robot2025.subsystems.climber;

import org.frogforce503.lib.control.TuningService;
import org.frogforce503.lib.control.pidf.PIDFConfig;
import org.frogforce503.lib.control.pidf.PIDFTuningService;
import org.frogforce503.lib.subsystem.FFSubsystemBase;
import org.frogforce503.lib.util.LoggedTracer;
import org.frogforce503.robot2025.Robot;
import org.frogforce503.robot2025.subsystems.superstructure.sensors.DigitalIO;
import org.frogforce503.robot2025.subsystems.superstructure.sensors.DigitalIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Climber extends FFSubsystemBase {
    private final ClimberIO climberIO;
    private final ClimberIOInputsAutoLogged climberInputs = new ClimberIOInputsAutoLogged();

    private final DigitalIO digitalIO;
    private final DigitalIOInputsAutoLogged digitalInputs = new DigitalIOInputsAutoLogged();

    private final Debouncer winchFilter = new Debouncer(0.1, DebounceType.kRising);

    private boolean holdRequested = false;

    // Tuning
    private TuningService<PIDFConfig> pidfTuningService =
        new PIDFTuningService("Climber",
            new PIDFConfig(
                Robot.bot.climberConstants.kP(),
                Robot.bot.climberConstants.kI(),
                Robot.bot.climberConstants.kD()));

    // Overrides
    private LoggedNetworkBoolean tuningEnabled =
        new LoggedNetworkBoolean("Tuning/Climber/Tuning?", false);

    private LoggedNetworkBoolean coastOverride =
        new LoggedNetworkBoolean("Coast Mode/Climber", false);

    private boolean requestCurrentControl = true;

    // Alerts
    private final Alert coastModeWhileRunning =
        new Alert("Climber/Warnings", "Climber is in coast mode while running!", Alert.AlertType.kWarning);

    public enum ClimberGoal {
        IDLE(0.0),

        SLOW_WIND(0.05),
        FAST_WIND(1.0),

        HOLD(0.3);
        
        private double current;
        
        private ClimberGoal(double current) {
            this.current = current;
        }
    }

    public ClimberGoal currentGoal = ClimberGoal.IDLE;

    public Climber(ClimberIO climberIO, DigitalIO digitalIO) {
        this.climberIO = climberIO;
        this.digitalIO = digitalIO;
    }

    @Override
    public void periodic() {
        climberIO.updateInputs(climberInputs);
        Logger.processInputs("Climber/Winch", climberInputs);

        digitalIO.updateInputs(digitalInputs);
        Logger.processInputs("Climber/LimitSwitch", digitalInputs);

        coastModeWhileRunning
            .set(coastOverride.get() && !RobotState.isDisabled());

        // Update tunable numbers
        tuningExecutor().accept(tuningEnabled.get());

        // Set coast mode
        if (RobotState.isDisabled()) {
            setBrakeMode(!coastOverride.get());
        }

        if (currentThresholdForHoldMet()) {
            holdRequested = true;
        }

        if (holdRequested && currentGoal != ClimberGoal.SLOW_WIND) {
            currentGoal = ClimberGoal.HOLD;
        }

        if (requestCurrentControl) {
            climberIO.runTorqueCurrent(currentGoal.current);
        }

        Logger.recordOutput("Climber/Goal", currentGoal.name());
        Logger.recordOutput("Climber/Reached Goal", atGoal());

        // Record cycle time
        LoggedTracer.record("Climber");
    }

    private boolean currentThresholdForHoldMet() {
        return winchFilter.calculate(
            climberInputs.data.statorCurrentAmps() > 63);
    }

    @Override
    public BooleanConsumer tuningExecutor() {
        return tuningEnabled -> {
            pidfTuningService.setTuning(tuningEnabled);
            
            if (tuningEnabled) {
                PIDFConfig newPIDFConfig = pidfTuningService.getUpdatedConfig();

                climberIO.setPID(
                    newPIDFConfig.kP(),
                    newPIDFConfig.kI(),
                    newPIDFConfig.kD());
            }
        };
    }

    @Override
    public boolean atGoal() {
        return MathUtil.isNear(currentGoal.current, climberInputs.data.statorCurrentAmps(), 10);
    }

    public void setBrakeMode(boolean enabled) {
        climberIO.setBrakeMode(enabled);
    }

    @Override
    public Command stop() {
        return Commands.sequence(
            runOnce(() -> requestCurrentControl = false),
            runOnce(climberIO::stop)
        );
    }

    @Override
    public Command runManual(double output) {
        return Commands.sequence(
            runOnce(() -> requestCurrentControl = false),
            run(() -> climberIO.runOpenLoop(output))
        );
    }

    public Command runGoal(ClimberGoal goal) {
        return Commands.sequence(
            runOnce(() -> requestCurrentControl = true),
            runOnce(() -> currentGoal = goal),
            Commands.waitUntil(this::atGoal)
        );
    }
}