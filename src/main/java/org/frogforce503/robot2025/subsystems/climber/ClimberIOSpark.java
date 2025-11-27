package org.frogforce503.robot2025.subsystems.climber;

import org.frogforce503.lib.motorcontrol.SparkUtil;
import org.frogforce503.robot2025.Robot;
import org.frogforce503.robot2025.config.subsystem.ClimberConfig;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import lombok.Getter;

public class ClimberIOSpark implements ClimberIO {
    // Hardware
    @Getter private final SparkMax motor;

    // Config
    private SparkMaxConfig config = new SparkMaxConfig();

    // Connected Debouncers
    private final Debouncer connectedDebouncer = new Debouncer(.5);
    
    public ClimberIOSpark() {
        final ClimberConfig climberConfig = Robot.bot.getClimberConfig();

        motor = new SparkMax(climberConfig.id(), MotorType.kBrushless);

        // Configure motor
        config.inverted(climberConfig.inverted());
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(climberConfig.statorCurrentLimit());
        config.voltageCompensation(12.0);

        SparkUtil.optimizeSignals(config, false, false);

        motor.clearFaults();

        // Apply configuration
        SparkUtil.configure(motor, config, true);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.data =
            new ClimberIOData(
                connectedDebouncer.calculate(motor.getLastError() == REVLibError.kOk),
                motor.getBusVoltage() * motor.getAppliedOutput(),
                motor.getOutputCurrent(),
                motor.getMotorTemperature());
    }

    @Override
    public void runOpenLoop(double output) {
        motor.set(output);
    }

    @Override
    public void runVolts(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        config.idleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
        SparkUtil.configure(motor, config, false);
    }
}