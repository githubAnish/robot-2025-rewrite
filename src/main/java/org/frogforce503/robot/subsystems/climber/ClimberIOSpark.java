package org.frogforce503.robot.subsystems.climber;

import java.time.Duration;

import org.frogforce503.lib.motorcontrol.SparkUtil;
import org.frogforce503.robot.Robot;
import org.frogforce503.robot.constants.hardware.subsystem_config.ClimberConfig;
import org.frogforce503.robot.constants.hardware.subsystem_config.SensorConfig;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalGlitchFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import lombok.Getter;

public class ClimberIOSpark implements ClimberIO {
    // Hardware
    @Getter private final SparkMax motor;

    @Getter private final DigitalInput limitSwitch;

    // Config
    private SparkMaxConfig config = new SparkMaxConfig();

    // Filters
    private final Debouncer connectedDebouncer = new Debouncer(.5);
    private final DigitalGlitchFilter limitSwitchFilter = new DigitalGlitchFilter();
    
    public ClimberIOSpark() {
        final ClimberConfig climberConfig = Robot.bot.getClimberConfig();
        final SensorConfig sensorConfig = Robot.bot.getSensorConfig();

        // Initialize motor
        motor = new SparkMax(climberConfig.id(), MotorType.kBrushless);

        // Initialize limit switch
        limitSwitch = new DigitalInput(sensorConfig.winchLimitSwitchId());
        limitSwitchFilter.setPeriodNanoSeconds(Duration.ofMillis(100).toNanos());
        limitSwitchFilter.add(limitSwitch);

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
                motor.getMotorTemperature(),
                !limitSwitch.get());
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