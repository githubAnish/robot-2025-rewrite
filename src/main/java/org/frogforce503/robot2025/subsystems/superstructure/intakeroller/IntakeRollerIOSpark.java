package org.frogforce503.robot2025.subsystems.superstructure.intakeroller;

import org.frogforce503.lib.motorcontrol.SparkUtil;
import org.frogforce503.robot2025.Robot;
import org.frogforce503.robot2025.config.subsystem.IntakeRollerConfig;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import lombok.Getter;

public class IntakeRollerIOSpark implements IntakeRollerIO {
    // Hardware
    @Getter private final SparkBase motor;
    private final RelativeEncoder encoder;

    // Control
    private final SparkClosedLoopController controller;

    // Config
    private SparkMaxConfig config = new SparkMaxConfig();

    // Connected Debouncers
    private final Debouncer connectedDebouncer = new Debouncer(.5);
    
    public IntakeRollerIOSpark() {
        final IntakeRollerConfig rollerConfig = Robot.bot.getIntakeRollerConfig();

        motor = rollerConfig.isSparkFlex()
            ? new SparkFlex(rollerConfig.id(), MotorType.kBrushless)
            : new SparkMax(rollerConfig.id(), MotorType.kBrushless);
        encoder = motor.getEncoder();
        controller = motor.getClosedLoopController();

        // Configure motor
        config.inverted(rollerConfig.inverted());
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(rollerConfig.statorCurrentLimit());
        config.voltageCompensation(12.0);

        config
            .encoder
                .positionConversionFactor(1 / rollerConfig.mechanismRatio())
                .velocityConversionFactor(1 / rollerConfig.mechanismRatio())
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);

        config
            .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(rollerConfig.kPID().kP(), rollerConfig.kPID().kI(), rollerConfig.kPID().kD());

        SparkUtil.optimizeSignals(config, false, false);

        motor.clearFaults();

        // Apply config
        SparkUtil.configure(motor, config, true);
    }

    @Override
    public void updateInputs(IntakeRollerIOInputs inputs) {
        inputs.data =
            new IntakeRollerIOData(
                connectedDebouncer.calculate(motor.getLastError() == REVLibError.kOk),
                encoder.getVelocity(),
                motor.getAppliedOutput() * motor.getBusVoltage(),
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
    public void runVelocity(double velocity, double feedforward) {
        controller.setReference(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0, feedforward);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        config.closedLoop.pid(kP, kI, kD);
        SparkUtil.configure(motor, config, false);
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        config.idleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
        SparkUtil.configure(motor, config, false);
    }
}