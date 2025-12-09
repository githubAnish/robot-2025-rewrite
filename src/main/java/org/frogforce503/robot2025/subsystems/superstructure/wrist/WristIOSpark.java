package org.frogforce503.robot2025.subsystems.superstructure.wrist;

import org.frogforce503.lib.motorcontrol.SparkUtil;
import org.frogforce503.robot2025.Robot;
import org.frogforce503.robot2025.constants.subsystem.subsystemconfig.WristConfig;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.filter.Debouncer;
import lombok.Getter;

public class WristIOSpark implements WristIO {
    // Hardware
    @Getter private final SparkMax motor;
    private final RelativeEncoder relativeEncoder;
    private final SparkAbsoluteEncoder absoluteEncoder;

    // Control
    private final SparkClosedLoopController controller;

    // Config
    private SparkMaxConfig config = new SparkMaxConfig();

    // Filters
    private final Debouncer connectedDebouncer = new Debouncer(.5);

    public WristIOSpark() {
        final WristConfig wristConfig = Robot.bot.getWristConfig();

        // Initialize motor
        motor = new SparkMax(wristConfig.id(), MotorType.kBrushless);
        relativeEncoder = motor.getEncoder();
        absoluteEncoder = motor.getAbsoluteEncoder();
        controller = motor.getClosedLoopController();

        // Configure motor
        config.inverted(wristConfig.inverted());
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(wristConfig.statorCurrentLimit());
        config.voltageCompensation(12.0);

        config
            .encoder
                .positionConversionFactor((1 / wristConfig.mechanismRatio()) * (2 * Math.PI)) // convert rotations to radians
                .velocityConversionFactor((1 / wristConfig.mechanismRatio()) * (2 * Math.PI) / 60) // convert RPM to rad/sec
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);

        config
            .absoluteEncoder
                .zeroOffset(wristConfig.zeroOffset())
                .positionConversionFactor(2 * Math.PI) // convert rotations to radians, mounted on axis wrist rotates on, so no need to apply gear ratio
                .velocityConversionFactor(2 * Math.PI / 60) // convert RPM to rad/sec, mounted on axis wrist rotates on, so no need to apply gear ratio
                .zeroCentered(true)
                .averageDepth(2)
                .setSparkMaxDataPortConfig();

        config
            .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(wristConfig.kPID().kP(), wristConfig.kPID().kI(), wristConfig.kPID().kD());

        SparkUtil.optimizeSignals(config, true, false);

        motor.clearFaults();

        // Apply configuration
        SparkUtil.configure(motor, config, true);

        setRelativeEncoderPosition(0.0);
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.data =
            new WristIOData(
                connectedDebouncer.calculate(motor.getLastError() == REVLibError.kOk),
                relativeEncoder.getPosition(),
                absoluteEncoder.getPosition(),
                relativeEncoder.getVelocity(),
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
    public void runPosition(double positionRad, double feedforward) {
        controller.setReference(positionRad, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedforward);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        config.closedLoop.pid(kP, kI, kD, ClosedLoopSlot.kSlot0);
        SparkUtil.configure(motor, config, false);
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        config.idleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
        SparkUtil.configure(motor, config, false);
    }

    @Override
    public void setRelativeEncoderPosition(double position) {
        relativeEncoder.setPosition(position);
    }
}