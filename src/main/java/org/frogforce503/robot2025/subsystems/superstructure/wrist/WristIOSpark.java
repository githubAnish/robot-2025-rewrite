package org.frogforce503.robot2025.subsystems.superstructure.wrist;

import org.frogforce503.lib.motor.SparkUtil;
import org.frogforce503.robot2025.Robot;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.filter.Debouncer;

public class WristIOSpark implements WristIO {
    // Hardware
    private SparkMax motor;
    private RelativeEncoder mainEncoder;
    private SparkAbsoluteEncoder seedEncoder;

    // Control
    private SparkClosedLoopController pidController;

    // Config
    private SparkMaxConfig config = new SparkMaxConfig();
    private final double ABSOLUTE_CONVERSION_FACTOR = 360.0;
    private final double RELATIVE_CONVERSION_FACTOR = 9.0;
    private final int STATOR_CURRENT_LIMIT = 40;

    // Connected Debouncers
    private final Debouncer connectedDebouncer = new Debouncer(.5);

    public WristIOSpark() {
        motor = SparkUtil.getSpark(Robot.bot.wristConstants.wristID(), false);
        mainEncoder = motor.getEncoder();
        seedEncoder = motor.getAbsoluteEncoder();

        pidController = motor.getClosedLoopController();

        // Configure motor
        config
            .absoluteEncoder
                .zeroOffset(Robot.bot.wristConstants.wristOffset())
                .positionConversionFactor(ABSOLUTE_CONVERSION_FACTOR)
                .setSparkMaxDataPortConfig();

        config
            .encoder
                .positionConversionFactor(RELATIVE_CONVERSION_FACTOR);

        config
            .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(
                    Robot.bot.wristConstants.kP(),
                    Robot.bot.wristConstants.kI(),
                    Robot.bot.wristConstants.kD(),
                    ClosedLoopSlot.kSlot0);

        config.inverted(Robot.bot.wristConstants.wristInverted());

        config.smartCurrentLimit(STATOR_CURRENT_LIMIT);

        motor.clearFaults();

        // Apply configuration
        SparkUtil.configure(motor, config, true);
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.data =
            new WristIOData(
                connectedDebouncer.calculate(motor.getLastError() == REVLibError.kOk),
                mainEncoder.getPosition(),
                seedEncoder.getPosition(),
                mainEncoder.getVelocity(),
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
    public void runPosition(double position, double feedforward) {
        pidController.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedforward);
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
    public void setEncoderPosition(double position) {
        mainEncoder.setPosition(position);
    }
}