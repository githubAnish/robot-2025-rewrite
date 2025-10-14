package org.frogforce503.robot2025.subsystems.superstructure.wrist;

import org.frogforce503.lib.motorcontrol.SparkUtil;
import org.frogforce503.robot2025.Constants;

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
import com.revrobotics.spark.config.ClosedLoopConfigAccessor;
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
        motor = new SparkMax(Constants.bot.Wrist.wristID(), MotorType.kBrushless);
        mainEncoder = motor.getEncoder();
        seedEncoder = motor.getAbsoluteEncoder();

        pidController = motor.getClosedLoopController();

        // Configure motor
        config
            .absoluteEncoder
                .zeroOffset(Constants.bot.Wrist.wristOffset())
                .positionConversionFactor(ABSOLUTE_CONVERSION_FACTOR)
                .setSparkMaxDataPortConfig();

        config
            .encoder
                .positionConversionFactor(RELATIVE_CONVERSION_FACTOR);

        config
            .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(
                    Constants.bot.Wrist.kPIDF().kP(),
                    Constants.bot.Wrist.kPIDF().kI(),
                    Constants.bot.Wrist.kPIDF().kD(),
                    ClosedLoopSlot.kSlot0);

        config.inverted(Constants.bot.Wrist.wristInverted());

        config.smartCurrentLimit(STATOR_CURRENT_LIMIT);

        config.voltageCompensation(12);

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
        ClosedLoopConfigAccessor accessor = motor.configAccessor.closedLoop;

        if (accessor.getP() != kP || accessor.getI() != kI || accessor.getD() != kD) {
            config.closedLoop.pid(kP, kI, kD, ClosedLoopSlot.kSlot0);
        }

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