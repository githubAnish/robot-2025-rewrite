package org.frogforce503.robot2025.subsystems.superstructure.wrist;

import org.frogforce503.lib.motorcontrol.SparkUtil;
import org.frogforce503.lib.motorcontrol.tuning.pidf.PIDFConfig;
import org.frogforce503.robot2025.Robot;

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

public class WristIOSpark implements WristIO {
    // Hardware
    private SparkMax motor;
    private RelativeEncoder relativeEncoder;
    private SparkAbsoluteEncoder absoluteEncoder;

    // Control
    private SparkClosedLoopController pidController;

    // Config
    private PIDFConfig currentPidConfig = Robot.bot.getWristConfig().kPIDF(); // Buffer variable to avoid calling configAccessor
    private IdleMode currentIdleMode = IdleMode.kBrake; // Buffer variable to avoid calling configAccessor

    private SparkMaxConfig config = new SparkMaxConfig();
    private final int STATOR_CURRENT_LIMIT = 40;
    private final double ABSOLUTE_CONVERSION_FACTOR = 360.0;
    private final double RELATIVE_CONVERSION_FACTOR = 9.0;

    // Connected Debouncers
    private final Debouncer connectedDebouncer = new Debouncer(.5);

    public WristIOSpark() {
        motor = new SparkMax(Robot.bot.getWristConfig().wristID(), MotorType.kBrushless);
        relativeEncoder = motor.getEncoder();
        absoluteEncoder = motor.getAbsoluteEncoder();

        pidController = motor.getClosedLoopController();

        // Configure motor
        config
            .absoluteEncoder
                .zeroOffset(Robot.bot.getWristConfig().wristOffset())
                .positionConversionFactor(ABSOLUTE_CONVERSION_FACTOR)
                .setSparkMaxDataPortConfig();

        config
            .encoder
                .positionConversionFactor(RELATIVE_CONVERSION_FACTOR);

        config
            .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(
                    currentPidConfig.kP(),
                    currentPidConfig.kI(),
                    currentPidConfig.kD(),
                    ClosedLoopSlot.kSlot0);

        config.inverted(Robot.bot.getWristConfig().wristInverted());
        config.smartCurrentLimit(STATOR_CURRENT_LIMIT);
        config.voltageCompensation(12);
        config.idleMode(currentIdleMode);

        motor.clearFaults();

        // Apply configuration
        SparkUtil.configure(motor, config, true);
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.data =
            new WristIOData(
                connectedDebouncer.calculate(motor.getLastError() == REVLibError.kOk),
                relativeEncoder.getPosition(),
                absoluteEncoder.getPosition(),
                relativeEncoder.getVelocity(),
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
        if (currentPidConfig.kP() != kP || currentPidConfig.kI() != kI || currentPidConfig.kD() != kD) {
            config.closedLoop.pid(kP, kI, kD, ClosedLoopSlot.kSlot0);
            SparkUtil.configure(motor, config, false);
            
            currentPidConfig = new PIDFConfig(kP, kI, kD);
        }
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        IdleMode request = enabled ? IdleMode.kBrake : IdleMode.kCoast;

        if (request != currentIdleMode) { // Doesn't set brake mode if it's already set
            config.idleMode(request);
            SparkUtil.configure(motor, config, false);
            
            currentIdleMode = request;
        }
    }

    @Override
    public void setEncoderPosition(double position) {
        relativeEncoder.setPosition(position);
    }
}