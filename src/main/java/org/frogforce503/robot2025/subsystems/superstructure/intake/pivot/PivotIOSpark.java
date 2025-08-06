package org.frogforce503.robot2025.subsystems.superstructure.intake.pivot;

import org.frogforce503.lib.motorcontrol.SparkUtil;
import org.frogforce503.robot2025.Robot;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.filter.Debouncer;

public class PivotIOSpark implements PivotIO {
    // Hardware
    private SparkMax motor;
    private SparkAbsoluteEncoder encoder;

    // Control
    private SparkClosedLoopController pidController;

    // Config
    private SparkMaxConfig config = new SparkMaxConfig();
    private final double ABSOLUTE_CONVERSION_FACTOR = 360.0;
    private final int STATOR_CURRENT_LIMIT = 40;

    // Connected Debouncers
    private final Debouncer connectedDebouncer = new Debouncer(.5);

    public PivotIOSpark() {
        motor = SparkUtil.getSpark(Robot.bot.intakeConstants.pivotConstants().pivotID(), false);
        encoder = motor.getAbsoluteEncoder();

        pidController = motor.getClosedLoopController();

        // Configure motor
        config
            .absoluteEncoder
                .zeroOffset(Robot.bot.intakeConstants.pivotConstants().pivotOffset())
                .positionConversionFactor(ABSOLUTE_CONVERSION_FACTOR)
                .setSparkMaxDataPortConfig();

        config
            .closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(
                    Robot.bot.intakeConstants.pivotConstants().kPIDF().kP(),
                    Robot.bot.intakeConstants.pivotConstants().kPIDF().kI(),
                    Robot.bot.intakeConstants.pivotConstants().kPIDF().kD(),
                    ClosedLoopSlot.kSlot0);

        config.inverted(Robot.bot.intakeConstants.pivotConstants().pivotInverted());

        config.smartCurrentLimit(STATOR_CURRENT_LIMIT);

        motor.clearFaults();

        // Apply configuration
        SparkUtil.configure(motor, config, true);
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.data =
            new PivotIOData(
                connectedDebouncer.calculate(motor.getLastError() == REVLibError.kOk),
                encoder.getPosition(),
                encoder.getVelocity(),
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
}