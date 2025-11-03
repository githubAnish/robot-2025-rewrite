package org.frogforce503.robot2025.subsystems.superstructure.elevator;

import org.frogforce503.lib.motorcontrol.SparkUtil;
import org.frogforce503.lib.motorcontrol.tuning.pidf.PIDFConfig;
import org.frogforce503.robot2025.Robot;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;

public class ElevatorIOSpark implements ElevatorIO {
    // Hardware
    private SparkMax motor;
    private RelativeEncoder encoder;

    // Control
    private SparkClosedLoopController pidController;

    // Config
    private PIDFConfig currentPidConfig = Robot.bot.getElevatorConfig().kPIDF(); // Buffer variable to avoid calling configAccessor
    private IdleMode currentIdleMode = IdleMode.kBrake; // Buffer variable to avoid calling configAccessor

    private SparkMaxConfig config = new SparkMaxConfig();
    private final int STATOR_CURRENT_LIMIT = 80;

    // Connected Debouncers
    private final Debouncer connectedDebouncer = new Debouncer(.5);

    public ElevatorIOSpark() {
        motor = new SparkMax(Robot.bot.getElevatorConfig().elevatorID(), MotorType.kBrushless);
        encoder = motor.getEncoder();

        pidController = motor.getClosedLoopController();

        // Configure motor
        config
            .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(
                    currentPidConfig.kP(),
                    currentPidConfig.kI(),
                    currentPidConfig.kD(),
                    ClosedLoopSlot.kSlot0);

        config.inverted(Robot.bot.getElevatorConfig().elevatorInverted());
        config.smartCurrentLimit(STATOR_CURRENT_LIMIT);
        config.voltageCompensation(12);
        config.idleMode(currentIdleMode);

        motor.clearFaults();

        resetEncoder();

        // Apply configuration
        SparkUtil.configure(motor, config, true);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.data =
            new ElevatorIOData(
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
    public void resetEncoder() {
        encoder.setPosition(0.0);
    }
}