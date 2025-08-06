package org.frogforce503.robot2025.subsystems.climber;

import org.frogforce503.lib.motorcontrol.SparkUtil;
import org.frogforce503.robot2025.Robot;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;

public class ClimberIOSpark implements ClimberIO {
    // Hardware
    private SparkMax motor;
    private RelativeEncoder encoder;

    // Control
    private SparkClosedLoopController pidController;

    // Config
    private SparkMaxConfig config = new SparkMaxConfig();
    private final int STATOR_CURRENT_LIMIT = 80;

    // Connected Debouncers
    private final Debouncer connectedDebouncer = new Debouncer(.5);
    
    public ClimberIOSpark() {
        motor = (SparkMax) SparkUtil.getSpark(Robot.bot.climberConstants.winchID(), false);
        encoder = motor.getEncoder();

        pidController = motor.getClosedLoopController();

        // Configure motor
        config
            .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(
                    Robot.bot.climberConstants.kPIDF().kP(),
                    Robot.bot.climberConstants.kPIDF().kI(),
                    Robot.bot.climberConstants.kPIDF().kD(),
                    ClosedLoopSlot.kSlot0);

        config.inverted(Robot.bot.climberConstants.winchInverted());

        config.smartCurrentLimit(STATOR_CURRENT_LIMIT);

        motor.clearFaults();

        encoder.setPosition(0.0);

        // Apply configuration
        SparkUtil.configure(motor, config, true);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.data =
            new ClimberIOData(
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
    public void runTorqueCurrent(double current) {
        pidController.setReference(current, ControlType.kCurrent, ClosedLoopSlot.kSlot0);
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