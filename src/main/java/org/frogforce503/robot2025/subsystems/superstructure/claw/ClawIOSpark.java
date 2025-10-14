package org.frogforce503.robot2025.subsystems.superstructure.claw;

import org.frogforce503.lib.motorcontrol.SparkUtil;
import org.frogforce503.robot2025.Constants;

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

public class ClawIOSpark implements ClawIO {
    // Hardware
    private SparkMax leftMotor;
    private SparkMax rightMotor;
    
    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;

    // Control
    private SparkClosedLoopController leftPidController;
    private SparkClosedLoopController rightPidController;

    // Config
    private SparkMaxConfig leftConfig = new SparkMaxConfig();
    private SparkMaxConfig rightConfig = new SparkMaxConfig();
    private final int STATOR_CURRENT_LIMIT = 35;

    // Connected Debouncers
    private final Debouncer connectedDebouncer = new Debouncer(.5);
    
    public ClawIOSpark() {
        leftMotor = new SparkMax(Constants.bot.Claw.leftMotorConfig().motorID(), MotorType.kBrushless);
        leftEncoder = leftMotor.getEncoder();

        rightMotor = new SparkMax(Constants.bot.Claw.rightMotorConfig().motorID(), MotorType.kBrushless);
        rightEncoder = rightMotor.getEncoder();

        leftPidController = leftMotor.getClosedLoopController();
        rightPidController = rightMotor.getClosedLoopController();

        // Configure motor
        leftConfig
            .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pidf(
                    Constants.bot.Claw.leftMotorConfig().kPIDF().kP(),
                    Constants.bot.Claw.leftMotorConfig().kPIDF().kI(),
                    Constants.bot.Claw.leftMotorConfig().kPIDF().kD(),
                    Constants.bot.Claw.leftMotorConfig().kPIDF().kV(),
                    ClosedLoopSlot.kSlot0);

        leftConfig.inverted(Constants.bot.Claw.leftMotorConfig().motorInverted());

        leftConfig.smartCurrentLimit(STATOR_CURRENT_LIMIT);

        leftConfig.voltageCompensation(12);

        rightConfig
            .apply(leftConfig)
            .inverted(Constants.bot.Claw.rightMotorConfig().motorInverted());

        rightConfig
            .closedLoop
            .pidf(
                Constants.bot.Claw.rightMotorConfig().kPIDF().kP(),
                Constants.bot.Claw.rightMotorConfig().kPIDF().kI(),
                Constants.bot.Claw.rightMotorConfig().kPIDF().kD(),
                Constants.bot.Claw.rightMotorConfig().kPIDF().kV(),
                ClosedLoopSlot.kSlot0);

        leftMotor.clearFaults();
        rightMotor.clearFaults();

        leftEncoder.setPosition(0.0);
        rightEncoder.setPosition(0.0);

        // Apply configuration
        SparkUtil.configure(leftMotor, leftConfig, true);
        SparkUtil.configure(rightMotor, rightConfig, true);
    }

    @Override
    public void updateInputs(ClawIOInputs inputs) {
        inputs.leftMotorData =
            new ClawIOData(
                connectedDebouncer.calculate(leftMotor.getLastError() == REVLibError.kOk),
                leftEncoder.getPosition(),
                leftEncoder.getVelocity(),
                leftMotor.getBusVoltage() * leftMotor.getAppliedOutput(),
                leftMotor.getOutputCurrent(),
                leftMotor.getMotorTemperature());

        inputs.rightMotorData =
            new ClawIOData(
                connectedDebouncer.calculate(rightMotor.getLastError() == REVLibError.kOk),
                rightEncoder.getPosition(),
                rightEncoder.getVelocity(),
                rightMotor.getBusVoltage() * rightMotor.getAppliedOutput(),
                rightMotor.getOutputCurrent(),
                rightMotor.getMotorTemperature());
    }

    @Override
    public void runOpenLoop(double outputLeft, double outputRight) {
        leftMotor.set(outputLeft);
        rightMotor.set(outputRight);
    }

    @Override
    public void runVolts(double voltsLeft, double voltsRight) {
        leftMotor.setVoltage(voltsLeft);
        rightMotor.setVoltage(voltsRight);
    }

    @Override
    public void runVelocity(double velocityLeft, double velocityRight) {
        leftPidController.setReference(velocityLeft, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        rightPidController.setReference(velocityRight, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    @Override
    public void stop() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        leftConfig.idleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
        rightConfig.idleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);

        SparkUtil.configure(leftMotor, leftConfig, false);
        SparkUtil.configure(rightMotor, rightConfig, false);
    }
}