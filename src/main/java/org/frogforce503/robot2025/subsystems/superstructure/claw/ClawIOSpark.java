package org.frogforce503.robot2025.subsystems.superstructure.claw;

import org.frogforce503.lib.motorcontrol.SparkUtil;
import org.frogforce503.robot2025.Robot;
import org.frogforce503.robot2025.config.subsystem.ClawConfig;

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
import lombok.Getter;

public class ClawIOSpark implements ClawIO {
    // Hardware
    @Getter private final SparkMax leftMotor;
    @Getter private final SparkMax rightMotor;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    // Control
    private final SparkClosedLoopController leftController;
    private final SparkClosedLoopController rightController;

    // Config
    private SparkMaxConfig leftConfig = new SparkMaxConfig();
    private SparkMaxConfig rightConfig = new SparkMaxConfig();

    // Connected Debouncers
    private final Debouncer connectedDebouncer = new Debouncer(.5);
    
    public ClawIOSpark() {
        final ClawConfig clawConfig = Robot.bot.getClawConfig();

        leftMotor = new SparkMax(clawConfig.leftId(), MotorType.kBrushless);
        leftEncoder = leftMotor.getEncoder();

        rightMotor = new SparkMax(clawConfig.rightId(), MotorType.kBrushless);
        rightEncoder = rightMotor.getEncoder();

        leftController = leftMotor.getClosedLoopController();
        rightController = rightMotor.getClosedLoopController();

        // Configure motor
        leftConfig.inverted(clawConfig.leftInverted());
        leftConfig.idleMode(IdleMode.kBrake);
        leftConfig.smartCurrentLimit(clawConfig.statorCurrentLimit());
        leftConfig.voltageCompensation(12.0);

        leftConfig
            .encoder
                .positionConversionFactor(1 / clawConfig.mechanismRatio())
                .velocityConversionFactor(1 / clawConfig.mechanismRatio())
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);

        leftConfig
            .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(clawConfig.kPID().kP(), clawConfig.kPID().kI(), clawConfig.kPID().kD());

        rightConfig
            .apply(leftConfig)
            .inverted(clawConfig.rightInverted());

        SparkUtil.optimizeSignals(leftConfig, false, false);
        SparkUtil.optimizeSignals(rightConfig, false, false);

        leftMotor.clearFaults();
        rightMotor.clearFaults();

        // Apply configuration
        SparkUtil.configure(leftMotor, leftConfig, true);
        SparkUtil.configure(rightMotor, rightConfig, true);
    }

    @Override
    public void updateInputs(ClawIOInputs inputs) {
        inputs.leftMotorData =
            new ClawIOData(
                connectedDebouncer.calculate(leftMotor.getLastError() == REVLibError.kOk),
                leftEncoder.getVelocity(),
                leftMotor.getAppliedOutput() * leftMotor.getBusVoltage(),
                leftMotor.getOutputCurrent(),
                leftMotor.getMotorTemperature());

        inputs.rightMotorData =
            new ClawIOData(
                connectedDebouncer.calculate(rightMotor.getLastError() == REVLibError.kOk),
                rightEncoder.getVelocity(),
                rightMotor.getAppliedOutput() * rightMotor.getBusVoltage(),
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
    public void runVelocity(double velocityLeft, double velocityRight, double feedforward) {
        leftController.setReference(velocityLeft, ControlType.kVelocity, ClosedLoopSlot.kSlot0, feedforward);
        rightController.setReference(velocityRight, ControlType.kVelocity, ClosedLoopSlot.kSlot0, feedforward);
    }

    @Override
    public void stop() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        leftConfig.closedLoop.pid(kP, kI, kD);
        rightConfig.closedLoop.pid(kP, kI, kD);

        SparkUtil.configure(leftMotor, leftConfig, false);
        SparkUtil.configure(rightMotor, rightConfig, false);
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        leftConfig.idleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
        rightConfig.idleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);

        SparkUtil.configure(leftMotor, leftConfig, false);
        SparkUtil.configure(rightMotor, rightConfig, false);
    }
}