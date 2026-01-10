package org.frogforce503.robot.subsystems.superstructure.claw;

import org.frogforce503.lib.motorcontrol.SparkUtil;
import org.frogforce503.robot.Robot;
import org.frogforce503.robot.constants.hardware.subsystem_config.ClawConfig;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.FeedbackSensor;
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

    // Filters
    private final Debouncer connectedDebouncer = new Debouncer(.5);
    
    public ClawIOSpark() {
        final ClawConfig clawConfig = Robot.bot.getClawConfig();

        // Initialize motors
        leftMotor = new SparkMax(clawConfig.leftId(), MotorType.kBrushless);
        leftEncoder = leftMotor.getEncoder();
        leftController = leftMotor.getClosedLoopController();

        rightMotor = new SparkMax(clawConfig.rightId(), MotorType.kBrushless);
        rightEncoder = rightMotor.getEncoder();
        rightController = rightMotor.getClosedLoopController();

        // Configure motor
        leftConfig.inverted(clawConfig.leftInverted());
        leftConfig.idleMode(IdleMode.kBrake);
        leftConfig.smartCurrentLimit(clawConfig.statorCurrentLimit());
        leftConfig.voltageCompensation(12.0);

        leftConfig
            .encoder
                .positionConversionFactor((1 / clawConfig.mechanismRatio()) * (2 * Math.PI)) // convert rotations to radians
                .velocityConversionFactor((1 / clawConfig.mechanismRatio()) * (2 * Math.PI) / 60) // convert RPM to rad/sec
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
        inputs.leftData =
            new ClawIOData(
                connectedDebouncer.calculate(leftMotor.getLastError() == REVLibError.kOk),
                leftEncoder.getVelocity(),
                leftMotor.getAppliedOutput() * leftMotor.getBusVoltage(),
                leftMotor.getOutputCurrent(),
                leftMotor.getMotorTemperature());

        inputs.rightData =
            new ClawIOData(
                connectedDebouncer.calculate(rightMotor.getLastError() == REVLibError.kOk),
                rightEncoder.getVelocity(),
                rightMotor.getAppliedOutput() * rightMotor.getBusVoltage(),
                rightMotor.getOutputCurrent(),
                rightMotor.getMotorTemperature());
    }

    @Override
    public void runOpenLoop(double leftOutput, double rightOutput) {
        leftMotor.set(leftOutput);
        rightMotor.set(rightOutput);
    }

    @Override
    public void runVolts(double leftVolts, double rightVolts) {
        leftMotor.setVoltage(leftVolts);
        rightMotor.setVoltage(rightVolts);
    }

    @Override
    public void runVelocity(double leftVelocityRadPerSec, double rightVelocityRadPerSec, double leftFeedforward, double rightFeedforward) {
        leftController.setSetpoint(leftVelocityRadPerSec, ControlType.kVelocity, ClosedLoopSlot.kSlot0, leftFeedforward);
        rightController.setSetpoint(rightVelocityRadPerSec, ControlType.kVelocity, ClosedLoopSlot.kSlot0, rightFeedforward);
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