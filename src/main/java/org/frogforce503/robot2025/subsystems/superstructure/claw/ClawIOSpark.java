package org.frogforce503.robot2025.subsystems.superstructure.claw;

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
    private PIDFConfig currentPidConfig = Robot.bot.getClawConfig().kPIDF(); // Buffer variable to avoid calling configAccessor
    private IdleMode currentIdleMode = IdleMode.kBrake; // Buffer variable to avoid calling configAccessor
    
    private SparkMaxConfig leftConfig = new SparkMaxConfig();
    private SparkMaxConfig rightConfig = new SparkMaxConfig();
    private final int STATOR_CURRENT_LIMIT = 35;

    // Connected Debouncers
    private final Debouncer connectedDebouncer = new Debouncer(.5);
    
    public ClawIOSpark() {
        leftMotor = new SparkMax(Robot.bot.getClawConfig().leftMotorID(), MotorType.kBrushless);
        leftEncoder = leftMotor.getEncoder();

        rightMotor = new SparkMax(Robot.bot.getClawConfig().rightMotorID(), MotorType.kBrushless);
        rightEncoder = rightMotor.getEncoder();

        leftPidController = leftMotor.getClosedLoopController();
        rightPidController = rightMotor.getClosedLoopController();

        // Configure motor
        leftConfig
            .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pidf(
                    currentPidConfig.kP(),
                    currentPidConfig.kI(),
                    currentPidConfig.kD(),
                    currentPidConfig.kV(),
                    ClosedLoopSlot.kSlot0);

        leftConfig.inverted(Robot.bot.getClawConfig().leftMotorInverted());
        leftConfig.smartCurrentLimit(STATOR_CURRENT_LIMIT);
        leftConfig.voltageCompensation(12);
        leftConfig.idleMode(currentIdleMode);

        rightConfig
            .apply(leftConfig)
            .inverted(Robot.bot.getClawConfig().rightMotorInverted());

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
    public void setPID(double kP, double kI, double kD) {
        if (currentPidConfig.kP() != kP || currentPidConfig.kI() != kI || currentPidConfig.kD() != kD) {
            leftConfig.closedLoop.pid(kP, kI, kD, ClosedLoopSlot.kSlot0);
            rightConfig.closedLoop.pid(kP, kI, kD, ClosedLoopSlot.kSlot0);

            SparkUtil.configure(leftMotor, leftConfig, false);
            SparkUtil.configure(rightMotor, rightConfig, false);
            
            currentPidConfig = new PIDFConfig(kP, kI, kD);
        }
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        IdleMode request = enabled ? IdleMode.kBrake : IdleMode.kCoast;

        if (request != currentIdleMode) { // Doesn't set brake mode if it's already set
            leftConfig.idleMode(request);
            rightConfig.idleMode(request);

            SparkUtil.configure(leftMotor, leftConfig, false);
            SparkUtil.configure(rightMotor, rightConfig, false);
            
            currentIdleMode = request;
        }
    }
}