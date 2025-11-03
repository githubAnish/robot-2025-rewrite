package org.frogforce503.robot2025.subsystems.superstructure.intake.roller;

import org.frogforce503.lib.motorcontrol.SparkUtil;
import org.frogforce503.lib.motorcontrol.tuning.pidf.PIDFConfig;
import org.frogforce503.robot2025.Robot;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;

public class RollerIOSpark implements RollerIO {
    // Hardware
    private SparkBase motor;
    private RelativeEncoder encoder;

    // Control
    private SparkClosedLoopController pidController;

    // Config
    private PIDFConfig currentPidConfig = Robot.bot.getIntakeConfig().rollerPIDF(); // Buffer variable to avoid calling configAccessor
    private IdleMode currentIdleMode = IdleMode.kBrake; // Buffer variable to avoid calling configAccessor

    private SparkMaxConfig config = new SparkMaxConfig();
    private final int STATOR_CURRENT_LIMIT = 80;

    // Connected Debouncers
    private final Debouncer connectedDebouncer = new Debouncer(.5);
    
    public RollerIOSpark() {
        motor =
            Robot.bot.getIntakeConfig().rollerIsSparkFlex()
                ? new SparkFlex(Robot.bot.getIntakeConfig().rollerID(), MotorType.kBrushless)
                : new SparkMax(Robot.bot.getIntakeConfig().rollerID(), MotorType.kBrushless);
        encoder = motor.getEncoder();

        pidController = motor.getClosedLoopController();

        // Configure motor
        config
            .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pidf(
                    currentPidConfig.kP(),
                    currentPidConfig.kI(),
                    currentPidConfig.kD(),
                    currentPidConfig.kV(),
                    ClosedLoopSlot.kSlot0);

        config.inverted(Robot.bot.getIntakeConfig().rollerInverted());
        config.smartCurrentLimit(STATOR_CURRENT_LIMIT);
        config.voltageCompensation(12);
        config.idleMode(currentIdleMode);

        motor.clearFaults();

        encoder.setPosition(0.0);

        // Apply config
        SparkUtil.configure(motor, config, true);
    }

    @Override
    public void updateInputs(RollerIOInputs inputs) {
        inputs.data =
            new RollerIOData(
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
    public void runVelocity(double velocity) {
        pidController.setReference(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
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
}