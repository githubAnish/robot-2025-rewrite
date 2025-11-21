package org.frogforce503.robot2025.subsystems.superstructure.arm;

import org.frogforce503.lib.motorcontrol.SparkUtil;
import org.frogforce503.robot2025.Robot;

import com.revrobotics.REVLibError;
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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class ArmIOSpark implements ArmIO {
    // Hardware
    private SparkMax motor;
    private SparkAbsoluteEncoder encoder;

    // Control
    private SparkClosedLoopController pidController;

    // Config
    private SparkMaxConfig config = new SparkMaxConfig();
    private final int STATOR_CURRENT_LIMIT = 30;

    // Connected Debouncers
    private final Debouncer connectedDebouncer = new Debouncer(.5);

    public ArmIOSpark() {
        motor = new SparkMax(Robot.bot.getArmConfig().armID(), MotorType.kBrushless);
        encoder = motor.getAbsoluteEncoder();

        pidController = motor.getClosedLoopController();

        // Configure motor
        config
            .absoluteEncoder
                .zeroOffset(Robot.bot.getArmConfig().armOffset())
                .positionConversionFactor(2 * Math.PI) // convert rotations to radians
                .velocityConversionFactor(2 * Math.PI / 60) // convert RPM to rad/sec
                .setSparkMaxDataPortConfig();

        config
            .closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(
                    Robot.bot.getArmConfig().kPID().kP(),
                    Robot.bot.getArmConfig().kPID().kI(),
                    Robot.bot.getArmConfig().kPID().kD(),
                    ClosedLoopSlot.kSlot0)
                .iZone(
                    Robot.bot.getArmConfig().kPID().kIZone(),
                    ClosedLoopSlot.kSlot0);

        config.inverted(Robot.bot.getArmConfig().armInverted());
        config.smartCurrentLimit(STATOR_CURRENT_LIMIT);
        config.voltageCompensation(12);
        config.idleMode(IdleMode.kBrake);

        

        motor.clearFaults();

        // Apply configuration
        SparkUtil.configure(motor, config, true);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.data =
            new ArmIOData(
                connectedDebouncer.calculate(motor.getLastError() == REVLibError.kOk),
                Rotation2d.fromRotations(encoder.getPosition()),
                Units.rotationsToRadians(encoder.getVelocity()),
                motor.getAppliedOutput() * motor.getBusVoltage(),
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
    public void runPosition(Rotation2d position, double feedforward) {
        pidController.setReference(position.getRotations(), ControlType.kPosition, ClosedLoopSlot.kSlot0, feedforward);
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