package org.frogforce503.lib.motor_wrappers;

import java.util.HashMap;

import org.frogforce503.lib.motor_wrappers.CANMotor.MotorControlMode;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.spark.SparkBase.ControlType;

public interface BaseMotorWrapper {
    void applyMotorConfig(boolean shouldFactoryDefault, boolean shouldBurnConfig);

    // PID Configuration
    void selectProfileSlot(int slotID);
    int getSelectedProfileSlot();
    void setPIDF(int slot, double p, double i, double d, double f);
    double getP(int slot);
    double getI(int slot);
    double getD(int slot);
    double getF(int slot);

    // Request setpoint to a motor controller
    void set(MotorControlMode mode, double value, double arbFF);

    // Other configuration changes
    void setMotorInverted(boolean inverted);
    void setIdleMode(boolean shouldCoast);
    void setEncoderPosition(double position);
    void configureProfiled(int slotID, double maxVel, double maxAcc, double tolerance);
    void setConversionFactor(double convFactor, ConversionFactorType type);

    // Getters
    String getIdleMode();

    double getMotorPercent();
    double getMotorPosition();
    double getMotorVelocity();

    double getOutputCurrent();
    double getTemperature();

    /** Type of conversion to use, either {@code POSITION} or {@code VELOCITY} */
    public enum ConversionFactorType {
        POSITION,
        VELOCITY
    }

    /** Mapping of {@link CANMotor}'s {@link MotorControlMode} to CTRE's {@link ControlMode} */
    public static HashMap<MotorControlMode, ControlMode> ctreModes = new HashMap<MotorControlMode, ControlMode>() {{
        put(MotorControlMode.PercentOutput, ControlMode.PercentOutput);
        put(MotorControlMode.Position, ControlMode.Position);
        put(MotorControlMode.Velocity, ControlMode.Velocity);
        put(MotorControlMode.ProfiledPosition, ControlMode.MotionMagic);
    }};

    /** Mapping of {@link CANMotor}'s {@link MotorControlMode} to REV's {@link ControlType} */
    public static HashMap<MotorControlMode, ControlType> revModes = new HashMap<MotorControlMode, ControlType>() {{
        put(MotorControlMode.PercentOutput, ControlType.kDutyCycle);
        put(MotorControlMode.Position, ControlType.kPosition);
        put(MotorControlMode.Velocity, ControlType.kVelocity);
        put(MotorControlMode.Voltage, ControlType.kVoltage);
        put(MotorControlMode.ProfiledPosition, ControlType.kMAXMotionPositionControl);
    }};
}