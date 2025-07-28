package org.frogforce503.lib.motor_wrappers.CTRE;

import org.frogforce503.lib.motor_wrappers.BaseMotorWrapper;
import org.frogforce503.lib.motor_wrappers.CANMotor.MotorControlMode;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

/**
 * This class is a thin wrapper around the CANTalon that reduces CAN bus / CPU
 * overhead by skipping duplicate set commands. (By default the Talon flushes
 * the Tx buffer on every set call).
 */
public class TalonSRXWrapper extends TalonSRX implements BaseMotorWrapper {
    protected double mLastSet = Double.NaN;
    protected ControlMode mLastControlMode = null;
    protected CANProfile mLastCANProfile = CANProfile.Default;

    TalonSRXConfiguration config = new TalonSRXConfiguration();

    public TalonSRXWrapper(int deviceNumber) {
        super(deviceNumber);
        super.configFactoryDefault();
    }

    @Override
    public void set(ControlMode mode, double value) {
        if (value != mLastSet || mode != mLastControlMode) {
            mLastSet = value;
            mLastControlMode = mode;
            super.set(mode, value);
        }
    }

    @Override
    public void set(MotorControlMode CANMotorMode, double value, double arbFF) {
        ControlMode mode = BaseMotorWrapper.ctreModes.get(CANMotorMode);

        set(mode, value); // feedforward is not applied, have to fix
    }

    /**
     * VERY IMPORTANT, MUST BE DONE AFTER MOTOR INITALIZATION
     */
    public void applyConfig() {
        this.configAllSettings(config);
    }

    @Override
    public void applyMotorConfig(boolean shouldFactoryDefault, boolean shouldBurnConfig) {
        // if (persist) {
        //     applyConfig();
        // } else {
        //     applyConfig(); // Need to figure out how to persist & not persist parameters, similar to REV
        // }

        applyConfig();
    }

    @Override
    public void configureProfiled(int slotID, double maxVel, double maxAcc, double tolerance) {
        config.motionAcceleration = maxAcc;
        config.motionCruiseVelocity = maxVel;
        config.motionCurveStrength = 4; // random default value
    }

    @Override
    public void setConversionFactor(double factor, ConversionFactorType type) {
        // NO FUNCTIONALITY YET
    }

    @Override
    public int getSelectedProfileSlot() {
        // NO FUNCTIONALITY YET
        return 0;
    }

    @Override
    public void selectProfileSlot(int slotID) {
        // NO FUNCTIONALITY YET
    }

    @Override
    public void setPIDF(int slot, double kP, double kI, double kD, double kFF) {
        switch (slot) {
            case 0:
                config.slot0.kP = kP;
                config.slot0.kI = kI;
                config.slot0.kD = kD;
                config.slot0.kF = kFF; // takes place of default feedforward
                break;
            case 1:
                config.slot1.kP = kP;
                config.slot1.kI = kI;
                config.slot1.kD = kD;
                config.slot1.kF = kFF; // takes place of default feedforward
                break;
            case 2:
                config.slot2.kP = kP;
                config.slot2.kI = kI;
                config.slot2.kD = kD;
                config.slot2.kF = kFF; // takes place of default feedforward
                break;
        }

        this.configAllSettings(config);
    }

    @Override
    public double getP(int slot) {
        return intToSRXSlot(slot).kP;        
    }

    @Override
    public double getI(int slot) {
        return intToSRXSlot(slot).kI;
    }

    @Override
    public double getD(int slot) {
        return intToSRXSlot(slot).kD;
    }

    @Override
    public double getF(int slot) {
        return intToSRXSlot(slot).kF;
    }

    @Override
    public void setIdleMode(boolean shouldCoast) {
        // NO FUNCTIONALITY YET
    }

    @Override
    public String getIdleMode() {
        // NO FUNCTIONALITY YET
        return "";
    }

    @Override
    public void setMotorInverted(boolean inverted) {
        this.setInverted(inverted);
    }

    @Override
    public void setEncoderPosition(double position) {
        this.setSelectedSensorPosition(position);
    }

    @Override
    public double getMotorPercent() {
        return this.getMotorOutputPercent();
    }

    @Override
    public double getMotorPosition() {
        return this.getSelectedSensorPosition();
    }

    @Override
    public double getMotorVelocity() {
        return this.getSelectedSensorVelocity();
    }

    public SlotConfiguration intToSRXSlot(int slot) {
        switch (slot) {
            case 0:
                return config.slot0;
            case 1:
                return config.slot1;
            case 2:
                return config.slot2;
            default:
                return config.slot0; // default return to Slot0
        }
    }

    public enum CANProfile {
        Low, Idle, Default
    }

    public void setCANProfile(CANProfile profile) {
        if (profile != mLastCANProfile) {
            mLastCANProfile = profile;
            switch (profile) {
                case Low: // Low Profiles for minimal CAN utilization
                    super.setStatusFramePeriod(StatusFrame.Status_1_General, 20);
                    super.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 200);
                    super.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 200);
                    break;
                case Idle: // Idle Profile for CAN utilization(Call when leaving a motor in idle but may
                           // call again soon)
                    super.setStatusFramePeriod(StatusFrame.Status_1_General, 20);
                    super.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 100);
                    super.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 100);
                    break;
                case Default: // Default Update Rates
                    super.setStatusFramePeriod(StatusFrame.Status_1_General, 10);
                    super.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
                    super.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 100);
                    break;
            }
        }
    }
}