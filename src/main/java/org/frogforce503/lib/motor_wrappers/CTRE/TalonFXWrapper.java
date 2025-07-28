package org.frogforce503.lib.motor_wrappers.CTRE;

import org.frogforce503.lib.motor_wrappers.BaseMotorWrapper;
import org.frogforce503.lib.motor_wrappers.CANMotor.MotorControlMode;
import org.frogforce503.lib.motor_wrappers.CTRE.TalonSRXWrapper.CANProfile;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.MathUtil;

/**
 * This class is a thin wrapper around the CANTalon that reduces CAN bus / CPU
 * overhead by skipping duplicate set commands. (By default the Talon flushes
 * the Tx buffer on every set call).
 */
public class TalonFXWrapper extends TalonFX implements BaseMotorWrapper {
    protected double mLastSet = Double.NaN;
    protected ControlMode mLastControlMode = null;
    protected CANProfile mLastCANProfile = CANProfile.Default;

    TalonFXConfiguration config = new TalonFXConfiguration();

    public TalonFXWrapper(int deviceNumber) {
        super(deviceNumber);
        super.getConfigurator().apply(config);
    }

    public TalonFXWrapper(int deviceNumber, String canBus) {
        super(deviceNumber, canBus);
        super.getConfigurator().apply(config);
    }

    @Override
    public void set(MotorControlMode CANMotorMode, double value, double arbFF) {
        ControlMode mode = BaseMotorWrapper.ctreModes.get(CANMotorMode);

        if (value != mLastSet || mode != mLastControlMode) {
            mLastSet = value;
            mLastControlMode = mode;

            ControlRequest request;

            switch (mode) {
                case PercentOutput:
                    request = new DutyCycleOut(value);
                    break;
                case Velocity:
                    request = new VelocityTorqueCurrentFOC(value).withFeedForward(arbFF); // requires Phoenix Pro
                    break;
                case MotionMagic:
                    request = new MotionMagicExpoTorqueCurrentFOC(value).withFeedForward(arbFF); // requires Phoenix Pro
                    break;
                case Position:
                    request = new PositionTorqueCurrentFOC(value).withFeedForward(arbFF); // requires Phoenix Pro
                    break;
                default:
                    // makes sure if controlmode is anything other than listed above, runs Duty Cycle ranging from -1.0 to 1.0
                    request = new DutyCycleOut(MathUtil.clamp(-1.0, 1.0, value));
                    break;
            }

            super.setControl(request);
        }
    }

    /**
     * VERY IMPORTANT, MUST BE DONE AFTER MOTOR INITALIZATION
     */
    public void applyConfig() {
        super.getConfigurator().apply(config);
    }

    @Override
    public void applyMotorConfig(boolean shouldFactoryDefault, boolean shouldBurnConfig) {
        applyConfig();
    }

    @Override
    public void configureProfiled(int slotID, double maxVel, double maxAcc, double tolerance) {
        config.MotionMagic
            .withMotionMagicCruiseVelocity(maxVel)
            .withMotionMagicAcceleration(maxAcc);
    }

    @Override
    public void setConversionFactor(double factor, ConversionFactorType type) {
        config.Feedback
            .withSensorToMechanismRatio(factor);
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
                config.Slot0.kP = kP;
                config.Slot0.kI = kI;
                config.Slot0.kD = kD;
                config.Slot0.kV = kFF; // takes place of default feedforward
                break;
            case 1:
                config.Slot1.kP = kP;
                config.Slot1.kI = kI;
                config.Slot1.kD = kD;
                config.Slot1.kV = kFF; // takes place of default feedforward
                break;
            case 2:
                config.Slot2.kP = kP;
                config.Slot2.kI = kI;
                config.Slot2.kD = kD;
                config.Slot2.kV = kFF; // takes place of default feedforward
                break;
        }

        super.getConfigurator().apply(config);
    }

    @Override
    public double getP(int slot) {
        return ((SlotConfigs) intToFXSlot(slot)).kP;
    }

    @Override
    public double getI(int slot) {
        return ((SlotConfigs) intToFXSlot(slot)).kI;
    }

    @Override
    public double getD(int slot) {
        return ((SlotConfigs) intToFXSlot(slot)).kD;
    }

    @Override
    public double getF(int slot) {
        return ((SlotConfigs) intToFXSlot(slot)).kS;
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
        this.config.MotorOutput.Inverted = inverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
    }

    @Override
    public void setEncoderPosition(double position) {
        this.setPosition(position);
    }

    @Override
    public double getMotorPercent() {
        return this.getDutyCycle().getValueAsDouble() / 2.0; // since getDutyCycle() ranges from -2.0 to 2.0 apparently
    }

    @Override
    public double getMotorPosition() {
        return this.getPosition().getValueAsDouble();
    }

    @Override
    public double getMotorVelocity() {
        return this.getVelocity().getValueAsDouble();
    }

    @Override
    public double getOutputCurrent() {
        return this.getTorqueCurrent().getValueAsDouble();
    }

    @Override
    public double getTemperature() {
        return this.getDeviceTemp().getValueAsDouble();
    }

    @SuppressWarnings("unchecked")
    public <T> T intToFXSlot(int slot) {
        switch (slot) {
            case 0:
                return (T) config.Slot0;
            case 1:
                return (T) config.Slot1;
            case 2:
                return (T) config.Slot2;
            default:
                return (T) config.Slot0; // default return to Slot0
        }
    }

    // public void setCANProfile(CANProfile profile) {
    //     if (profile != mLastCANProfile) {
    //         mLastCANProfile = profile;
    //         switch (profile) {
    //             case Low: // Low Profiles for minimal CAN utilization
    //                 super.setStatusFramePeriod(StatusFrame.Status_1_General, 20);
    //                 super.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 200);
    //                 super.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 200);
    //                 break;
    //             case Idle: // Idle Profile for CAN utilization(Call when leaving a motor in idle but may
    //                        // call again soon)
    //                 super.setStatusFramePeriod(StatusFrame.Status_1_General, 20);
    //                 super.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 100);
    //                 super.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 100);
    //                 break;
    //             case Default: // Default Update Rates
    //                 super.setStatusFramePeriod(StatusFrame.Status_1_General, 10);
    //                 super.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
    //                 super.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 100);
    //                 break;
    //         }
    //     }
    // }

    // public enum CANProfile {
    //     Low, Idle, Default
    // }
}