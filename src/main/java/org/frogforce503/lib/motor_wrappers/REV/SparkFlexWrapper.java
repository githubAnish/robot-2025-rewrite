package org.frogforce503.lib.motor_wrappers.REV;

import org.frogforce503.lib.motor_wrappers.BaseMotorWrapper;
import org.frogforce503.lib.motor_wrappers.CANMotor.MotorControlMode;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.ClosedLoopConfigAccessor;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.ExternalEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

/**
 * This class is a thin wrapper around the CANSparkFlex that reduces CAN bus / CPU
 * overhead by skipping duplicate set commands. (By default the Spark flushes
 * the Tx buffer on every set call). Additionally, Low CAN utilization profiles
 * can be selected for when a device is idle.
 * 
 * @implNote Remember that all the methods for changing the config will ONLY be applied when the applyConfig() or its caller methods {@code persistParameters()} or {@code persistNoParameters()} is called.
 */
public class SparkFlexWrapper extends SparkFlex implements BaseMotorWrapper {
    protected double mLastSet = Double.NaN;
    protected double mLastFF = -10000;

    private boolean hasExternalEncoder = false;

    protected ControlType mLastControlMode = ControlType.kDutyCycle;
    protected ClosedLoopSlot mSlotID = ClosedLoopSlot.kSlot0;

    protected SparkFlexConfig motorConfig;
    protected ClosedLoopConfig pidConfig;
    protected EncoderConfig encoderConfig;

    protected RelativeEncoder encoder;
    protected SparkClosedLoopController pidController;

    public SparkFlexWrapper(int deviceNumber, MotorType motorType, boolean hasExternalEncoder) {
        super(deviceNumber, motorType);

        motorConfig = new SparkFlexConfig();
        pidController = this.getClosedLoopController();
        pidConfig = motorConfig.closedLoop;
        encoderConfig = motorConfig.encoder;

        this.hasExternalEncoder = hasExternalEncoder;

        if (this.hasExternalEncoder) {
            encoder = this.getExternalEncoder();
            pidConfig.feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder);
            
            // Applying the ThroughBore Encoder Configuration
            motorConfig.apply(
                new ExternalEncoderConfig()
                    .countsPerRevolution(8192)
            );
            motorConfig.apply(pidConfig);
        } else {
            encoder = this.getEncoder();
            pidConfig.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        }

        pidConfig.outputRange(-1, 1, ClosedLoopSlot.kSlot0);
        pidConfig.outputRange(-1, 1, ClosedLoopSlot.kSlot1);
        pidConfig.outputRange(-1, 1, ClosedLoopSlot.kSlot2);
        pidConfig.outputRange(-1, 1, ClosedLoopSlot.kSlot3);

        this.clearFaults();
    }

    public SparkFlexWrapper(int deviceNumber, MotorType motorType) {
        this(deviceNumber, motorType, false);
    }

    public SparkFlexConfig getConfig() {
        return this.motorConfig;
    }
    
    public void applyConfig(ResetMode resetMode, PersistMode persistMode) {
        motorConfig
            .apply(pidConfig)
            .apply(encoderConfig);

        this.configure(motorConfig, resetMode, persistMode);
    }

    /**
     * @param shouldFactoryDefault If resetting parameters every time at controller restart.
     * @param shouldBurnConfig If the configuration should be burned to flash and retained after controller restart.
     * 
     * @apiNote
     * <b>QUICK TIPS: If...</b>
     * <ul>
     *  <li>{@code shouldFactoryDefault} = true & {@code shouldBurnConfig} = true, then this should be called during initialization, similar to burnFlash from pre-2025 REVLib.</li>
     *  <li>{@code shouldFactoryDefault} = false & {@code shouldBurnConfig} = true, then this should be called when making updates to the configuration while running the motor.</li>
     *  <li>{@code shouldFactoryDefault} = false & {@code shouldBurnConfig} = false, then this should be used when making updates to the configuration while running the motor in <b>specific instances</b>.
     *  <ul>
     *   <li><b>Example:</b> When setting coast vs brake mode.</li>
     *  </ul>
     * </ul>
     */
    @Override
    public void applyMotorConfig(boolean shouldFactoryDefault, boolean shouldBurnConfig) {
        applyConfig(
            shouldFactoryDefault ? ResetMode.kResetSafeParameters : ResetMode.kNoResetSafeParameters,
            shouldBurnConfig ? PersistMode.kPersistParameters : PersistMode.kPersistParameters
        );
    }

    @Override
    public void set(MotorControlMode CANMotorMode, double value, double arbFF) {
        ControlType mode = BaseMotorWrapper.revModes.get(CANMotorMode);

        if (value != mLastSet || mode != mLastControlMode || arbFF != mLastFF) {
            mLastSet = value;
            mLastControlMode = mode;
            mLastFF = arbFF;

            switch (mode) {
                case kDutyCycle:
                    super.set(value);
                    break;
                case kVoltage:
                    pidController.setReference(value, ControlType.kVoltage, mSlotID);
                    break;
                case kPosition:
                    pidController.setReference(value, ControlType.kPosition, mSlotID, arbFF);
                    break;
                case kVelocity:
                    pidController.setReference(value, ControlType.kVelocity, mSlotID, arbFF);
                    break;
                case kMAXMotionPositionControl:
                    pidController.setReference(value, ControlType.kMAXMotionPositionControl, mSlotID);
                    break;
                case kMAXMotionVelocityControl:
                    pidController.setReference(value, ControlType.kMAXMotionVelocityControl, mSlotID);
                    break;
                default:
                    break;
            }
        }
    }

    public void set(MotorControlMode mode, double value) {
        set(mode, value, 0.0);
    }

    public ControlType getControlMode() {
        return mLastControlMode;
    }

    @Override
    public int getSelectedProfileSlot() {
        return mSlotID.value;
    }

    @Override
    public void selectProfileSlot(int slotID) {
        mSlotID = intToRevSlot(slotID);
    }

    public ClosedLoopConfigAccessor getPIDController() {
        return this.configAccessor.closedLoop;
    }

    @Override
    public double getP(int slot) {
        return getPIDController().getP(intToRevSlot(slot));
    }

    @Override
    public double getI(int slot) {
        return getPIDController().getI(intToRevSlot(slot));
    }

    @Override
    public double getD(int slot) {
        return getPIDController().getD(intToRevSlot(slot));
    }

    @Override
    public double getF(int slot) {
        return getPIDController().getFF(intToRevSlot(slot));
    }

    @Override
    public void setPIDF(int slotID, double kP, double kI, double kD, double kFF) {
        pidConfig.pidf(kP, kI, kD, kFF, intToRevSlot(slotID));
    }

    public void setIzone(ClosedLoopSlot slotID, double kIz) {
        pidConfig.iZone(kIz, slotID);
    }

    public void configureMaxMotion(ClosedLoopSlot slotID, double maxVel, double maxAcc, double allowedError) {
        pidConfig.maxMotion
            .maxAcceleration(maxAcc, slotID)
            .maxVelocity(maxVel, slotID)
            .allowedClosedLoopError(maxAcc, slotID);
    }

    @Override
    public void configureProfiled(int slot, double maxVel, double maxAcc, double allowedError) {
        configureMaxMotion(intToRevSlot(slot), maxVel, maxAcc, allowedError);
    }

    public void setIdleMode(IdleMode mode) {
        motorConfig.idleMode(mode);
    }

    @Override
    public void setIdleMode(boolean shouldCoast) {
        setIdleMode(shouldCoast ? IdleMode.kCoast : IdleMode.kBrake);
    }

    @Override
    public String getIdleMode() {
        return this.configAccessor.getIdleMode() + "";
    }

    @Override
    public void setMotorInverted(boolean invert) {
        if (this.hasExternalEncoder) {
            motorConfig.apply(
                new ExternalEncoderConfig()
                    .countsPerRevolution(8192)
                    .inverted(invert)
            );
        } else {
            motorConfig.inverted(invert);
        }
    }

    public void setCurrentLimit(int limit) {
        motorConfig.smartCurrentLimit(limit);
    }

    public void follow(SparkFlexWrapper mainMotor, boolean invert) {
        motorConfig.follow(mainMotor, invert);
    }

    public void follow(SparkFlexWrapper mainMotor) {
        follow(mainMotor, false);
    }

    public double getEncoderPosition() {
        return encoder.getPosition();
    }

    public double getEncoderVelocity() {
        return encoder.getVelocity();
    }

    @Override
    public void setEncoderPosition(double position) {
        encoder.setPosition(position);
    }

    public void resetEncoder() {
        setEncoderPosition(0.0);
    }

    public void setPositionConversionFactor(double factor) {
        if (this.hasExternalEncoder) {
            motorConfig.apply(
                new ExternalEncoderConfig()
                    .countsPerRevolution(8192)
                    .positionConversionFactor(factor)
            );
        } else {
            encoderConfig.positionConversionFactor(factor);
        }
    }

    public void setVelocityConversionFactor(double factor) {
        if (this.hasExternalEncoder) {
            motorConfig.apply(
                new ExternalEncoderConfig()
                    .countsPerRevolution(8192)
                    .velocityConversionFactor(factor)
            );
        } else {
            encoderConfig.velocityConversionFactor(factor);
        }
    }

    @Override
    public void setConversionFactor(double factor, ConversionFactorType type) {
        if (type == ConversionFactorType.POSITION) {
            setPositionConversionFactor(factor);
        } else if (type == ConversionFactorType.VELOCITY) {
            setVelocityConversionFactor(factor);
        }
    }

    public double getClosedLoopError() {
        return mLastSet - ((this.mLastControlMode == ControlType.kVelocity) ? getEncoderVelocity() : getEncoderPosition());
    }

    @Override
    public double getMotorPercent() {
        return this.getAppliedOutput();
    }

    @Override
    public double getMotorPosition() {
        return getEncoderPosition();
    }

    @Override
    public double getMotorVelocity() {
        return getEncoderVelocity();
    }

    @Override
    public double getTemperature() {
        return this.getMotorTemperature();
    }

    // Utility methods
    private ClosedLoopSlot intToRevSlot(int slot) {
        switch (slot) {
            case 0:
                return ClosedLoopSlot.kSlot0;
            case 1:
                return ClosedLoopSlot.kSlot1;
            case 2:
                return ClosedLoopSlot.kSlot2;
            case 3:
                return ClosedLoopSlot.kSlot3;
        }

        throw new RuntimeException("Error returned from SparkFlexWrapper intToRevSlot() --- slot ID invalid");
    }



    // -------------------------- CAN Utilization Profiles (Doesn't work, but has potential to minimize CAN Bus usage) -------------------------- //

    // protected CANProfile mLastCANProfile = CANProfile.Default;

    // public void setCANProfile(CANProfile profile) {
    //     if (profile != mLastCANProfile) {
    //         mLastCANProfile = profile;
    //         switch (profile) {
    //             case Low: // Low Profiles for minimal CAN utilization
    //                 super.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 40);
    //                 super.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 250);
    //                 super.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 250);
    //                 super.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 250);
    //                 break;
    //             case Idle: // Idle Profile for CAN utilization(Call when leaving a motor in idle but may
    //                        // call again soon)
    //                 super.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    //                 super.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 100);
    //                 super.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 100);
    //                 super.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 250);
    //                 break;
    //             case Default: // Default Update Rates
    //                 super.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
    //                 super.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    //                 super.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    //                 super.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 50);
    //                 break;
    //         }
    //     }
    // }

    // public enum CANProfile {
    //     Low, Idle, Default
    // }
}