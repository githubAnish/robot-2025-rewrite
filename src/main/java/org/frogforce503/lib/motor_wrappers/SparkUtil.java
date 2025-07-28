package org.frogforce503.lib.motor_wrappers;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SignalsConfig;
import com.revrobotics.spark.config.SparkBaseConfig;

/** Helper class for Spark IO implementations */
public final class SparkUtil {
    private static final int THROUGH_BORE_ENCODER_COUNTS = 8192;

    private SparkUtil() {}

    @SuppressWarnings("unchecked")
    public static <T extends SparkBase> T getSpark(int deviceId, boolean isFlex) {
        return
            isFlex
                ? (T) new SparkFlex(deviceId, MotorType.kBrushless)
                : (T) new SparkMax(deviceId, MotorType.kBrushless);
    }

    public static ClosedLoopSlot getClosedLoopSlot(int slot) {
        return switch (slot) {
            case 0 -> ClosedLoopSlot.kSlot0;
            case 1 -> ClosedLoopSlot.kSlot1;
            case 2 -> ClosedLoopSlot.kSlot2;
            case 3 -> ClosedLoopSlot.kSlot3;
            default -> throw new IllegalArgumentException("Invalid slot ID: " + slot);
        };
    }

    public static void configure(SparkBase motor, SparkBaseConfig config, boolean burnFlash) {
        motor.configure(
            config,
            burnFlash ? ResetMode.kResetSafeParameters : ResetMode.kNoResetSafeParameters,
            burnFlash ? PersistMode.kPersistParameters : PersistMode.kNoPersistParameters);
    }

    /** Optimizes motor signals to limit unnecessary data over CAN. */
    public static SignalsConfig optimizeSignals(SparkBaseConfig config) {
        return
            new SignalsConfig()
                .analogPositionAlwaysOn(false)
                .analogVelocityAlwaysOn(false)
                .analogVoltageAlwaysOn(false)
                .faultsAlwaysOn(true)
                .faultsPeriodMs(250)
                .motorTemperaturePeriodMs(250)
                .warningsAlwaysOn(true)
                .warningsPeriodMs(250);
    }
}