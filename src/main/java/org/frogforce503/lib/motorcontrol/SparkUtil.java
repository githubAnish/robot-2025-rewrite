package org.frogforce503.lib.motorcontrol;

import org.frogforce503.lib.util.ErrorUtil;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SignalsConfig;
import com.revrobotics.spark.config.SparkBaseConfig;

/** Helper class for Spark IO implementations */
public final class SparkUtil {
    public static final int kDefaultCurrentLimit = 80;
    public static final int kThroughBoreEncoderCounts = 8192;

    private SparkUtil() {}

    public static ClosedLoopSlot getClosedLoopSlot(int slot) {
        assert (0 <= slot && slot <= 3) : "Invalid slot ID: " + slot + ErrorUtil.attachJavaClassName(SparkUtil.class);
        return ClosedLoopSlot.values()[slot];
    }

    public static <S extends SparkBase, C extends SparkBaseConfig> void configure(S motor, C config, boolean burnFlash) {
        motor.configure(
            config,
            burnFlash ? ResetMode.kResetSafeParameters : ResetMode.kNoResetSafeParameters,
            burnFlash ? PersistMode.kPersistParameters : PersistMode.kNoPersistParameters);
    }

    /** Optimizes motor signals to limit unnecessary data over CAN. */
    public static SignalsConfig signalsOptimized() {
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