package org.frogforce503.robot2025.subsystems.leds;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix.led.Animation;

import edu.wpi.first.wpilibj.util.Color;

public interface LedsIO {
    @AutoLog
    class LedsIOInputs {
        public LedsIOData data = new LedsIOData(false, 0.0, 0.0, 0.0);
    }

    record LedsIOData(
        boolean stripConnected,
        double supplyVolts,
        double statorCurrentAmps,
        double tempCelsius) {}

    default void updateInputs(LedsIOInputs inputs) {}

    default void runColor(Color color) {}

    default void runAnimation(Animation animation) {}

    default void stop() {}
}