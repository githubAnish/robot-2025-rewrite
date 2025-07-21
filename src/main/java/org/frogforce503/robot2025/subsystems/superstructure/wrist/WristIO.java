package org.frogforce503.robot2025.subsystems.superstructure.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
    @AutoLog
    class WristIOInputs {
        public WristIOData data = new WristIOData(false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    record WristIOData(
        boolean motorConnected,
        double relativePosition,
        double absolutePosition,
        double velocity,
        double appliedVolts,
        double statorCurrentAmps,
        double tempCelsius) {}

    default void updateInputs(WristIOInputs inputs) {}

    default void runOpenLoop(double output) {}

    default void runVolts(double volts) {}

    default void runPosition(double position, double feedforward) {}

    default void stop() {}

    default void setPID(double kP, double kI, double kD) {}

    default void setBrakeMode(boolean enabled) {}

    default void setEncoderPosition(double position) {}
}