package org.frogforce503.robot2025.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    class ClimberIOInputs {
        public ClimberIOData data = new ClimberIOData(false, 0.0, 0.0, 0.0, false);
    }

    record ClimberIOData(
        boolean motorConnected,
        double appliedVolts,
        double statorCurrentAmps,
        double tempCelsius,
        boolean limitSwitchPressed) {}

    default void updateInputs(ClimberIOInputs inputs) {}

    default void runOpenLoop(double output) {}

    default void runVolts(double volts) {}

    default void stop() {}

    default void setBrakeMode(boolean enabled) {}
}
