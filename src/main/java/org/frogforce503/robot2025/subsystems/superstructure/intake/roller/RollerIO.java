package org.frogforce503.robot2025.subsystems.superstructure.intake.roller;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {
    @AutoLog
    class RollerIOInputs {
        public RollerIOData data = new RollerIOData(false, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    record RollerIOData(
        boolean motorConnected,
        double position,
        double velocity,
        double appliedVolts,
        double statorCurrentAmps,
        double tempCelsius) {}

    default void updateInputs(RollerIOInputs inputs) {}

    default void runOpenLoop(double output) {}

    default void runVolts(double volts) {}

    default void runVelocity(double velocity) {}

    default void stop() {}

    default void setBrakeMode(boolean enabled) {}
}
