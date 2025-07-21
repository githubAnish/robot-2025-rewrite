package org.frogforce503.robot2025.subsystems.superstructure.intake.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
    @AutoLog
    class PivotIOInputs {
        public PivotIOData data = new PivotIOData(false, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    record PivotIOData(
        boolean motorConnected,
        double position,
        double velocity,
        double appliedVolts,
        double statorCurrentAmps,
        double tempCelsius) {}

    default void updateInputs(PivotIOInputs inputs) {}

    default void runOpenLoop(double output) {}

    default void runVolts(double volts) {}

    default void runPosition(double position, double feedforward) {}

    default void stop() {}

    default void setPID(double kP, double kI, double kD) {}

    default void setBrakeMode(boolean enabled) {}
}
