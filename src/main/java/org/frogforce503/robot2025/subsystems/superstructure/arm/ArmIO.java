package org.frogforce503.robot2025.subsystems.superstructure.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    class ArmIOInputs {
        public ArmIOData data = new ArmIOData(false, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    record ArmIOData(
        boolean motorConnected,
        double position,
        double velocity,
        double appliedVolts,
        double statorCurrentAmps,
        double tempCelsius) {}

    default void updateInputs(ArmIOInputs inputs) {}

    default void runOpenLoop(double output) {}

    default void runVolts(double volts) {}

    default void runPosition(double position, double feedforward) {}

    default void stop() {}

    default void setPID(double kP, double kI, double kD) {}

    default void setBrakeMode(boolean enabled) {}
}