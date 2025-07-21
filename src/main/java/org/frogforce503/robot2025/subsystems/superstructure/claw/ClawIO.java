package org.frogforce503.robot2025.subsystems.superstructure.claw;

import org.littletonrobotics.junction.AutoLog;

public interface ClawIO {
    @AutoLog
    class ClawIOInputs {
        public ClawIOData leftMotorData = new ClawIOData(false, 0.0, 0.0, 0.0, 0.0, 0.0);
        public ClawIOData rightMotorData = new ClawIOData(false, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    record ClawIOData(
        boolean motorConnected,
        double position,
        double velocity,
        double appliedVolts,
        double statorCurrentAmps,
        double tempCelsius) {}

    default void updateInputs(ClawIOInputs inputs) {}

    default void runOpenLoop(double outputLeft, double outputRight) {}

    default void runVolts(double voltsLeft, double voltsRight) {}

    default void runVelocity(double velocityLeft, double velocityRight) {}

    default void stop() {}

    default void setBrakeMode(boolean enabled) {}
}
