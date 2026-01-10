package org.frogforce503.robot.subsystems.superstructure.claw;

import org.littletonrobotics.junction.AutoLog;

public interface ClawIO {
    @AutoLog
    class ClawIOInputs {
        public ClawIOData leftData = new ClawIOData(false, 0.0, 0.0, 0.0, 0.0);
        public ClawIOData rightData = new ClawIOData(false, 0.0, 0.0, 0.0, 0.0);
    }

    record ClawIOData(
        boolean motorConnected,
        double velocityRadPerSec,
        double appliedVolts,
        double statorCurrentAmps,
        double tempCelsius) {}

    default void updateInputs(ClawIOInputs inputs) {}

    default void runOpenLoop(double leftOutput, double rightOutput) {}

    default void runVolts(double leftVolts, double rightVolts) {}

    default void runVelocity(double leftVelocityRadPerSec, double rightVelocityRadPerSec, double leftFeedforward, double rightFeedforward) {}

    default void stop() {}

    default void setPID(double kP, double kI, double kD) {}

    default void setBrakeMode(boolean enabled) {}
}
