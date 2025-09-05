package org.frogforce503.lib.motorcontrol.test;

import org.littletonrobotics.junction.AutoLog;

public interface DummyElevatorIO {
    @AutoLog
    class DummyElevatorIOInputs {
        public DummyElevatorIOData data =
            new DummyElevatorIOData(false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    record DummyElevatorIOData(
        boolean motorConnected,
        double positionRad,
        double velocityRadPerSec,
        double appliedVolts,
        double torqueCurrentAmps,
        double supplyCurrentAmps,
        double tempCelsius) {}

    default void updateInputs(DummyElevatorIOInputs inputs) {}

    default void runOpenLoop(double output) {}

    default void runVolts(double volts) {}

    default void runPosition(double positionRad, double feedforward) {}

    default void stop() {}

    default void setPID(double kP, double kI, double kD) {}

    default void setBrakeMode(boolean enabled) {}
}