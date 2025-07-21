package org.frogforce503.robot2025.subsystems.superstructure.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    class ElevatorIOInputs {
        public ElevatorIOData data = new ElevatorIOData(false, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    record ElevatorIOData(
        boolean motorConnected,
        double position,
        double velocity,
        double appliedVolts,
        double statorCurrentAmps,
        double tempCelsius) {}

    default void updateInputs(ElevatorIOInputs inputs) {}

    default void runOpenLoop(double output) {}

    default void runVolts(double volts) {}

    default void runPosition(double position, double feedforward) {}

    default void stop() {}

    default void setPID(double kP, double kI, double kD) {}

    default void setBrakeMode(boolean enabled) {}  
    
    default void resetEncoder() {}
}