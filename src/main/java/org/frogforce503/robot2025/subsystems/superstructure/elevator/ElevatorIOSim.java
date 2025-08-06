package org.frogforce503.robot2025.subsystems.superstructure.elevator;

import org.frogforce503.lib.motorcontrol.MotorControlMode;
import org.frogforce503.lib.motorcontrol.MotorSim;

public class ElevatorIOSim implements ElevatorIO {
    private MotorSim sim;

    public ElevatorIOSim() {
        sim = new MotorSim(3.25,360.0);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.data =
            new ElevatorIOData(
                true,
                sim.getPosition(),
                sim.getVelocity(),
                0.0,
                0.0,
                24.0);
        
        sim.update();
    }

    @Override
    public void runOpenLoop(double output) {
        sim.set(MotorControlMode.DutyCycle, output);
    }

    @Override
    public void runVolts(double volts) {
        sim.set(MotorControlMode.Voltage, volts);
    }

    @Override
    public void runPosition(double position, double feedforward) {
        sim.set(MotorControlMode.Position, position);
    }

    @Override
    public void stop() {
        sim.set(MotorControlMode.DutyCycle, 0);
    }
}