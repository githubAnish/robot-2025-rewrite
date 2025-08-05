package org.frogforce503.robot2025.subsystems.climber;

import org.frogforce503.lib.motor.MotorSim;
import org.frogforce503.lib.motor.MotorControlMode;

public class ClimberIOSim implements ClimberIO {
    private MotorSim sim;

    public ClimberIOSim() {
        sim = new MotorSim(3.25, 360);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.data =
            new ClimberIOData(
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
    public void runTorqueCurrent(double current) {
        sim.set(MotorControlMode.Current, current);
    }

    @Override
    public void stop() {
        sim.set(MotorControlMode.DutyCycle, 0);
    }
}