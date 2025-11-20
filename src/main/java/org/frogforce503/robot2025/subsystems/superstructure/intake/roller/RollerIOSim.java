package org.frogforce503.robot2025.subsystems.superstructure.intake.roller;

import org.frogforce503.lib.motorcontrol.MotorControlMode;
import org.frogforce503.lib.motorcontrol.SimpleMotorSim;

public class RollerIOSim implements RollerIO {
    private SimpleMotorSim sim;

    public RollerIOSim() {
        sim = new SimpleMotorSim(1.0, 360.0);
    }

    @Override
    public void updateInputs(RollerIOInputs inputs) {
        inputs.data =
            new RollerIOData(
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
    public void runVelocity(double velocity, double feedforward) {
        sim.set(MotorControlMode.Position, velocity);
    }

    @Override
    public void stop() {
        sim.set(MotorControlMode.DutyCycle, 0);
    }
}