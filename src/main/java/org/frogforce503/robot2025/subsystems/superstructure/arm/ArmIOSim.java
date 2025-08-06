package org.frogforce503.robot2025.subsystems.superstructure.arm;

import org.frogforce503.lib.motorcontrol.MotorControlMode;
import org.frogforce503.lib.motorcontrol.MotorSim;

public class ArmIOSim implements ArmIO {
    private MotorSim sim;

    public ArmIOSim() {
        sim = new MotorSim(1.0, 360.0);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.data =
            new ArmIOData(
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