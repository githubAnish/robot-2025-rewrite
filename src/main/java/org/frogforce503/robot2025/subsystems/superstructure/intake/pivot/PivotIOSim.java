package org.frogforce503.robot2025.subsystems.superstructure.intake.pivot;

import org.frogforce503.lib.motor_wrappers.Sim;
import org.frogforce503.lib.motor_wrappers.CANMotor.MotorControlMode;

public class PivotIOSim implements PivotIO {
    private Sim sim;

    public PivotIOSim() {
        sim = new Sim(3.25,360.0);
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.data =
            new PivotIOData(
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
        sim.set(MotorControlMode.PercentOutput, output);
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
        sim.set(MotorControlMode.PercentOutput, 0);
    }  
}