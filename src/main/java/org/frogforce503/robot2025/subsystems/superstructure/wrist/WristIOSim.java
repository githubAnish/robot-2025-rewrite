package org.frogforce503.robot2025.subsystems.superstructure.wrist;

import org.frogforce503.lib.motor_wrappers.Sim;
import org.frogforce503.lib.motor_wrappers.CANMotor.MotorControlMode;

public class WristIOSim implements WristIO {
    private Sim sim;

    public WristIOSim() {
        sim = new Sim(3.25,360.0);
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.data =
            new WristIOData(
                true,
                sim.getPosition(),
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

    @Override
    public void setEncoderPosition(double position) {
        sim.setEncoderPosition(position);
    }
}