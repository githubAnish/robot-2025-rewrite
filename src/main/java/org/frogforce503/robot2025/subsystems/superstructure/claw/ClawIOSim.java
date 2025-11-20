package org.frogforce503.robot2025.subsystems.superstructure.claw;

import org.frogforce503.lib.motorcontrol.MotorControlMode;
import org.frogforce503.lib.motorcontrol.SimpleMotorSim;

public class ClawIOSim implements ClawIO {
    private SimpleMotorSim leftSim;
    private SimpleMotorSim rightSim;

    public ClawIOSim() {
        leftSim = new SimpleMotorSim(3.25,360.0);
        rightSim = new SimpleMotorSim(3.25,360.0);
    }

    @Override
    public void updateInputs(ClawIOInputs inputs) {
        inputs.leftMotorData =
            new ClawIOData(
                true,
                leftSim.getPosition(),
                leftSim.getVelocity(),
                0.0,
                0.0,
                24.0);

        inputs.rightMotorData =
            new ClawIOData(
                true,
                leftSim.getPosition(),
                leftSim.getVelocity(),
                0.0,
                0.0,
                24.0);
        
        leftSim.update();
        rightSim.update();
    }

    @Override
    public void runOpenLoop(double outputLeft, double outputRight) {
        leftSim.set(MotorControlMode.DutyCycle, outputLeft);
        rightSim.set(MotorControlMode.DutyCycle, outputRight);
    }

    @Override
    public void runVolts(double voltsLeft, double voltsRight) {
        leftSim.set(MotorControlMode.Voltage, voltsLeft);
        rightSim.set(MotorControlMode.Voltage, voltsRight);
    }

    @Override
    public void runVelocity(double velocityLeft, double velocityRight, double feedforward) {
        leftSim.set(MotorControlMode.Velocity, velocityLeft);
        rightSim.set(MotorControlMode.Velocity, velocityRight);
    }

    @Override
    public void stop() {
        leftSim.set(MotorControlMode.DutyCycle, 0.0);
        rightSim.set(MotorControlMode.DutyCycle, 0.0);
    }
}