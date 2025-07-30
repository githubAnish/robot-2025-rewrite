package org.frogforce503.robot2025.subsystems.superstructure.claw;

import org.frogforce503.lib.motor.Sim;
import org.frogforce503.lib.motor.CANMotor.MotorControlMode;

public class ClawIOSim implements ClawIO {
    private Sim leftSim;
    private Sim rightSim;

    public ClawIOSim() {
        leftSim = new Sim(3.25,360.0);
        rightSim = new Sim(3.25,360.0);
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
        leftSim.set(MotorControlMode.PercentOutput, outputLeft);
        rightSim.set(MotorControlMode.PercentOutput, outputRight);
    }

    @Override
    public void runVolts(double voltsLeft, double voltsRight) {
        leftSim.set(MotorControlMode.Voltage, voltsLeft);
        rightSim.set(MotorControlMode.Voltage, voltsRight);
    }

    @Override
    public void runVelocity(double velocityLeft, double velocityRight) {
        leftSim.set(MotorControlMode.Velocity, velocityLeft);
        rightSim.set(MotorControlMode.Velocity, velocityRight);
    }

    @Override
    public void stop() {
        leftSim.set(MotorControlMode.PercentOutput, 0.0);
        rightSim.set(MotorControlMode.PercentOutput, 0.0);
    }
}