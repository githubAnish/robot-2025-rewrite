package org.frogforce503.robot2025.subsystems.superstructure.sensors;

import org.frogforce503.robot2025.Robot;

import edu.wpi.first.wpilibj.DigitalInput;

public class DigitalIOElevator implements DigitalIO {
    private DigitalInput zeroSwitch;

    public DigitalIOElevator() {
        zeroSwitch = new DigitalInput(Robot.bot.sensorConstants.elevatorZeroSwitchID());
    }

    @Override
    public void updateInputs(DigitalIOInputs inputs) {
        inputs.data =
            new DigitalIOData(!zeroSwitch.get());
    }
}