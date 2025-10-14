package org.frogforce503.robot2025.subsystems.superstructure.sensors;

import org.frogforce503.robot2025.Constants;

import edu.wpi.first.wpilibj.DigitalInput;

public class DigitalIOElevator implements DigitalIO {
    private DigitalInput zeroSwitch;

    public DigitalIOElevator() {
        zeroSwitch = new DigitalInput(Constants.bot.Sensors.elevatorZeroSwitchID());
    }

    @Override
    public void updateInputs(DigitalIOInputs inputs) {
        inputs.data =
            new DigitalIOData(!zeroSwitch.get());
    }
}