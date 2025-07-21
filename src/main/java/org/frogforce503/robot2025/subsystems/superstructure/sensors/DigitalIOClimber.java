package org.frogforce503.robot2025.subsystems.superstructure.sensors;

import org.frogforce503.robot2025.Robot;

import edu.wpi.first.wpilibj.DigitalInput;

public class DigitalIOClimber implements DigitalIO {
    private DigitalInput zeroSwitch;

    public DigitalIOClimber() {
        zeroSwitch = new DigitalInput(Robot.bot.sensorConstants.winchSwitchID());
    }

    @Override
    public void updateInputs(DigitalIOInputs inputs) {
        inputs.data =
            new DigitalIOData(!zeroSwitch.get());
    }
}