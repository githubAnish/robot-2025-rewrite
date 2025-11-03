package org.frogforce503.robot2025.subsystems.superstructure.sensors;

import org.frogforce503.robot2025.Robot;

import edu.wpi.first.wpilibj.DigitalInput;

public class LimitSwitchIOClimber implements LimitSwitchIO {
    private DigitalInput zeroSwitch;

    public LimitSwitchIOClimber() {
        zeroSwitch = new DigitalInput(Robot.bot.getSensorsConfig().winchSwitchID());
    }

    @Override
    public void updateInputs(DigitalIOInputs inputs) {
        inputs.data =
            new DigitalIOData(!zeroSwitch.get());
    }
}