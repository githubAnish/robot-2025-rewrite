package org.frogforce503.robot2025.subsystems.superstructure.sensors;

import org.frogforce503.robot2025.Robot;

import edu.wpi.first.wpilibj.DigitalInput;

public class LimitSwitchIOElevator implements LimitSwitchIO {
    private DigitalInput zeroSwitch;

    public LimitSwitchIOElevator() {
        zeroSwitch = new DigitalInput(Robot.bot.getSensorsConfig().elevatorZeroSwitchID());
    }

    @Override
    public void updateInputs(LimitSwitchIOInputs inputs) {
        inputs.data =
            new LimitSwitchIOData(!zeroSwitch.get());
    }
}