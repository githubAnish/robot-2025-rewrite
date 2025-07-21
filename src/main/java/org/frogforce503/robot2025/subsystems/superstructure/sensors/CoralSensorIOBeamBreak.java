package org.frogforce503.robot2025.subsystems.superstructure.sensors;

import org.frogforce503.robot2025.Robot;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;

public class CoralSensorIOBeamBreak implements CoralSensorIO {
    private DigitalInput upperBeamBreak, lowerBeamBreak;

    private Debouncer upperDebouncer = new Debouncer(0.1);
    private Debouncer lowerDebouncer = new Debouncer(0.1);

    public CoralSensorIOBeamBreak() {
        upperBeamBreak = new DigitalInput(Robot.bot.sensorConstants.upperBeamID());
        lowerBeamBreak = new DigitalInput(Robot.bot.sensorConstants.lowerBeamID());
    }

    @Override
    public void updateInputs(CoralSensorIOInputs inputs) {
        inputs.data =
            new CoralSensorIOData(
                upperDebouncer.calculate(upperBeamBreak.get()),
                lowerDebouncer.calculate(lowerBeamBreak.get()));
    }
}
