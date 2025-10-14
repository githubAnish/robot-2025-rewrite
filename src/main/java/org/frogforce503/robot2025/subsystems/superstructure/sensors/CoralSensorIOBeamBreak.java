package org.frogforce503.robot2025.subsystems.superstructure.sensors;

import org.frogforce503.robot2025.Constants;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;

public class CoralSensorIOBeamBreak implements CoralSensorIO {
    private DigitalInput upperBeamBreak, lowerBeamBreak;

    private Debouncer upperDebouncer = new Debouncer(0.1);
    private Debouncer lowerDebouncer = new Debouncer(0.1);

    public CoralSensorIOBeamBreak() {
        upperBeamBreak = new DigitalInput(Constants.bot.Sensors.upperBeamID());
        lowerBeamBreak = new DigitalInput(Constants.bot.Sensors.lowerBeamID());
    }

    @Override
    public void updateInputs(CoralSensorIOInputs inputs) {
        inputs.data =
            new CoralSensorIOData(
                upperDebouncer.calculate(upperBeamBreak.get()),
                lowerDebouncer.calculate(lowerBeamBreak.get()));
    }
}
