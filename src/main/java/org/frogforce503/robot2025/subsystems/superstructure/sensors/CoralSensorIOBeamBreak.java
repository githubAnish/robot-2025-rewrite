package org.frogforce503.robot2025.subsystems.superstructure.sensors;

import java.time.Duration;

import org.frogforce503.robot2025.Robot;
import org.frogforce503.robot2025.constants.hardware.subsystem_config.SensorConfig;

import edu.wpi.first.wpilibj.DigitalGlitchFilter;
import edu.wpi.first.wpilibj.DigitalInput;

public class CoralSensorIOBeamBreak implements CoralSensorIO {
    // Hardware
    private final DigitalInput upperBeamBreak;
    private final DigitalInput lowerBeamBreak;

    // Filters
    private final DigitalGlitchFilter beamBreakFilter = new DigitalGlitchFilter();

    public CoralSensorIOBeamBreak() {
        final SensorConfig sensorConfig = Robot.bot.getSensorConfig();

        // Initialize beam breaks
        upperBeamBreak = new DigitalInput(sensorConfig.upperBeamBreakId());
        lowerBeamBreak = new DigitalInput(sensorConfig.lowerBeamBreakId());
        beamBreakFilter.setPeriodNanoSeconds(Duration.ofMillis(10).toNanos());
        beamBreakFilter.add(upperBeamBreak);
        beamBreakFilter.add(lowerBeamBreak);
    }

    @Override
    public void updateInputs(CoralSensorIOInputs inputs) {
        inputs.data =
            new CoralSensorIOData(
                upperBeamBreak.get(),
                lowerBeamBreak.get());
    }
}
