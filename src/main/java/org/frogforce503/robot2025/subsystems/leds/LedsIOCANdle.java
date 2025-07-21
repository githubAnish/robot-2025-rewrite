package org.frogforce503.robot2025.subsystems.leds;

import org.frogforce503.robot2025.Robot;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdleStatusFrame;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class LedsIOCANdle implements LedsIO {
    private CANdle leds;

    public LedsIOCANdle() {
        leds = new CANdle(Robot.bot.candleID);

        CANdleConfiguration config = new CANdleConfiguration();

        leds.configFactoryDefault();
        leds.clearStickyFaults();

        leds.setStatusFramePeriod(CANdleStatusFrame.CANdleStatusFrame_Status_1_General, 50);

        leds.configAllSettings(config);
    }

    @Override
    public void updateInputs(LedsIOInputs inputs) {
        inputs.data =
            new LedsIOData(
                leds.getLastError() == ErrorCode.OK,
                leds.getBusVoltage(),
                leds.getCurrent(),
                leds.getTemperature());
    }

    @Override
    public void runColor(Color color) {
        Color8Bit simpleColor = new Color8Bit(color);

        leds.setLEDs(
            (int) simpleColor.red * 255,
            (int) simpleColor.green * 255,
            (int) simpleColor.blue * 255
        );
    }

    @Override
    public void runAnimation(Animation animation) {
        leds.animate(animation);
    }

    @Override
    public void stop() {
        leds.setLEDs(0, 0, 0);
        leds.clearAnimation(0);
    }
}