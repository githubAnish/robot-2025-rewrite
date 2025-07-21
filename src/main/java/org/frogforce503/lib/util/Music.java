package org.frogforce503.lib.util;

import java.util.Arrays;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Music {
    private Orchestra orchestra;
    
    public Music(TalonFX... motors) {
        orchestra = new Orchestra();

        for (TalonFX motor : motors) {
            orchestra.addInstrument(motor, 0);
        }

        orchestra.loadMusic("music/duck.chrp");
    }

    public Music(SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain) {
        this(
            Arrays
                .stream(drivetrain.getModules())
                .flatMap(
                    module ->
                        Arrays.stream(
                            new TalonFX[] {
                                module.getDriveMotor(),
                                module.getSteerMotor()}))
                .toArray(TalonFX[]::new));
    }

    public Command play() {
        return
            Commands.runOnce(orchestra::play)
                .ignoringDisable(true);
    }

    public Command stop() {
        return
            Commands.runOnce(orchestra::stop)
                .ignoringDisable(true);
    }
}