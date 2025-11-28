package org.frogforce503.robot2025.subsystems.superstructure.wrist;

import org.frogforce503.robot2025.Constants;
import org.frogforce503.robot2025.Robot;
import org.frogforce503.robot2025.config.subsystem.WristConfig;

import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class WristIOSim extends WristIOSpark {
    // Control
    private final SparkMaxSim motorSim;
    private final SingleJointedArmSim wristSim;

    // Constants
    private final DCMotor motorModel = DCMotor.getNeo550(1);
    private final double length = Units.inchesToMeters(2.5);
    private final double moi = 0.075; // kg * m^2

    public WristIOSim() {
        final WristConfig wristConfig = Robot.bot.getWristConfig();

        motorSim = new SparkMaxSim(super.getMotor(), motorModel);
        wristSim =
            new SingleJointedArmSim(
                motorModel,
                wristConfig.mechanismRatio(),
                moi,
                length,
                wristConfig.motionRange().min(),
                wristConfig.motionRange().max(),
                true,
                WristConstants.START);
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        double appliedVolts = motorSim.getAppliedOutput() * RobotController.getBatteryVoltage();
        
        // Apply physics
        wristSim.setInputVoltage(appliedVolts);
        wristSim.update(Constants.loopPeriodSecs);

        // Update motor simulation
        motorSim.iterate(wristSim.getVelocityRadPerSec(), RobotController.getBatteryVoltage(), Constants.loopPeriodSecs);

        inputs.data =
            new WristIOData(
                true,
                motorSim.getPosition(),
                motorSim.getPosition(), // In sim, assume absolute encoder pos matches relative pos
                motorSim.getVelocity(),
                appliedVolts,
                motorSim.getMotorCurrent(),
                24.0);
    }
}