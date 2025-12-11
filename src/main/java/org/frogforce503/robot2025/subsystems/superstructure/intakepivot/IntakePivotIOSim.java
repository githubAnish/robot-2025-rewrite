package org.frogforce503.robot2025.subsystems.superstructure.intakepivot;

import org.frogforce503.robot2025.Constants;
import org.frogforce503.robot2025.Robot;
import org.frogforce503.robot2025.constants.hardware.subsystem_config.IntakePivotConfig;

import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakePivotIOSim extends IntakePivotIOSpark {
    // Control
    private final SparkMaxSim motorSim;
    private final SingleJointedArmSim pivotSim;

    // Constants
    private final DCMotor motorModel = DCMotor.getNEO(1);
    private final double length = Units.inchesToMeters(14.75);
    private final double moi = 0.68; // kg * m^2

    public IntakePivotIOSim() {
        final IntakePivotConfig pivotConfig = Robot.bot.getIntakePivotConfig();

        motorSim = new SparkMaxSim(super.getMotor(), motorModel);
        pivotSim =
            new SingleJointedArmSim(
                motorModel,
                pivotConfig.mechanismRatio(),
                moi,
                length,
                pivotConfig.minAngle(),
                pivotConfig.maxAngle(),
                true,
                IntakePivotConstants.START);
    }

    @Override
    public void updateInputs(IntakePivotIOInputs inputs) {
        double appliedVolts = motorSim.getAppliedOutput() * RobotController.getBatteryVoltage();
        
        // Apply physics
        pivotSim.setInputVoltage(appliedVolts);
        pivotSim.update(Constants.loopPeriodSecs);

        // Update motor simulation
        motorSim.iterate(pivotSim.getVelocityRadPerSec(), RobotController.getBatteryVoltage(), Constants.loopPeriodSecs);

        inputs.data =
            new IntakePivotIOData(
                true,
                pivotSim.getAngleRads(),
                pivotSim.getVelocityRadPerSec(),
                appliedVolts,
                pivotSim.getCurrentDrawAmps(),
                24.0);
    } 
}