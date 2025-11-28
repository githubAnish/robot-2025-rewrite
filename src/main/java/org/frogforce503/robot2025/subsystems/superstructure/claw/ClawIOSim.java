package org.frogforce503.robot2025.subsystems.superstructure.claw;

import org.frogforce503.robot2025.Constants;
import org.frogforce503.robot2025.Robot;
import org.frogforce503.robot2025.config.subsystem.ClawConfig;

import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClawIOSim extends ClawIOSpark {
    // Control
    private final SparkMaxSim leftMotorSim;
    private final SparkMaxSim rightMotorSim;

    private final DCMotorSim leftRollerSim;
    private final DCMotorSim rightRollerSim;
    
    // Constants
    private final DCMotor motorModel = DCMotor.getNeo550(1);
    private final double moi = 0.0001;

    public ClawIOSim() {
        final ClawConfig clawConfig = Robot.bot.getClawConfig();

        leftMotorSim = new SparkMaxSim(super.getLeftMotor(), motorModel);
        rightMotorSim = new SparkMaxSim(super.getRightMotor(), motorModel);

        leftRollerSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motorModel, moi, clawConfig.mechanismRatio()), motorModel);
        rightRollerSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motorModel, moi, clawConfig.mechanismRatio()), motorModel);
    }

    @Override
    public void updateInputs(ClawIOInputs inputs) {
        double appliedVolts = leftMotorSim.getAppliedOutput() * RobotController.getBatteryVoltage();
        
        // Apply physics
        leftRollerSim.setInputVoltage(appliedVolts);
        leftRollerSim.update(Constants.loopPeriodSecs);
        rightRollerSim.setInputVoltage(appliedVolts);
        rightRollerSim.update(Constants.loopPeriodSecs);

        // Update motor simulation
        leftMotorSim.iterate(leftRollerSim.getAngularVelocityRPM(), RobotController.getBatteryVoltage(), Constants.loopPeriodSecs);
        rightMotorSim.iterate(rightRollerSim.getAngularVelocityRPM(), RobotController.getBatteryVoltage(), Constants.loopPeriodSecs);

        inputs.leftMotorData =
            new ClawIOData(
                true,
                leftMotorSim.getVelocity(),
                appliedVolts,
                leftMotorSim.getMotorCurrent(),
                24.0);

        inputs.rightMotorData =
            new ClawIOData(
                true,
                rightMotorSim.getVelocity(),
                appliedVolts,
                rightMotorSim.getMotorCurrent(),
                24.0);
    }
}