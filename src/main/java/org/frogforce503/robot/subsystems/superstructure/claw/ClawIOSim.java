package org.frogforce503.robot.subsystems.superstructure.claw;

import org.frogforce503.robot.Constants;
import org.frogforce503.robot.Robot;
import org.frogforce503.robot.constants.hardware.subsystem_config.ClawConfig;

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

        // Sync physics and motor sim positions
        leftMotorSim.setVelocity(ClawConstants.START);
        rightMotorSim.setVelocity(ClawConstants.START);
    }

    @Override
    public void updateInputs(ClawIOInputs inputs) {
        double leftAppliedVolts = leftMotorSim.getAppliedOutput() * RobotController.getBatteryVoltage();
        double rightAppliedVolts = rightMotorSim.getAppliedOutput() * RobotController.getBatteryVoltage();
        
        // Apply physics
        leftRollerSim.setInputVoltage(leftAppliedVolts);
        leftRollerSim.update(Constants.loopPeriodSecs);

        rightRollerSim.setInputVoltage(rightAppliedVolts);
        rightRollerSim.update(Constants.loopPeriodSecs);

        // Update motor simulation
        leftMotorSim.iterate(leftRollerSim.getAngularVelocityRadPerSec(), RobotController.getBatteryVoltage(), Constants.loopPeriodSecs);
        leftMotorSim.setVelocity(leftRollerSim.getAngularVelocityRadPerSec());

        rightMotorSim.iterate(rightRollerSim.getAngularVelocityRadPerSec(), RobotController.getBatteryVoltage(), Constants.loopPeriodSecs);
        rightMotorSim.setVelocity(rightRollerSim.getAngularVelocityRadPerSec());

        inputs.leftData =
            new ClawIOData(
                true,
                leftMotorSim.getVelocity(),
                leftAppliedVolts,
                leftMotorSim.getMotorCurrent(),
                24.0);

        inputs.rightData =
            new ClawIOData(
                true,
                rightMotorSim.getVelocity(),
                rightAppliedVolts,
                rightMotorSim.getMotorCurrent(),
                24.0);
    }
}