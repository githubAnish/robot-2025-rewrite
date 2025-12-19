package org.frogforce503.robot2025.subsystems.superstructure.arm;

import org.frogforce503.robot2025.Constants;
import org.frogforce503.robot2025.Robot;
import org.frogforce503.robot2025.constants.hardware.subsystem_config.ArmConfig;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim extends ArmIOSpark {
    // Control
    private final SparkMaxSim motorSim;
    private final SingleJointedArmSim armSim;

    // Constants
    private final DCMotor motorModel = DCMotor.getNEO(1);
    private final double length = Units.inchesToMeters(19);
    private final double moi = 0.95; // kg * m^2

    public ArmIOSim() {
        final ArmConfig armConfig = Robot.bot.getArmConfig();

        motorSim = new SparkMaxSim(super.getMotor(), motorModel);
        armSim =
            new SingleJointedArmSim(
                motorModel,
                armConfig.mechanismRatio(),
                moi,
                length,
                armConfig.minAngle(),
                armConfig.maxAngle(),
                true,
                ArmConstants.START);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        double appliedVolts = motorSim.getAppliedOutput() * RobotController.getBatteryVoltage();
        
        // Apply physics
        armSim.setInputVoltage(appliedVolts);
        armSim.update(Constants.loopPeriodSecs);

        // Update motor simulation
        motorSim.iterate(armSim.getVelocityRadPerSec(), RobotController.getBatteryVoltage(), Constants.loopPeriodSecs);

        // motorSim.setPosition(armSim.getAngleRads());
        // motorSim.setVelocity(armSim.getVelocityRadPerSec());

        inputs.data =
            new ArmIOData(
                true,
                armSim.getAngleRads(),
                armSim.getVelocityRadPerSec(),
                appliedVolts,
                armSim.getCurrentDrawAmps(),
                24.0);
    }
}