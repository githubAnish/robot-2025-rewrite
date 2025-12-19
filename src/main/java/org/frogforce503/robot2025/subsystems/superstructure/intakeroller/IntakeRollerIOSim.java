package org.frogforce503.robot2025.subsystems.superstructure.intakeroller;

import org.frogforce503.robot2025.Constants;
import org.frogforce503.robot2025.Robot;
import org.frogforce503.robot2025.constants.hardware.subsystem_config.IntakeRollerConfig;

import com.revrobotics.spark.SparkSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeRollerIOSim extends IntakeRollerIOSpark {
    // Control
    private final SparkSim motorSim;
    private final DCMotorSim rollerSim;
    
    // Constants
    private final DCMotor motorModel = Robot.bot.getIntakeRollerConfig().isSparkFlex() ? DCMotor.getNeoVortex(1) : DCMotor.getNEO(1);
    private final double moi = 0.00005;

    public IntakeRollerIOSim() {
        final IntakeRollerConfig rollerConfig = Robot.bot.getIntakeRollerConfig();

        motorSim = new SparkSim(super.getMotor(), motorModel);
        rollerSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motorModel, moi, rollerConfig.mechanismRatio()), motorModel);

        // Sync physics and motor sim positions
        motorSim.setVelocity(IntakeRollerConstants.START);
    }

    @Override
    public void updateInputs(IntakeRollerIOInputs inputs) {
        double appliedVolts = motorSim.getAppliedOutput() * RobotController.getBatteryVoltage();
        
        // Apply physics
        rollerSim.setInputVoltage(appliedVolts);
        rollerSim.update(Constants.loopPeriodSecs);

        // Update motor simulation
        motorSim.iterate(rollerSim.getAngularVelocityRadPerSec(), RobotController.getBatteryVoltage(), Constants.loopPeriodSecs);
        
        inputs.data =
            new IntakeRollerIOData(
                true,
                rollerSim.getAngularVelocityRadPerSec(),
                appliedVolts,
                rollerSim.getCurrentDrawAmps(),
                24.0);
    }
}