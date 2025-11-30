package org.frogforce503.robot2025.subsystems.superstructure.elevator;

import org.frogforce503.robot2025.Constants;
import org.frogforce503.robot2025.Robot;
import org.frogforce503.robot2025.config.subsystem.ElevatorConfig;

import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim extends ElevatorIOSpark {
    // Control
    private final SparkMaxSim motorSim;
    private final ElevatorSim elevatorSim;

    // Constants
    private final DCMotor motorModel = DCMotor.getNEO(1);
    private final double simCarriageMass = Units.lbsToKilograms(20.163);

    public ElevatorIOSim() {
        final ElevatorConfig elevatorConfig = Robot.bot.getElevatorConfig();

        motorSim = new SparkMaxSim(super.getMotor(), motorModel);
        elevatorSim =
            new ElevatorSim(
                motorModel,
                elevatorConfig.mechanismRatio(),
                simCarriageMass,
                elevatorConfig.sprocketPitchDiameter() / 2,
                elevatorConfig.motionRange().min(),
                elevatorConfig.motionRange().max(),
                true,
                ElevatorConstants.START);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        double appliedVolts = motorSim.getAppliedOutput() * RobotController.getBatteryVoltage();
        
        // Apply physics
        elevatorSim.setInputVoltage(appliedVolts);
        elevatorSim.update(Constants.loopPeriodSecs);

        // Update motor simulation
        motorSim.iterate(elevatorSim.getVelocityMetersPerSecond(), RobotController.getBatteryVoltage(), Constants.loopPeriodSecs);

        inputs.data =
            new ElevatorIOData(
                true,
                elevatorSim.getPositionMeters(),
                elevatorSim.getVelocityMetersPerSecond(),
                appliedVolts,
                elevatorSim.getCurrentDrawAmps(),
                24.0);
    }
}