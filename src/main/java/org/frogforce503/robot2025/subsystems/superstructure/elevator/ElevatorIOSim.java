package org.frogforce503.robot2025.subsystems.superstructure.elevator;

import org.frogforce503.robot2025.Constants;
import org.frogforce503.robot2025.Robot;
import org.frogforce503.robot2025.constants.hardware.subsystem_config.ElevatorConfig;

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
    private final double simCarriageMass = Units.lbsToKilograms(16.895); // from CAD (elevator carriage (6.423 lb) + shoulder (3.000 lb) + arm (2.229 lb) + gripper (5.243 lb))

    public ElevatorIOSim() {
        final ElevatorConfig elevatorConfig = Robot.bot.getElevatorConfig();

        motorSim = new SparkMaxSim(super.getMotor(), motorModel);
        elevatorSim =
            new ElevatorSim(
                motorModel,
                elevatorConfig.mechanismRatio(),
                simCarriageMass,
                elevatorConfig.sprocketPitchDiameter() / 2,
                elevatorConfig.minHeight(),
                elevatorConfig.maxHeight(),
                true,
                ElevatorConstants.START);

        // Sync physics and motor sim positions
        motorSim.setPosition(ElevatorConstants.START);
        motorSim.setVelocity(0.0);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        double appliedVolts = motorSim.getAppliedOutput() * RobotController.getBatteryVoltage();
        
        // Apply physics
        elevatorSim.setInputVoltage(appliedVolts);
        elevatorSim.update(Constants.loopPeriodSecs);

        // Update motor simulation
        motorSim.iterate(elevatorSim.getVelocityMetersPerSecond(), RobotController.getBatteryVoltage(), Constants.loopPeriodSecs);
        motorSim.setPosition(elevatorSim.getPositionMeters());
        motorSim.setVelocity(elevatorSim.getVelocityMetersPerSecond());

        inputs.data =
            new ElevatorIOData(
                true,
                motorSim.getPosition(),
                motorSim.getVelocity(),
                appliedVolts,
                motorSim.getMotorCurrent(),
                24.0,
                !super.getLimitSwitch().get());
    }
}