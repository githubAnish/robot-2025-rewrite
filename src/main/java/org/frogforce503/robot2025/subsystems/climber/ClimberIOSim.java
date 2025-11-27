package org.frogforce503.robot2025.subsystems.climber;

import org.frogforce503.lib.motorcontrol.SimpleMotorSim;
import org.frogforce503.lib.motorcontrol.SimpleMotorSim.MotorControlMode;
import org.frogforce503.robot2025.Constants;

import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;

public class ClimberIOSim extends ClimberIOSpark {
    // Control
    private final SparkMaxSim motorSim;
    private final SimpleMotorSim climberSim = new SimpleMotorSim(1.0, 42);

    public ClimberIOSim() {
        motorSim = new SparkMaxSim(super.getMotor(), DCMotor.getNEO(1));
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        double appliedVolts = motorSim.getAppliedOutput() * RobotController.getBatteryVoltage();

        // Apply physics
        climberSim.set(MotorControlMode.Voltage, appliedVolts);
        climberSim.update();

        // Update motor simulation
        motorSim.iterate(climberSim.getVelocity(), RobotController.getBatteryVoltage(), Constants.loopPeriodSecs);

        inputs.data =
            new ClimberIOData(
                true,
                0.0,
                0.0,
                24.0);
    }
}