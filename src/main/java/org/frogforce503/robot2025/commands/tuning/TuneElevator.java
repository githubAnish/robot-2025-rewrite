package org.frogforce503.robot2025.commands.tuning;

import org.frogforce503.lib.logging.LoggedTunableNumber;
import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;
import org.frogforce503.robot2025.Robot;
import org.frogforce503.robot2025.constants.hardware.subsystem_config.ElevatorConfig;
import org.frogforce503.robot2025.subsystems.superstructure.elevator.Elevator;
import org.frogforce503.robot2025.subsystems.superstructure.elevator.ElevatorConstants;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class TuneElevator extends Command {
    private final Elevator elevator;

    private final LoggedTunableNumber kP;
    private final LoggedTunableNumber kI;
    private final LoggedTunableNumber kD;
    private final LoggedTunableNumber kS;
    private final LoggedTunableNumber kG;
    private final LoggedTunableNumber kV;
    private final LoggedTunableNumber kA;
    private final LoggedTunableNumber maxVelocityInchesPerSec;
    private final LoggedTunableNumber maxAccelerationInchesPerSec2;

    private final LoggedTunableNumber setpointHeightInches;

    public TuneElevator(Elevator elevator) {
        this.elevator = elevator;

        // Get initial values from config
        final ElevatorConfig elevatorConfig = Robot.bot.getElevatorConfig();

        final PIDConfig initialPID = elevatorConfig.kPID();
        final FFConfig initialFF = elevatorConfig.kFF();
        final Constraints initialConstraints = elevatorConfig.kConstraints();

        // Create tunable numbers
        this.kP = new LoggedTunableNumber("Elevator/kP", initialPID.kP());
        this.kI = new LoggedTunableNumber("Elevator/kI", initialPID.kI());
        this.kD = new LoggedTunableNumber("Elevator/kD", initialPID.kD());
        this.kS = new LoggedTunableNumber("Elevator/kS", initialFF.kS());
        this.kG = new LoggedTunableNumber("Elevator/kG", initialFF.kG());
        this.kV = new LoggedTunableNumber("Elevator/kV", initialFF.kV());
        this.kA = new LoggedTunableNumber("Elevator/kA", initialFF.kA());

        this.maxVelocityInchesPerSec = new LoggedTunableNumber("Elevator/MaxVelocityInchesPerSec", Units.metersToInches(initialConstraints.maxVelocity));
        this.maxAccelerationInchesPerSec2 = new LoggedTunableNumber("Elevator/MaxAccelerationInchesPerSec2", Units.metersToInches(initialConstraints.maxAcceleration));

        this.setpointHeightInches = new LoggedTunableNumber("Elevator/SetpointInches", Units.metersToInches(ElevatorConstants.START));

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        // Set tuning mode to true
        this.kP.setTuningMode(true);
        this.kI.setTuningMode(true);
        this.kD.setTuningMode(true);
        this.kS.setTuningMode(true);
        this.kG.setTuningMode(true);
        this.kV.setTuningMode(true);
        this.kA.setTuningMode(true);
        this.maxVelocityInchesPerSec.setTuningMode(true);
        this.maxAccelerationInchesPerSec2.setTuningMode(true);
        this.setpointHeightInches.setTuningMode(true);
    }

    @Override
    public void execute() {
        // Update PID only if changed
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> elevator.setPID(kP.get(), kI.get(), kD.get()),
            kP, kI, kD);
        
        // Update FF only if changed
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> elevator.setFeedforward(new ElevatorFeedforward(kS.get(), kG.get(), kV.get(), kA.get())),
            kS, kG, kV, kA);

        // Update trapezoid profile only if changed
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> elevator.setProfile(new TrapezoidProfile(new Constraints(Units.inchesToMeters(maxVelocityInchesPerSec.get()), Units.inchesToMeters(maxAccelerationInchesPerSec2.get())))),
            maxVelocityInchesPerSec, maxAccelerationInchesPerSec2);

        // Update setpoint only if changed
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> elevator.setHeight(Units.inchesToMeters(setpointHeightInches.get())),
            setpointHeightInches);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }
}
