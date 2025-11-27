package org.frogforce503.robot2025.commands.tuning;

import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;
import org.frogforce503.lib.util.LoggedTunableNumber;
import org.frogforce503.robot2025.Robot;
import org.frogforce503.robot2025.config.subsystem.IntakeRollerConfig;
import org.frogforce503.robot2025.subsystems.superstructure.intakeroller.IntakeRoller;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;

public class TuneIntakeRoller extends Command {
    private final IntakeRoller intakeRoller;

    private final LoggedTunableNumber kP;
    private final LoggedTunableNumber kI;
    private final LoggedTunableNumber kD;
    private final LoggedTunableNumber kS;
    private final LoggedTunableNumber kV;
    private final LoggedTunableNumber kA;

    private final LoggedTunableNumber setpointVelocity;

    public TuneIntakeRoller(IntakeRoller intakeRoller) {
        this.intakeRoller = intakeRoller;

        // Get initial values from config
        final IntakeRollerConfig intakeRollerConfig = Robot.bot.getIntakeRollerConfig();

        final PIDConfig initialPID = intakeRollerConfig.kPID();
        final FFConfig initialFF = intakeRollerConfig.kFF();

        // Create tunable numbers
        this.kP = new LoggedTunableNumber("IntakeRoller/kP", initialPID.kP());
        this.kI = new LoggedTunableNumber("IntakeRoller/kI", initialPID.kI());
        this.kD = new LoggedTunableNumber("IntakeRoller/kD", initialPID.kD());
        this.kS = new LoggedTunableNumber("IntakeRoller/kS", initialFF.kS());
        this.kV = new LoggedTunableNumber("IntakeRoller/kV", initialFF.kV());
        this.kA = new LoggedTunableNumber("IntakeRoller/kA", initialFF.kA());

        this.setpointVelocity = new LoggedTunableNumber("IntakeRoller/SetpointRPM", intakeRoller.getVelocityRPM());

        addRequirements(intakeRoller);
    }

    @Override
    public void initialize() {
        // Set tuning mode to true
        this.kP.setTuningMode(true);
        this.kI.setTuningMode(true);
        this.kD.setTuningMode(true);
        this.kS.setTuningMode(true);
        this.kV.setTuningMode(true);
        this.kA.setTuningMode(true);
    }

    @Override
    public void execute() {
        // Update PID only if changed
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> intakeRoller.setPID(kP.get(), kI.get(), kD.get()),
            kP, kI, kD);
        
        // Update FF only if changed
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> intakeRoller.setFeedforward(new SimpleMotorFeedforward(kS.get(), kV.get(), kA.get())),
            kS, kV, kA);

        // Update setpoint only if changed
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> intakeRoller.setVelocity(setpointVelocity.get()),
            setpointVelocity);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intakeRoller.stop();
    }
}
