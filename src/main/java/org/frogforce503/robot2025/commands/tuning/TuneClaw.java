package org.frogforce503.robot2025.commands.tuning;

import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;
import org.frogforce503.lib.util.LoggedTunableNumber;
import org.frogforce503.robot2025.Robot;
import org.frogforce503.robot2025.config.subsystem.ClawConfig;
import org.frogforce503.robot2025.subsystems.superstructure.claw.Claw;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;

public class TuneClaw extends Command {
    private final Claw claw;

    private final LoggedTunableNumber kP;
    private final LoggedTunableNumber kI;
    private final LoggedTunableNumber kD;
    private final LoggedTunableNumber kS;
    private final LoggedTunableNumber kV;
    private final LoggedTunableNumber kA;

    private final LoggedTunableNumber setpointVelocity;

    public TuneClaw(Claw claw) {
        this.claw = claw;

        // Get initial values from config
        final ClawConfig clawConfig = Robot.bot.getClawConfig();

        final PIDConfig initialPID = clawConfig.kPID();
        final FFConfig initialFF = clawConfig.kFF();

        // Create tunable numbers
        this.kP = new LoggedTunableNumber("Claw/kP", initialPID.kP());
        this.kI = new LoggedTunableNumber("Claw/kI", initialPID.kI());
        this.kD = new LoggedTunableNumber("Claw/kD", initialPID.kD());
        this.kS = new LoggedTunableNumber("Claw/kS", initialFF.kS());
        this.kV = new LoggedTunableNumber("Claw/kV", initialFF.kV());
        this.kA = new LoggedTunableNumber("Claw/kA", initialFF.kA());

        this.setpointVelocity = new LoggedTunableNumber("Claw/SetpointRPM", (claw.getLeftVelocityRPM() + claw.getRightVelocityRPM()) / 2.0);

        addRequirements(claw);
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
            () -> claw.setPID(kP.get(), kI.get(), kD.get()),
            kP, kI, kD);
        
        // Update FF only if changed
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> claw.setFeedforward(new SimpleMotorFeedforward(kS.get(), kV.get(), kA.get())),
            kS, kV, kA);

        // Update setpoint only if changed
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> claw.setVelocity(setpointVelocity.get()),
            setpointVelocity);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        claw.stop();
    }
}
