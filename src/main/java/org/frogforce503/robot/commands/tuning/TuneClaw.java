package org.frogforce503.robot.commands.tuning;

import org.frogforce503.lib.logging.LoggedTunableNumber;
import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;
import org.frogforce503.robot.Robot;
import org.frogforce503.robot.constants.hardware.subsystem_config.ClawConfig;
import org.frogforce503.robot.subsystems.superstructure.claw.Claw;
import org.frogforce503.robot.subsystems.superstructure.claw.ClawConstants;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class TuneClaw extends Command {
    private final Claw claw;

    private final LoggedTunableNumber kP;
    private final LoggedTunableNumber kI;
    private final LoggedTunableNumber kD;
    private final LoggedTunableNumber kS;
    private final LoggedTunableNumber kV;
    private final LoggedTunableNumber kA;

    private final LoggedTunableNumber setpointVelocityRpm;

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

        this.setpointVelocityRpm = new LoggedTunableNumber("Claw/SetpointRpm", Units.radiansPerSecondToRotationsPerMinute(ClawConstants.START));

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
        this.setpointVelocityRpm.setTuningMode(true);
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
            () -> claw.setVelocity(Units.rotationsPerMinuteToRadiansPerSecond(setpointVelocityRpm.get())),
            setpointVelocityRpm);
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
