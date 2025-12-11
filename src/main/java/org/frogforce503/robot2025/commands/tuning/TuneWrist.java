package org.frogforce503.robot2025.commands.tuning;

import org.frogforce503.lib.logging.LoggedTunableNumber;
import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;
import org.frogforce503.robot2025.Robot;
import org.frogforce503.robot2025.constants.hardware.subsystem_config.WristConfig;
import org.frogforce503.robot2025.subsystems.superstructure.wrist.Wrist;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.Command;

public class TuneWrist extends Command {
    private final Wrist wrist;

    private final LoggedTunableNumber kP;
    private final LoggedTunableNumber kI;
    private final LoggedTunableNumber kD;
    private final LoggedTunableNumber kS;
    private final LoggedTunableNumber kG;
    private final LoggedTunableNumber kV;
    private final LoggedTunableNumber kA;

    private final LoggedTunableNumber setpointAngleRad;

    public TuneWrist(Wrist wrist) {
        this.wrist = wrist;

        // Get initial values from config
        final WristConfig wristConfig = Robot.bot.getWristConfig();

        final PIDConfig initialPID = wristConfig.kPID();
        final FFConfig initialFF = wristConfig.kFF();

        // Create tunable numbers
        this.kP = new LoggedTunableNumber("Wrist/kP", initialPID.kP());
        this.kI = new LoggedTunableNumber("Wrist/kI", initialPID.kI());
        this.kD = new LoggedTunableNumber("Wrist/kD", initialPID.kD());
        this.kS = new LoggedTunableNumber("Wrist/kS", initialFF.kS());
        this.kG = new LoggedTunableNumber("Wrist/kG", initialFF.kG());
        this.kV = new LoggedTunableNumber("Wrist/kV", initialFF.kV());
        this.kA = new LoggedTunableNumber("Wrist/kA", initialFF.kA());

        this.setpointAngleRad = new LoggedTunableNumber("Wrist/SetpointRad", wrist.getRelativeAngleRad());

        addRequirements(wrist);
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
        this.setpointAngleRad.setTuningMode(true);
    }

    @Override
    public void execute() {
        // Update PID only if changed
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> wrist.setPID(kP.get(), kI.get(), kD.get()),
            kP, kI, kD);
        
        // Update FF only if changed
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> wrist.setFeedforward(new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get())),
            kS, kG, kV, kA);

        // Update setpoint only if changed
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> wrist.setAngle(setpointAngleRad.get()),
            setpointAngleRad);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        wrist.stop();
    }
}
