package org.frogforce503.robot2025.commands.tuning;

import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;
import org.frogforce503.lib.util.LoggedTunableNumber;
import org.frogforce503.robot2025.Robot;
import org.frogforce503.robot2025.constants.hardware.subsystem_config.IntakePivotConfig;
import org.frogforce503.robot2025.subsystems.superstructure.intakepivot.IntakePivot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;

public class TuneIntakePivot extends Command {
    private final IntakePivot intakePivot;

    private final LoggedTunableNumber kP;
    private final LoggedTunableNumber kI;
    private final LoggedTunableNumber kD;
    private final LoggedTunableNumber kS;
    private final LoggedTunableNumber kG;
    private final LoggedTunableNumber kV;
    private final LoggedTunableNumber kA;
    private final LoggedTunableNumber maxVel;
    private final LoggedTunableNumber maxAcc;

    private final LoggedTunableNumber setpointAngleRad;

    public TuneIntakePivot(IntakePivot intakePivot) {
        this.intakePivot = intakePivot;

        // Get initial values from config
        final IntakePivotConfig intakePivotConfig = Robot.bot.getIntakePivotConfig();

        final PIDConfig initialPID = intakePivotConfig.kPID();
        final FFConfig initialFF = intakePivotConfig.kFF();
        final Constraints initialConstraints = intakePivotConfig.kConstraints();

        // Create tunable numbers
        this.kP = new LoggedTunableNumber("IntakePivot/kP", initialPID.kP());
        this.kI = new LoggedTunableNumber("IntakePivot/kI", initialPID.kI());
        this.kD = new LoggedTunableNumber("IntakePivot/kD", initialPID.kD());
        this.kS = new LoggedTunableNumber("IntakePivot/kS", initialFF.kS());
        this.kG = new LoggedTunableNumber("IntakePivot/kG", initialFF.kG());
        this.kV = new LoggedTunableNumber("IntakePivot/kV", initialFF.kV());
        this.kA = new LoggedTunableNumber("IntakePivot/kA", initialFF.kA());

        this.maxVel = new LoggedTunableNumber("IntakePivot/MaxVelocityRadPerSec", initialConstraints.maxVelocity);
        this.maxAcc = new LoggedTunableNumber("IntakePivot/MaxAccelerationRadPerSec2", initialConstraints.maxAcceleration);

        this.setpointAngleRad = new LoggedTunableNumber("IntakePivot/SetpointRad", intakePivot.getAngleRad());

        addRequirements(intakePivot);
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
        this.maxVel.setTuningMode(true);
        this.maxAcc.setTuningMode(true);
        this.setpointAngleRad.setTuningMode(true);
    }

    @Override
    public void execute() {
        // Update PID only if changed
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> intakePivot.setPID(kP.get(), kI.get(), kD.get()),
            kP, kI, kD);
        
        // Update FF only if changed
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> intakePivot.setFeedforward(new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get())),
            kS, kG, kV, kA);

        // Update trapezoid profile only if changed
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> intakePivot.setProfile(new TrapezoidProfile(new Constraints(maxVel.get(), maxAcc.get()))),
            maxVel, maxAcc);

        // Update setpoint only if changed
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> intakePivot.setAngle(setpointAngleRad.get()),
            setpointAngleRad);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intakePivot.stop();
    }
}
