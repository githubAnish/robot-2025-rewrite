package org.frogforce503.robot2025.commands.tuning;

import org.frogforce503.lib.motorcontrol.FFConfig;
import org.frogforce503.lib.motorcontrol.PIDConfig;
import org.frogforce503.lib.util.LoggedTunableNumber;
import org.frogforce503.robot2025.Robot;
import org.frogforce503.robot2025.subsystems.superstructure.arm.Arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;

public class TuneIntakePivot extends Command {
    private final Arm arm;

    private final LoggedTunableNumber kP;
    private final LoggedTunableNumber kI;
    private final LoggedTunableNumber kD;
    private final LoggedTunableNumber kS;
    private final LoggedTunableNumber kG;
    private final LoggedTunableNumber kV;
    private final LoggedTunableNumber kA;
    private final LoggedTunableNumber maxVel;
    private final LoggedTunableNumber maxAcc;

    private final LoggedTunableNumber setpoint;

    public TuneIntakePivot(Arm arm) {
        this.arm = arm;

        // Get initial values from config
        final PIDConfig initialPID = Robot.bot.getArmConfig().kPID();
        final FFConfig initialFF = Robot.bot.getArmConfig().kFF();
        final Constraints initialConstraints = Robot.bot.getArmConfig().kConstraints();

        // Create tunable numbers
        this.kP = new LoggedTunableNumber("Arm/kP", initialPID.kP());
        this.kI = new LoggedTunableNumber("Arm/kI", initialPID.kI());
        this.kD = new LoggedTunableNumber("Arm/kD", initialPID.kD());
        this.kS = new LoggedTunableNumber("Arm/kS", initialFF.kS());
        this.kG = new LoggedTunableNumber("Arm/kG", initialFF.kG());
        this.kV = new LoggedTunableNumber("Arm/kV", initialFF.kV());
        this.kA = new LoggedTunableNumber("Arm/kA", initialFF.kA());

        this.maxVel = new LoggedTunableNumber("Arm/MaxVelocityMetersPerSec", initialConstraints.maxVelocity);
        this.maxAcc = new LoggedTunableNumber("Arm/MaxAccelerationMetersPerSec2", initialConstraints.maxAcceleration);

        this.setpoint = new LoggedTunableNumber("Arm/Setpoint", arm.getAngle());

        addRequirements(arm);
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
    }

    @Override
    public void execute() {
        // Update PID only if changed
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> arm.setPID(kP.get(), kI.get(), kD.get()),
            kP, kI, kD);
        
        // Update FF only if changed
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> arm.setFeedforward(new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get())),
            kS, kG, kV, kA);

        // Update constraints only if changed
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> arm.setConstraints(new Constraints(maxVel.get(), maxAcc.get())),
            maxVel, maxAcc);

        // Update setpoint only if changed
        if (setpoint.hasChanged(hashCode())) {
            arm.setAngle(setpoint.get());
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }
}