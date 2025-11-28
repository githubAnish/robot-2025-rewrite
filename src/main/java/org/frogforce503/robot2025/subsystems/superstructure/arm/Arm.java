package org.frogforce503.robot2025.subsystems.superstructure.arm;

import org.frogforce503.robot2025.Constants;
import org.frogforce503.robot2025.Robot;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.RobotState;
import lombok.Getter;
import lombok.Setter;

import org.frogforce503.lib.math.Range;
import org.frogforce503.lib.subsystem.FFSubsystemBase;
import org.frogforce503.lib.util.LoggedTracer;

public class Arm extends FFSubsystemBase {
    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    // Constants
    private final Range motionRange = Robot.bot.getArmConfig().motionRange();
    @Setter private ArmFeedforward feedforward = Robot.bot.getArmConfig().kFF().getArmFF();

    // Control
    private double targetAngleRad = ArmConstants.START;

    private boolean shouldRunProfile = false;
    @Setter private TrapezoidProfile profile;
    @Getter private State setpoint = new State();
    private boolean atGoal = false;

    public Arm(ArmIO io) {
        this.io = io;
        profile = new TrapezoidProfile(Robot.bot.getArmConfig().kConstraints());
    }

    @Override
    public void periodic() {
        super.periodic();

        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);

        // Update profile
        if (shouldRunProfile && RobotState.isEnabled()) {
            var goalState =
                new State(
                    motionRange.clamp(targetAngleRad),
                    0.0);

            double previousVelocity = setpoint.velocity;

            setpoint =
                profile
                    .calculate(Constants.loopPeriodSecs, setpoint, goalState);

            if (!motionRange.contains(setpoint.position)) {
                setpoint =
                    new State(
                        motionRange.clamp(setpoint.position),
                        0.0);
            }

            atGoal = isAtAngle(goalState.position, ArmConstants.kTolerance);

            if (atGoal) {
                stop();
            } else {
                double accel = (setpoint.velocity - previousVelocity) / Constants.loopPeriodSecs;
                io.runPosition(setpoint.position, feedforward.calculate(setpoint.position, setpoint.velocity, accel));
            }

            // Log state
            Logger.recordOutput("Arm/Profile/SetpointPositionRad", setpoint.position);
            Logger.recordOutput("Arm/Profile/SetpointVelocityRadPerSec", setpoint.velocity);
            Logger.recordOutput("Arm/Profile/GoalPositionRad", goalState.position);
            Logger.recordOutput("Arm/AtGoal", atGoal);
        } else {
            // Reset setpoint
            setpoint = new State(getAngleRad(), 0.0);
      
            // Clear logs
            Logger.recordOutput("Arm/Profile/SetpointPositionRad", 0.0);
            Logger.recordOutput("Arm/Profile/SetpointVelocityRadPerSec", 0.0);
            Logger.recordOutput("Arm/Profile/GoalPositionRad", 0.0);
            Logger.recordOutput("Arm/AtGoal", true);
        }

        Logger.recordOutput("Arm/CurrentPositionRad", getAngleRad());

        // Record cycle time
        LoggedTracer.record("Arm");
    }

    public double getAngleRad() {
        return inputs.data.positionRad();
    }

    // Actions
    public void setPID(double kP, double kI, double kD) {
        io.setPID(kP, kI, kD);
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        io.setBrakeMode(enabled);
    }

    @Override
    public void stop() {
        io.stop();
    }

    public void runOpenLoop(double output) {
        this.shouldRunProfile = false;
        io.runOpenLoop(output);
    }

    public void setAngle(double angleRad) {
        this.shouldRunProfile = true;
        this.targetAngleRad = angleRad;
    }

    public boolean isAtAngle(double angleRad, double tolerance) {
        return MathUtil.isNear(angleRad, getAngleRad(), tolerance);
    }
}