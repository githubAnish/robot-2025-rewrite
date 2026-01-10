package org.frogforce503.robot.subsystems.superstructure;

import java.util.function.Supplier;

import org.frogforce503.lib.logging.LoggedTracer;
import org.frogforce503.lib.math.MathUtils;
import org.frogforce503.lib.subsystem.VirtualSubsystem;
import org.frogforce503.robot.subsystems.superstructure.arm.Arm;
import org.frogforce503.robot.subsystems.superstructure.arm.ArmConstants;
import org.frogforce503.robot.subsystems.superstructure.claw.Claw;
import org.frogforce503.robot.subsystems.superstructure.elevator.Elevator;
import org.frogforce503.robot.subsystems.superstructure.intakepivot.IntakePivot;
import org.frogforce503.robot.subsystems.superstructure.intakeroller.IntakeRoller;
import org.frogforce503.robot.subsystems.superstructure.sensors.CoralSensorIO;
import org.frogforce503.robot.subsystems.superstructure.wrist.Wrist;
import org.frogforce503.robot.subsystems.superstructure.sensors.CoralSensorIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import lombok.Getter;
import lombok.Setter;

/** Wrapper class for {@link Elevator}, {@link Arm}, {@link Wrist}, {@link Claw}, {@link IntakePivot}, {@link IntakeRoller}, and {@link CoralSensorIO} */
public class Superstructure extends VirtualSubsystem {
    // Subsystems
    @Getter private final Elevator elevator;
    @Getter private final Arm arm;
    @Getter private final Wrist wrist;
    @Getter private final Claw claw;
    @Getter private final IntakePivot intakePivot;
    @Getter private final IntakeRoller intakeRoller;

    private final CoralSensorIO coralSensorIO;
    private final CoralSensorIOInputsAutoLogged coralSensorInputs = new CoralSensorIOInputsAutoLogged();

    // Inputs
    @Setter @Getter private boolean hasCoral;
    @Setter @Getter private boolean hasAlgaeInClaw;
    @Setter @Getter private boolean hasAlgaeInIntake;

    @Setter @Getter private SuperstructureMode currentMode = SuperstructureMode.CORAL_INTAKE;

    // Viz
    @Getter private final SuperstructureViz viz;

    // Overrides
    private LoggedNetworkBoolean superstructureCoastOverride =
        new LoggedNetworkBoolean("Coast Mode/Superstructure", false);

    private boolean inCoast = false;

    public Superstructure(
        Elevator elevator,
        Arm arm,
        Wrist wrist,
        Claw claw,
        IntakePivot intakePivot,
        IntakeRoller intakeRoller,
        CoralSensorIO coralSensorIO,
        Supplier<Pose2d> robotPoseSupplier
    ) {
        this.elevator = elevator;
        this.arm = arm;
        this.wrist = wrist;
        this.claw = claw;
        this.intakePivot = intakePivot;
        this.intakeRoller = intakeRoller;
        this.coralSensorIO = coralSensorIO;

        this.viz = new SuperstructureViz(this, robotPoseSupplier);
    }

    @Override
    public void periodic() {
        coralSensorIO.updateInputs(coralSensorInputs);
        Logger.processInputs("CoralSensors", coralSensorInputs);

        boolean shouldCoast = superstructureCoastOverride.get();
        
        if (RobotState.isDisabled() && shouldCoast != inCoast) {
            inCoast = shouldCoast;
            setCoastMode(shouldCoast);
        }

        // Update viz
        if (RobotBase.isSimulation()) {
            viz.update(
                elevator.getHeightMeters(),
                arm.getAngleRad(),
                wrist.getRelativeAngleRad(),
                intakePivot.getAngleRad(),
                hasCoral,
                hasAlgaeInClaw,
                hasAlgaeInIntake);
        }

        Logger.recordOutput("Superstructure/Inputs/Has Coral", hasCoral);
        Logger.recordOutput("Superstructure/Inputs/Algae In Claw", hasAlgaeInClaw);
        Logger.recordOutput("Superstructure/Inputs/Algae In Intake", hasAlgaeInIntake);

        Logger.recordOutput("Superstructure/Mode", currentMode);

        // Record cycle time
        LoggedTracer.record("Superstructure");
    }

    public boolean upperBeamTriggered() {
        return coralSensorInputs.data.upperBeamBreakTriggered();
    }

    public boolean lowerBeamTriggered() {
        return coralSensorInputs.data.lowerBeamBreakTriggered();
    }

    // Actions
    public void setCoastMode(boolean enabled) {
        elevator.getCoastOverride().set(enabled);
        arm.getCoastOverride().set(enabled);
        wrist.getCoastOverride().set(enabled);
        claw.getCoastOverride().set(enabled);
        intakePivot.getCoastOverride().set(enabled);
        intakeRoller.getCoastOverride().set(enabled);
    }

    public void seedWristPosition() { // Only works in real anyway, so doesn't affect sim
        if (RobotBase.isReal() &&
            MathUtils.inRange(arm.getAngleRad(), ArmConstants.START, Units.degreesToRadians(-60)) &&
            MathUtils.inRange(wrist.getAbsoluteAngleRad(), Units.degreesToRadians(-90), Units.degreesToRadians(90))
        ) {
            wrist.setRelativeEncoderPosition((arm.getAngleRad() + Math.PI / 2) + wrist.getAbsoluteAngleRad()); // Look at math in robot-2025, it should make sense
        }
    }

    public void stop() {
        elevator.stop();
        arm.stop();
        wrist.stop();
        claw.stop();
        intakePivot.stop();
        intakeRoller.stop();
    }
}