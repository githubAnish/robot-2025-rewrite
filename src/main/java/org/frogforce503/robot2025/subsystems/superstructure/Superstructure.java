package org.frogforce503.robot2025.subsystems.superstructure;

import java.util.function.Supplier;

import org.frogforce503.lib.math.MathUtils;
import org.frogforce503.lib.util.LoggedTracer;
import org.frogforce503.robot2025.subsystems.superstructure.arm.Arm;
import org.frogforce503.robot2025.subsystems.superstructure.claw.Claw;
import org.frogforce503.robot2025.subsystems.superstructure.elevator.Elevator;
import org.frogforce503.robot2025.subsystems.superstructure.intake.Intake;
import org.frogforce503.robot2025.subsystems.superstructure.sensors.CoralSensorIO;
import org.frogforce503.robot2025.subsystems.superstructure.sensors.CoralSensorIOInputsAutoLogged;
import org.frogforce503.robot2025.subsystems.superstructure.wrist.Wrist;
import org.frogforce503.robot2025.visualization.SuperstructureVisualizer;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.Setter;

/** Wrapper class for {@link Elevator}, {@link Arm}, {@link Wrist}, {@link Claw}, {@link Intake}, and {@link CoralSensorIO} */
public class Superstructure extends SubsystemBase {
    // Subsystems
    @Getter private final Elevator elevator;
    @Getter private final Arm arm;
    @Getter private final Wrist wrist;
    @Getter private final Claw claw;
    @Getter private final Intake intake;

    private final CoralSensorIO coralSensorIO;
    private final CoralSensorIOInputsAutoLogged coralSensorInputs = new CoralSensorIOInputsAutoLogged();

    // Inputs
    @Setter @Getter private boolean hasCoral;
    @Setter @Getter private boolean hasAlgaeInClaw;
    @Setter @Getter private boolean hasAlgaeInIntake;

    @Setter @Getter private SuperstructureMode currentMode = SuperstructureMode.CORAL_INTAKE;

    // Visualizer
    @Getter private final SuperstructureVisualizer visualizer;

    // Overrides
    private LoggedNetworkBoolean superstructureCoastOverride =
        new LoggedNetworkBoolean("Coast Mode/Superstructure", false);

    public Superstructure(
        Elevator elevator,
        Arm arm,
        Wrist wrist,
        Claw claw,
        Intake intake,
        CoralSensorIO coralSensorIO,
        Supplier<Pose2d> robotPoseSupplier
    ) {
        this.elevator = elevator;
        this.arm = arm;
        this.wrist = wrist;
        this.claw = claw;
        this.intake = intake;
        this.coralSensorIO = coralSensorIO;

        this.visualizer = new SuperstructureVisualizer(this, robotPoseSupplier);
    }

    @Override
    public void periodic() {
        coralSensorIO.updateInputs(coralSensorInputs);
        Logger.processInputs("CoralSensors", coralSensorInputs);

        setCoastMode(
            DriverStation.isDisabled() &&
            superstructureCoastOverride.get());

        // Update visualizer
        if (RobotBase.isSimulation()) {
            visualizer.update(
                elevator.getHeight(),
                arm.getAngle(),
                wrist.getRelativeAngle(),
                intake.getPivotAngle());
        }

        Logger.recordOutput("Superstructure/Inputs/Has Coral", hasCoral);
        Logger.recordOutput("Superstructure/Inputs/Algae In Claw", hasAlgaeInClaw);
        Logger.recordOutput("Superstructure/Inputs/Algae In Intake", hasAlgaeInIntake);

        Logger.recordOutput("Superstructure/Mode", currentMode);

        // Record cycle time
        LoggedTracer.record("Superstructure");
    }

    public boolean upperBeamTriggered() {
        return coralSensorInputs.data.upperTriggered();
    }

    public boolean lowerBeamTriggered() {
        return coralSensorInputs.data.lowerTriggered();
    }

    public void seedWristPosition() {
        if (RobotBase.isReal() &&
            MathUtils.inRange(arm.getAngle(), 0, 30) &&
            MathUtils.inRange(wrist.getAbsoluteAngle(), 0, 180)
        ) {
            wrist.setEncoderPosition(
                arm.getAngle() + wrist.getAbsoluteAngle());
        }
    }

    public void setCoastMode(boolean enabled) {
        elevator.getCoastOverride().set(enabled);
        arm.getCoastOverride().set(enabled);
        wrist.getCoastOverride().set(enabled);
        claw.getCoastOverride().set(enabled);
        intake.getCoastOverride().set(enabled);
    }

    public void stop() {
        elevator.stop();
        arm.stop();
        wrist.stop();
        claw.stop();
        intake.stop();
    }
}