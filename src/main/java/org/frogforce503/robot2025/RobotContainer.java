package org.frogforce503.robot2025;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import org.frogforce503.lib.commands.RumbleCommand;
import org.frogforce503.lib.io.JoystickInputs;
import org.frogforce503.lib.util.DoublePressTracker;
import org.frogforce503.lib.util.ErrorUtil;
import org.frogforce503.lib.util.Logic;
import org.frogforce503.lib.util.ProximityService;
import org.frogforce503.lib.util.TriConsumer;
import org.frogforce503.lib.util.TriggerUtil;
import org.frogforce503.robot2025.auto.AutoChooser;
import org.frogforce503.robot2025.commands.AutoIntakeCommands;
import org.frogforce503.robot2025.commands.AutoScoreCommands;
import org.frogforce503.robot2025.commands.DriveCommands;
import org.frogforce503.robot2025.commands.coral_score_reef.Branch;
import org.frogforce503.robot2025.commands.gamepiece_eject.WaitAfterAlgaeEject;
import org.frogforce503.robot2025.commands.gamepiece_eject.WaitAfterCoralEject;
import org.frogforce503.robot2025.fields.FieldInfo;
import org.frogforce503.robot2025.fields.FieldConfig.Venue;
import org.frogforce503.robot2025.offsets.OffsetManager;
import org.frogforce503.robot2025.offsets.OffsetsIO;
import org.frogforce503.robot2025.offsets.OffsetsIOServer;
import org.frogforce503.robot2025.subsystems.climber.Climber;
import org.frogforce503.robot2025.subsystems.climber.Climber.ClimberGoal;
import org.frogforce503.robot2025.subsystems.climber.ClimberIO;
import org.frogforce503.robot2025.subsystems.climber.ClimberIOSim;
import org.frogforce503.robot2025.subsystems.climber.ClimberIOSpark;
import org.frogforce503.robot2025.subsystems.drive.Drive;
import org.frogforce503.robot2025.subsystems.drive.io.DriveIOPhoenix;
import org.frogforce503.robot2025.subsystems.drive.io.DriveIOSim;
import org.frogforce503.robot2025.subsystems.leds.Leds;
import org.frogforce503.robot2025.subsystems.leds.LedsIO;
import org.frogforce503.robot2025.subsystems.leds.LedsIOCANdle;
import org.frogforce503.robot2025.subsystems.superstructure.Superstructure;
import org.frogforce503.robot2025.subsystems.superstructure.Superstructure.Gamepiece;
import org.frogforce503.robot2025.subsystems.superstructure.Superstructure.Mode;
import org.frogforce503.robot2025.subsystems.superstructure.arm.Arm;
import org.frogforce503.robot2025.subsystems.superstructure.arm.ArmIO;
import org.frogforce503.robot2025.subsystems.superstructure.arm.ArmIOSim;
import org.frogforce503.robot2025.subsystems.superstructure.arm.ArmIOSpark;
import org.frogforce503.robot2025.subsystems.superstructure.claw.Claw;
import org.frogforce503.robot2025.subsystems.superstructure.claw.ClawIO;
import org.frogforce503.robot2025.subsystems.superstructure.claw.ClawIOSim;
import org.frogforce503.robot2025.subsystems.superstructure.claw.ClawIOSpark;
import org.frogforce503.robot2025.subsystems.superstructure.elevator.Elevator;
import org.frogforce503.robot2025.subsystems.superstructure.elevator.ElevatorIO;
import org.frogforce503.robot2025.subsystems.superstructure.elevator.ElevatorIOSim;
import org.frogforce503.robot2025.subsystems.superstructure.elevator.ElevatorIOSpark;
import org.frogforce503.robot2025.subsystems.superstructure.intake.Intake;
import org.frogforce503.robot2025.subsystems.superstructure.intake.pivot.PivotIO;
import org.frogforce503.robot2025.subsystems.superstructure.intake.pivot.PivotIOSim;
import org.frogforce503.robot2025.subsystems.superstructure.intake.pivot.PivotIOSpark;
import org.frogforce503.robot2025.subsystems.superstructure.intake.roller.RollerIO;
import org.frogforce503.robot2025.subsystems.superstructure.intake.roller.RollerIOSim;
import org.frogforce503.robot2025.subsystems.superstructure.intake.roller.RollerIOSpark;
import org.frogforce503.robot2025.subsystems.superstructure.sensors.CoralSensorIOBeamBreak;
import org.frogforce503.robot2025.subsystems.superstructure.sensors.DigitalIO;
import org.frogforce503.robot2025.subsystems.superstructure.sensors.DigitalIOClimber;
import org.frogforce503.robot2025.subsystems.superstructure.sensors.DigitalIOElevator;
import org.frogforce503.robot2025.subsystems.superstructure.wrist.Wrist;
import org.frogforce503.robot2025.subsystems.superstructure.wrist.WristIO;
import org.frogforce503.robot2025.subsystems.superstructure.wrist.WristIOSim;
import org.frogforce503.robot2025.subsystems.superstructure.wrist.WristIOSpark;
import org.frogforce503.robot2025.subsystems.vision.Camera;
import org.frogforce503.robot2025.subsystems.vision.Vision;
import org.frogforce503.test.UnitTest;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.experimental.ExtensionMethod;

@ExtensionMethod({DoublePressTracker.class, TriggerUtil.class})
public class RobotContainer implements UnitTest {
    // Subsystems
    private Drive drive;
    private Vision vision;
    private final Superstructure superstructure;
    private Climber climber;
    private Leds leds;

    // Field
    private final FieldInfo field = new FieldInfo();

    // Dashboard Inputs
    private final AutoChooser autoChooser;
    private final BooleanSupplier selectAllianceFromDS = () -> true;

    // Offset Manager
    private final OffsetManager offsetManager;

    // Controllers
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);
    private final Supplier<JoystickInputs> driverInputs = () -> new JoystickInputs(driver);

    // Triggers
    private Trigger coralMode, algaeMode;
    private Trigger manualControlEnabled;
    private Trigger camerasConnected;

    // Command Mappers
    private Map<Mode, Command> intakeRunner;
    private Map<Mode, Command> releaseIntakeRunner;
    private Map<Mode, Command> scoreRunner;
    private Map<Mode, Command> releaseScoreRunner;
  
    // Vision Estimate Acceptor
    private final Consumer<EstimatedRobotPose> visionEstimateConsumer =
        estimatedPose ->
            drive.acceptVisionMeasurement(estimatedPose);

    // Driver-Assisted Commands
    private final LoggedNetworkBoolean autoDrivingEnabled =
        new LoggedNetworkBoolean("AutoDrivingEnabled", true);

    private final ProximityService proximityService;

    private final AutoIntakeCommands autoIntakeCommands;
    private final AutoScoreCommands autoScoreCommands;
    
    public RobotContainer(final Venue venue) {
        field.setVenue(venue);

        Elevator elevator = null;
        Arm arm = null;
        Wrist wrist = null;
        Claw claw = null;
        Intake intake = null;
    
        // Initialize subsystems based on robot type
        switch (Constants.getRobot()) {
            case CompBot -> {
                drive = new Drive(new DriveIOPhoenix(), field);
                vision = new Vision(field, visionEstimateConsumer, drive::getCurrentPose);
                elevator = new Elevator(new ElevatorIOSpark(), new DigitalIOElevator());
                arm = new Arm(new ArmIOSpark());
                wrist = new Wrist(new WristIOSpark());
                claw = new Claw(new ClawIOSpark());
                intake = new Intake(new PivotIOSpark(), new RollerIOSpark());
                climber = new Climber(new ClimberIOSpark(), new DigitalIOClimber());
                leds = new Leds(new LedsIOCANdle());
            }
            case PracticeBot -> {
                drive = new Drive(new DriveIOPhoenix(), field);
                vision = new Vision(field, visionEstimateConsumer, drive::getCurrentPose);
                elevator = new Elevator(new ElevatorIOSpark(), new DigitalIOElevator());
                arm = new Arm(new ArmIOSpark());
                wrist = new Wrist(new WristIOSpark());
                claw = new Claw(new ClawIOSpark());
                intake = new Intake(new PivotIOSpark(), new RollerIOSpark());
                climber = new Climber(new ClimberIOSpark(), new DigitalIO() {});
                leds = new Leds(new LedsIOCANdle());
            }
            case SimBot -> {
                drive = new Drive(new DriveIOSim(), field);
                vision = new Vision(field, visionEstimateConsumer, drive::getCurrentPose);
                elevator = new Elevator(new ElevatorIOSim(), new DigitalIO() {});
                arm = new Arm(new ArmIOSim());
                wrist = new Wrist(new WristIOSim());
                claw = new Claw(new ClawIOSim());
                intake = new Intake(new PivotIOSim(), new RollerIOSim());
                climber = new Climber(new ClimberIOSim(), new DigitalIO() {});
                leds = new Leds(new LedsIO() {});
            }
            case ProgrammingBot -> {
                drive = new Drive(new DriveIOPhoenix(), field);
                vision = new Vision(field, visionEstimateConsumer, drive::getCurrentPose);
                elevator = new Elevator(new ElevatorIO() {}, new DigitalIO() {});
                arm = new Arm(new ArmIO() {});
                wrist = new Wrist(new WristIO() {});
                claw = new Claw(new ClawIO() {});
                intake = new Intake(new PivotIO() {}, new RollerIO() {});
                climber = new Climber(new ClimberIO() {}, new DigitalIO() {});
                leds = new Leds(new LedsIO() {});
            }
            default -> {
                System.err.println("What happened here?" + ErrorUtil.attachJavaClassName(RobotContainer.class));
            }
        }
    
        // Create superstructure
        superstructure =
            new Superstructure(
                elevator,
                arm,
                wrist,
                claw,
                intake,
                new CoralSensorIOBeamBreak(),
                drive::getCurrentPose);
    
        // Create offset manager
        offsetManager =
            new OffsetManager(
                venue,
                Constants.getMode() == Constants.Mode.REPLAY
                    ? new OffsetsIO() {}
                    : new OffsetsIOServer());

        // Create proximity service
        proximityService = new ProximityService(drive, field);

        // Create auto intake commands
        autoIntakeCommands =
            new AutoIntakeCommands(
                drive,
                vision,
                superstructure,
                leds,
                field,
                offsetManager,
                proximityService,
                driverInputs,
                autoDrivingEnabled::get);

        // Create auto score commands
        autoScoreCommands =
            new AutoScoreCommands(
                drive,
                vision,
                superstructure,
                leds,
                field,
                offsetManager,
                proximityService,
                driverInputs,
                autoDrivingEnabled::get);

        // Create auto chooser
        autoChooser =
            new AutoChooser(
                drive,
                field,
                superstructure,
                autoIntakeCommands,
                autoScoreCommands,
                selectAllianceFromDS);
    
        // Initialize command mappers
        this.intakeRunner = new HashMap<>() {{
            // Coral
            put(Mode.CORAL_INTAKE,
                autoIntakeCommands
                    .coralAutoIntake()
                    .alongWith(leds.intakeCoral()));

            // Algae
            put(Mode.ALGAE_GROUND,
                superstructure
                    .intakeAlgaeFromGround()
                    .alongWith(leds.intakeAlgae()));
            put(Mode.ALGAE_HANDOFF,
                superstructure
                    .intakeAlgaeFromHandoff()
                    .alongWith(leds.intakeAlgae()));
            put(Mode.ALGAE_PLUCK_HIGH,
                autoIntakeCommands
                    .algaeAutoHighPluck()
                    .alongWith(leds.intakeAlgae()));
            put(Mode.ALGAE_PLUCK_LOW,
                autoIntakeCommands
                    .algaeAutoLowPluck()
                    .alongWith(leds.intakeAlgae()));
        }};

        this.releaseIntakeRunner = new HashMap<>() {{
            // Coral
            put(Mode.CORAL_INTAKE, Commands.none());

            // Algae
            put(Mode.ALGAE_GROUND, superstructure.holdAlgaeFromGround());
            put(Mode.ALGAE_HANDOFF, superstructure.holdAlgaeFromHandoff());
            put(Mode.ALGAE_PLUCK_HIGH,
                autoIntakeCommands  
                    .algaeBackup()
                    .andThen(superstructure.holdAlgaeFromPluck()));
            put(Mode.ALGAE_PLUCK_LOW,
                autoIntakeCommands
                    .algaeBackup()
                    .andThen(superstructure.holdAlgaeFromPluck()));
        }};

        this.scoreRunner = new HashMap<>() {{
            // Coral
            put(Mode.L1,
                autoScoreCommands
                    .coralAutoScoreL1(() -> Mode.L1)
                    .alongWith(leds.scoreCoral()));
            put(Mode.L2,
                autoScoreCommands
                    .coralAutoScore(() -> Mode.L2)
                    .alongWith(leds.scoreCoral()));
            put(Mode.L3,
                autoScoreCommands
                    .coralAutoScore(() -> Mode.L3)
                    .alongWith(leds.scoreCoral()));
            put(Mode.L4,
                autoScoreCommands
                    .coralAutoScore(() -> Mode.L4)
                    .alongWith(leds.scoreCoral()));

            // Algae
            put(Mode.PROCESSOR,
                autoScoreCommands
                    .processorAutoScore()
                    .alongWith(leds.scoreAlgae()));
            put(Mode.BARGE,
                autoScoreCommands
                    .bargeAutoScore()
                    .alongWith(leds.scoreAlgae()));
        }};

        this.releaseScoreRunner = new HashMap<>() {{
            // Coral
            put(Mode.L1,
                superstructure
                    .ejectCoralForL1()
                    .andThen(new WaitAfterCoralEject())
                    .andThen(superstructure.home())
                    .andThen(() -> superstructure.setCurrentMode(Mode.CORAL_INTAKE)));
            put(Mode.L2,
                superstructure
                    .ejectCoral()
                    .andThen(new WaitAfterCoralEject())
                    .andThen(superstructure.home())
                    .andThen(() -> superstructure.setCurrentMode(Mode.CORAL_INTAKE)));
            put(Mode.L3,
                superstructure
                    .ejectCoral()
                    .andThen(new WaitAfterCoralEject())
                    .andThen(superstructure.home())
                    .andThen(() -> superstructure.setCurrentMode(Mode.CORAL_INTAKE)));
            put(Mode.L4,
                superstructure
                    .ejectCoral()
                    .andThen(new WaitAfterCoralEject())
                    .andThen(superstructure.homeAfterL4())
                    .andThen(() -> superstructure.setCurrentMode(Mode.CORAL_INTAKE)));

            // Algae
            put(Mode.PROCESSOR,
                new WaitAfterAlgaeEject()
                    .andThen(superstructure.home()));
            put(Mode.BARGE,
                superstructure
                    .ejectAlgaeFromClaw()
                    .andThen(new WaitAfterAlgaeEject())
                    .andThen(superstructure.home()));
        }};

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // Triggers
        coralMode = new Trigger(() -> superstructure.getCurrentPiece() == Gamepiece.CORAL);
        algaeMode = new Trigger(() -> superstructure.getCurrentPiece() == Gamepiece.ALGAE);
        manualControlEnabled = new Trigger(superstructure::isManualControlEnabled);
        camerasConnected =
            new Trigger(
                Logic.and(
                    () ->
                        Arrays
                            .stream(Camera.values())
                            .allMatch(vision::getCameraStatus),
                    vision::areTagsVisible)); // Cameras connected only when all cameras are connected & any tags are visible

        // Set default mode whenever gamepiece changes
        coralMode.onTrue(Commands.runOnce(() -> superstructure.setCurrentMode(Mode.CORAL_INTAKE)));
        algaeMode.onTrue(Commands.runOnce(() -> superstructure.setCurrentMode(Mode.ALGAE_GROUND)));

        // Driver Toggles / Overrides
        BiConsumer<Trigger, Runnable> bindDriverSwitches =
            (trigger, runnable) ->
                trigger
                    .onTrue(Commands.runOnce(runnable));

        bindDriverSwitches.accept(driver.povUp(), drive::resetRotation);
        bindDriverSwitches.accept(driver.back(), drive::toggleSlowMode);
        bindDriverSwitches.accept(driver.start(), drive::toggleRobotRelative);

        // Joystick drive command
        Supplier<Command> joystickDriveCommandFactory =
            () -> DriveCommands.joystickDrive(drive, field, driverInputs.get(), drive::isRobotRelative);

        drive.setDefaultCommand(joystickDriveCommandFactory.get());

        // Main Controls
        driver
            .leftTrigger()
                .whileTrue(intake())
                .whileFalse(releaseIntake());

        driver
            .rightTrigger()
                .whileTrue(score())
                .whileFalse(releaseScore());

        bindDriverSwitches.accept(driver.leftBumper(), () -> superstructure.setCurrentBranch(Branch.LEFT));
        bindDriverSwitches.accept(driver.rightBumper(), () -> superstructure.setCurrentBranch(Branch.RIGHT));

        // Preset Selection
        operator
            .start()
            .onTrue(Commands.runOnce(superstructure::setPiece));

        BiConsumer<Trigger, Mode> bindOperatorCoralSelection =
            (trigger, mode) ->
                trigger
                    .and(coralMode)
                    .onTrue(
                        Commands.runOnce(() -> superstructure.setCurrentMode(mode)));

        bindOperatorCoralSelection.accept(operator.y(), Mode.L1);
        bindOperatorCoralSelection.accept(operator.b(), Mode.L2);
        bindOperatorCoralSelection.accept(operator.a(), Mode.L3);
        bindOperatorCoralSelection.accept(operator.x(), Mode.L4);

        BiConsumer<Trigger, Mode> bindOperatorAlgaeSelection =
            (trigger, mode) ->
                trigger
                    .and(algaeMode)
                    .onTrue(
                        Commands.runOnce(() -> superstructure.setCurrentMode(mode)));

        bindOperatorAlgaeSelection.accept(operator.y(), Mode.ALGAE_PLUCK_HIGH);
        bindOperatorAlgaeSelection.accept(operator.a(), Mode.ALGAE_PLUCK_LOW);
        bindOperatorAlgaeSelection.accept(operator.povUp(), Mode.ALGAE_GROUND);
        bindOperatorAlgaeSelection.accept(operator.povDown(), Mode.ALGAE_HANDOFF);
        bindOperatorAlgaeSelection.accept(operator.povLeft(), Mode.PROCESSOR);
        bindOperatorAlgaeSelection.accept(operator.povRight(), Mode.BARGE);

        BiConsumer<Trigger, Command> bindOperatorClimbing =
            (trigger, command) ->
                trigger
                    .whileTrue(command)
                    .whileFalse(superstructure.stop().alongWith(climber.stop()));

        bindOperatorClimbing.accept(operator.leftBumper(), superstructure.setPivotDown());
        bindOperatorClimbing.accept(operator.rightBumper(), superstructure.bringPivotUp());
        bindOperatorClimbing.accept(operator.rightTrigger(), climber.runGoal(ClimberGoal.FAST_WIND));

        // Overrides
        operator
            .leftTrigger()
            .onTrue(Commands.runOnce(this::seedWristPosition));
        
        operator
            .back()
            .onTrue(Commands.runOnce(superstructure::toggleManualControl));

        Function<Double, Double> limiter =
            input ->
                MathUtil.clamp(
                    MathUtil.applyDeadband(input, 0.2), -1.0, 1.0);

        TriConsumer<DoubleSupplier, DoubleSupplier, DoubleSupplier> bindOperatorManualControl =
            (elevatorPercent, armPercent, wristPercent) ->
                manualControlEnabled.whileTrue(
                    Commands.parallel(
                        superstructure.manualElevatorControl(
                            limiter.apply(elevatorPercent.getAsDouble())),

                        superstructure.manualArmControl(
                            limiter.apply(armPercent.getAsDouble())),
                                
                        superstructure.manualWristControl(
                            limiter.apply(wristPercent.getAsDouble()))));

        bindOperatorManualControl.accept(
            operator::getLeftY,
            operator::getRightY,
            operator::getRightTriggerAxis);

        // Coast superstructure only if the robot is disabled, manual control is enabled, and specified button is true
        BiConsumer<Trigger, BooleanConsumer> bindSuperstructureCoast =
            (trigger, brakeModeSetter) ->
                RobotModeTriggers
                    .disabled()
                    .and(manualControlEnabled)
                    .and(trigger)
                        .onTrue(
                            Commands.runOnce(() -> brakeModeSetter.accept(false))
                                .ignoringDisable(true))
                        .onFalse(
                            Commands.runOnce(() -> brakeModeSetter.accept(true))
                                .ignoringDisable(true));

        bindSuperstructureCoast.accept(operator.start(), superstructure::setBrakeMode);
        
        // Leds
        BiConsumer<Trigger, Command> gotPiece =
            (trigger, command) ->
                trigger
                    .onTrue(
                        command
                            .alongWith(
                                new RumbleCommand(driver)));

        Trigger gotCoral = new Trigger(superstructure::isHasCoral);
        Trigger gotAlgaeInClaw = new Trigger(superstructure::isHasAlgaeInClaw);
        Trigger gotAlgaeInIntake = new Trigger(superstructure::isHasAlgaeInIntake);
        
        // Signal when robot intook a gamepiece
        gotPiece.accept(gotCoral, leds.gotCoral());
        gotPiece.accept(gotAlgaeInClaw.or(gotAlgaeInIntake), leds.gotAlgae());

        // Signal when gamepiece selected changes
        coralMode.onTrue(leds.signalCoralMode());
        algaeMode.onTrue(leds.signalAlgaeMode());

        // If cameras disconnected for 5 seconds, then leds will blink red for the rest of the match
        camerasConnected
            .debounce(5.0, DebounceType.kBoth)
            .onFalse(
                Commands.runOnce(
                    () -> leds.setCameraDisconnected(true))
                        .ignoringDisable(true));
    }

    // Main Driver Commands
    public Command intake() {
        return Commands.select(intakeRunner, superstructure::getCurrentMode);
    }

    public Command releaseIntake() {
        return Commands.select(releaseIntakeRunner, superstructure::getCurrentMode);
    }
    
    public Command score() {
        return Commands.select(scoreRunner, superstructure::getCurrentMode);
    }

    public Command releaseScore() {
        return Commands.select(releaseScoreRunner, superstructure::getCurrentMode);
    }

    // Auto Chooser
    public void warmupAutoChooser() {
        autoChooser.scheduleWarmupCommand();
    }

    public void startAuto() {
        autoChooser.startAuto();
    }

    public void updateAutoChooser() {
        autoChooser.periodic();
    }

    public void cleanupAutoChooser() {
        autoChooser.cleanup();
    }

    // Other
    public void seedWristPosition() {
        superstructure.seedWristPosition();
    }

    public void stopClaw() {
        superstructure.stopClaw().schedule();
    }

    /** Coasts drivetrain in disabled mode if post-auto coasting is enabled. */
    public void coastAfterAutoEnd() {
        if (RobotState.isDisabled() && drive.isCoastAfterAutoEnd()) {
            drive.coast();
        }
    }

    @Override
    public void test() {
        RobotModeTriggers
            .teleop()
            .onTrue(
                Commands.none()
            );
    }
}