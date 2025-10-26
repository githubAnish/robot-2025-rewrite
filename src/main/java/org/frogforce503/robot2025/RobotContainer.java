package org.frogforce503.robot2025;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.frogforce503.lib.io.JoystickInputs;
import org.frogforce503.lib.reefscape.Branch;
import org.frogforce503.lib.util.DoublePressTracker;
import org.frogforce503.lib.util.ErrorUtil;
import org.frogforce503.lib.util.FFSelectCommand;
import org.frogforce503.lib.util.TriConsumer;
import org.frogforce503.lib.util.TriggerUtil;
import org.frogforce503.lib.vision.apriltag_detection.VisionMeasurement;
import org.frogforce503.robot2025.auto.AutoChooser;
import org.frogforce503.robot2025.auto.AutoWarmupExecutor;
import org.frogforce503.robot2025.commands.AutoIntakeCommands;
import org.frogforce503.robot2025.commands.AutoScoreCommands;
import org.frogforce503.robot2025.commands.RumbleCommand;
import org.frogforce503.robot2025.commands.drive.TeleopSwerveCommand;
import org.frogforce503.robot2025.commands.WaitAfterAlgaeEject;
import org.frogforce503.robot2025.commands.WaitAfterCoralEject;
import org.frogforce503.robot2025.subsystems.climber.Climber;
import org.frogforce503.robot2025.subsystems.climber.Climber.ClimberGoal;
import org.frogforce503.robot2025.subsystems.climber.ClimberIO;
import org.frogforce503.robot2025.subsystems.climber.ClimberIOSim;
import org.frogforce503.robot2025.subsystems.climber.ClimberIOSpark;
import org.frogforce503.robot2025.subsystems.drive.Drive;
import org.frogforce503.robot2025.subsystems.drive.io.DriveIOPhoenix;
import org.frogforce503.robot2025.subsystems.drive.io.DriveIOBasicSim;
import org.frogforce503.robot2025.subsystems.leds.Leds;
import org.frogforce503.robot2025.subsystems.leds.LedsIO;
import org.frogforce503.robot2025.subsystems.leds.LedsIOCANdle;
import org.frogforce503.robot2025.subsystems.offsets.OffsetManager;
import org.frogforce503.robot2025.subsystems.offsets.OffsetsIO;
import org.frogforce503.robot2025.subsystems.offsets.OffsetsIOServer;
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
import org.frogforce503.robot2025.subsystems.vision.Vision;
import org.frogforce503.robot2025.subsystems.vision.VisionSimulator;
import org.frogforce503.robot2025.subsystems.vision.Vision.CameraName;
import org.frogforce503.robot2025.subsystems.vision.apriltag_detection.AprilTagIO;
import org.frogforce503.robot2025.subsystems.vision.apriltag_detection.AprilTagIOPhotonSim;
import org.frogforce503.robot2025.subsystems.vision.apriltag_detection.AprilTagIOPhotonVision;
import org.frogforce503.robot2025.subsystems.vision.object_detection.ObjectDetectionIO;
import org.frogforce503.robot2025.visualization.GameVisualizer;
import org.frogforce503.test.UnitTest;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.UnaryFunction;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
    private final OffsetManager offsetManager;

    // Field
    private final FieldInfo field = new FieldInfo();

    // Auto
    private final AutoChooser autoChooser;
    private final AutoWarmupExecutor autoWarmupExecutor;

    // Simulation
    private final GameVisualizer gameVisualizer;
    private final VisionSimulator visionVisualizer = new VisionSimulator();

    // Controllers
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);
    private final Supplier<Trigger> driverLeftPaddle = driver.leftPaddle();
    private final Supplier<Trigger> driverRightPaddle = driver.rightPaddle();
    
    private final Supplier<JoystickInputs> driverInputs = () -> new JoystickInputs(driver);

    // Triggers
    private Trigger coralMode, algaeMode;
    private Trigger manualControlEnabled, superstructureCoastEnabled;
    private Trigger camerasConnected;

    // Command Mappers
    private Map<Mode, Command> intakeRunner;
    private Map<Mode, Command> releaseIntakeRunner;
    private Map<Mode, Command> scoreRunner;
    private Map<Mode, Command> releaseScoreRunner;
  
    // Vision Estimate Acceptor
    private final Consumer<VisionMeasurement> visionEstimateConsumer =
        visionMeasurement ->    
            drive.acceptVisionMeasurement(visionMeasurement);

    // Driver-Assisted Commands
    private final LoggedNetworkBoolean autoDrivingEnabled =
        new LoggedNetworkBoolean("AutoDrivingEnabled", true);

    private final AutoIntakeCommands autoIntakeCommands;
    private final AutoScoreCommands autoScoreCommands;
    
    // Overrides
    private LoggedNetworkBoolean superstructureCoastOverride =
        new LoggedNetworkBoolean("Coast Mode/Superstructure", false);
    
    public RobotContainer() {
        field.setVenue(Constants.selectedVenue);

        Elevator elevator = null;
        Arm arm = null;
        Wrist wrist = null;
        Claw claw = null;
        Intake intake = null;
    
        // Initialize subsystems based on robot type
        switch (Constants.getRobot()) {
            case CompBot -> {
                drive = new Drive(new DriveIOPhoenix(), field);
                vision =
                    new Vision(
                        visionEstimateConsumer,
                        drive::getCurrentPose,
                        new AprilTagIO[] {
                            new AprilTagIOPhotonVision(CameraName.FRONT_LEFT, Robot.bot.getVisionConfig().FRONT_LEFT_CAMERA_TO_CENTER()),
                            new AprilTagIOPhotonVision(CameraName.UPPER_FRONT_RIGHT, Robot.bot.getVisionConfig().UPPER_FRONT_RIGHT_CAMERA_TO_CENTER()),
                            new AprilTagIOPhotonVision(CameraName.LOWER_FRONT_RIGHT, Robot.bot.getVisionConfig().LOWER_FRONT_RIGHT_CAMERA_TO_CENTER()),
                            new AprilTagIOPhotonVision(CameraName.ELEVATOR_BACK, Robot.bot.getVisionConfig().ELEVATOR_BACK_CAMERA_TO_CENTER())
                        },
                        new ObjectDetectionIO[] {});
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
                vision =
                    new Vision(
                        visionEstimateConsumer,
                        drive::getCurrentPose,
                        new AprilTagIO[] {
                            new AprilTagIOPhotonVision(CameraName.FRONT_LEFT, Robot.bot.getVisionConfig().FRONT_LEFT_CAMERA_TO_CENTER()),
                            new AprilTagIOPhotonVision(CameraName.UPPER_FRONT_RIGHT, Robot.bot.getVisionConfig().UPPER_FRONT_RIGHT_CAMERA_TO_CENTER()),
                            new AprilTagIOPhotonVision(CameraName.ELEVATOR_BACK, Robot.bot.getVisionConfig().ELEVATOR_BACK_CAMERA_TO_CENTER()),
                            new AprilTagIOPhotonVision(CameraName.ELEVATOR_FRONT, Robot.bot.getVisionConfig().ELEVATOR_FRONT_CAMERA_TO_CENTER())
                        },
                        new ObjectDetectionIO[] {});
                elevator = new Elevator(new ElevatorIOSpark(), new DigitalIOElevator());
                arm = new Arm(new ArmIOSpark());
                wrist = new Wrist(new WristIOSpark());
                claw = new Claw(new ClawIOSpark());
                intake = new Intake(new PivotIOSpark(), new RollerIOSpark());
                climber = new Climber(new ClimberIOSpark(), new DigitalIO() {});
                leds = new Leds(new LedsIOCANdle());
            }
            case SimBot -> {
                drive = new Drive(new DriveIOBasicSim(), field);
                vision =
                    new Vision(
                        visionEstimateConsumer,
                        drive::getCurrentPose,
                        new AprilTagIO[] {
                            new AprilTagIOPhotonSim(CameraName.FRONT_LEFT, Robot.bot.getVisionConfig().FRONT_LEFT_CAMERA_TO_CENTER(), visionVisualizer),
                            new AprilTagIOPhotonSim(CameraName.UPPER_FRONT_RIGHT, Robot.bot.getVisionConfig().UPPER_FRONT_RIGHT_CAMERA_TO_CENTER(), visionVisualizer),
                            new AprilTagIOPhotonSim(CameraName.LOWER_FRONT_RIGHT, Robot.bot.getVisionConfig().LOWER_FRONT_RIGHT_CAMERA_TO_CENTER(), visionVisualizer),
                            new AprilTagIOPhotonSim(CameraName.ELEVATOR_BACK, Robot.bot.getVisionConfig().ELEVATOR_BACK_CAMERA_TO_CENTER(), visionVisualizer)
                        },
                        new ObjectDetectionIO[] {});
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
                vision =
                    new Vision(
                        visionEstimateConsumer,
                        drive::getCurrentPose,
                        new AprilTagIO[] {},
                        new ObjectDetectionIO[] {});
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
                Constants.selectedVenue,
                Constants.getMode() == Constants.Mode.REPLAY
                    ? new OffsetsIO() {}
                    : new OffsetsIOServer());

        // Create auto intake commands
        autoIntakeCommands =
            new AutoIntakeCommands(
                drive,
                vision,
                superstructure,
                leds,
                field,
                offsetManager,
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
                driverInputs,
                autoDrivingEnabled::get);

        // Create auto chooser
        autoChooser =
            new AutoChooser(
                drive,
                field,
                superstructure,
                autoIntakeCommands,
                autoScoreCommands);

        autoWarmupExecutor = new AutoWarmupExecutor(autoChooser);

        // Create game visualizer
        gameVisualizer = new GameVisualizer(field, drive::getCurrentPose);
    
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
                    .coralAutoScoreL1()
                    .alongWith(leds.scoreCoral()));
            put(Mode.L2,
                autoScoreCommands
                    .coralAutoScore()
                    .alongWith(Commands.runOnce(vision::reefAlignment))
                    .alongWith(leds.scoreCoral()));
            put(Mode.L3,
                autoScoreCommands
                    .coralAutoScore()
                    .alongWith(Commands.runOnce(vision::reefAlignment))
                    .alongWith(leds.scoreCoral()));
            put(Mode.L4,
                autoScoreCommands
                    .coralAutoScore()
                    .alongWith(Commands.runOnce(vision::reefAlignment))
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
                    .andThen(() -> superstructure.setCurrentMode(Mode.CORAL_INTAKE))
                    .alongWith(Commands.runOnce(vision::globalLocalization)));
            put(Mode.L2,
                superstructure
                    .ejectCoral()
                    .andThen(new WaitAfterCoralEject())
                    .andThen(superstructure.home())
                    .andThen(() -> superstructure.setCurrentMode(Mode.CORAL_INTAKE))
                    .alongWith(Commands.runOnce(vision::globalLocalization)));
            put(Mode.L3,
                superstructure
                    .ejectCoral()
                    .andThen(new WaitAfterCoralEject())
                    .andThen(superstructure.home())
                    .andThen(() -> superstructure.setCurrentMode(Mode.CORAL_INTAKE))
                    .alongWith(Commands.runOnce(vision::globalLocalization)));
            put(Mode.L4,
                superstructure
                    .ejectCoral()
                    .andThen(new WaitAfterCoralEject())
                    .andThen(superstructure.homeAfterL4())
                    .andThen(() -> superstructure.setCurrentMode(Mode.CORAL_INTAKE))
                    .alongWith(Commands.runOnce(vision::globalLocalization)));

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
        superstructureCoastEnabled = new Trigger(superstructureCoastOverride::get);
        camerasConnected = new Trigger(() -> true); // TODO: Make a method for this in Vision.java

        // Set default mode whenever gamepiece changes
        coralMode.onTrue(Commands.runOnce(() -> superstructure.setCurrentMode(Mode.CORAL_INTAKE)));
        algaeMode.onTrue(Commands.runOnce(() -> superstructure.setCurrentMode(Mode.ALGAE_GROUND)));

        BiConsumer<Trigger, Runnable> bindSwitches =
            (trigger, runnable) ->
                trigger
                    .onTrue(Commands.runOnce(runnable));

        // Joystick drive command
        drive.setDefaultCommand(
            new TeleopSwerveCommand(
                drive,
                field,
                driverInputs.get(),
                drive::isRobotRelative,
                drive::isSlowMode));

        // Main Controls
        driver
            .leftTrigger()
                .whileTrue(intake())
                .whileFalse(releaseIntake());

        driver
            .rightTrigger()
                .whileTrue(score())
                .whileFalse(releaseScore());

        bindSwitches.accept(driver.leftBumper(), () -> superstructure.setCurrentBranch(Branch.LEFT));
        bindSwitches.accept(driver.rightBumper(), () -> superstructure.setCurrentBranch(Branch.RIGHT));

        // Preset Selection
        TriConsumer<Trigger, Gamepiece, Mode> bindSelection =
            (trigger, gamepiece, mode) ->
                trigger
                    .onTrue(
                        Commands.parallel(
                            Commands.runOnce(() -> superstructure.setCurrentPiece(gamepiece)),
                            Commands.runOnce(() -> superstructure.setCurrentMode(mode))
                        )
                        .ignoringDisable(true));

        bindSelection.accept(driver.y(), Gamepiece.CORAL, Mode.L1);
        bindSelection.accept(driver.b(), Gamepiece.CORAL, Mode.L2);
        bindSelection.accept(driver.a(), Gamepiece.CORAL, Mode.L3);
        bindSelection.accept(driver.x(), Gamepiece.CORAL, Mode.L4);

        bindSelection.accept(driver.povUp(), Gamepiece.ALGAE, Mode.ALGAE_PLUCK_HIGH);
        bindSelection.accept(driver.povDown(), Gamepiece.ALGAE, Mode.ALGAE_PLUCK_LOW);
        bindSelection.accept(driver.povLeft(), Gamepiece.ALGAE, Mode.PROCESSOR);
        bindSelection.accept(driver.povRight(), Gamepiece.ALGAE, Mode.BARGE);

        bindSelection.accept(operator.y(), Gamepiece.ALGAE, Mode.ALGAE_GROUND);
        bindSelection.accept(operator.a(), Gamepiece.ALGAE, Mode.ALGAE_HANDOFF);

        BiConsumer<Trigger, Command> bindClimbing =
            (trigger, command) ->
                trigger
                    .whileTrue(command)
                    .whileFalse(superstructure.stop().alongWith(climber.stop()));

        bindClimbing.accept(driverLeftPaddle.get(), superstructure.setPivotDown());
        bindClimbing.accept(driverRightPaddle.get(), superstructure.bringPivotUp());
        bindClimbing.accept(operator.b(), climber.runGoal(ClimberGoal.FAST_WIND));

        // Toggles / Overrides
        bindSwitches.accept(driver.back(), drive::toggleSlowMode);
        bindSwitches.accept(driver.start(), drive::toggleRobotRelative);
        bindSwitches.accept(operator.povUp(), drive::resetRotation);

        operator
            .leftTrigger()
            .onTrue(Commands.runOnce(superstructure::seedWristPosition));
        
        operator
            .back()
            .onTrue(Commands.runOnce(superstructure::toggleManualControl));

        // Coast superstructure if manual control is enabled and specified trigger is true
        Consumer<Trigger> bindSuperstructureCoast =
            trigger ->
                manualControlEnabled
                    .and(trigger)
                        .onChange(
                            Commands.runOnce(() -> superstructure.setBrakeMode(!superstructure.isBrakeModeEnabled()))
                                .ignoringDisable(true));

        bindSuperstructureCoast.accept(superstructureCoastEnabled);

        UnaryFunction limiter =
            input ->
                MathUtil.clamp(
                    MathUtil.applyDeadband(input, 0.2), -1.0, 1.0);

        TriConsumer<DoubleSupplier, DoubleSupplier, DoubleSupplier> bindManualControl =
            (elevatorPercent, armPercent, wristPercent) ->
                manualControlEnabled
                    .whileTrue(
                        Commands.parallel(
                            superstructure.manualElevatorControl(
                                limiter.apply(elevatorPercent.getAsDouble())),

                            superstructure.manualArmControl(
                                limiter.apply(armPercent.getAsDouble())),
                                    
                            superstructure.manualWristControl(
                                limiter.apply(wristPercent.getAsDouble()))));

        bindManualControl.accept(
            operator::getLeftY,
            operator::getRightY,
            operator::getRightTriggerAxis);
        
        // Leds
        BiConsumer<Trigger, Command> gotPiece =
            (trigger, command) ->
                trigger
                    .onTrue(
                        Commands.parallel(
                            command,
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

    public void updateVisualizers() {
        if (RobotBase.isSimulation()) {
            visionVisualizer.update(drive.getCurrentPose());
        }
    }

    // Main Driver Commands
    public Command intake() {
        return new FFSelectCommand<>(intakeRunner, superstructure::getCurrentMode);
    }

    public Command releaseIntake() {
        return new FFSelectCommand<>(releaseIntakeRunner, superstructure::getCurrentMode);
    }
    
    public Command score() {
        return new FFSelectCommand<>(scoreRunner, superstructure::getCurrentMode);
    }

    public Command releaseScore() {
        return new FFSelectCommand<>(releaseScoreRunner, superstructure::getCurrentMode);
    }

    public void autonomousInit() {
        autoChooser.startAuto();
    }

    public void teleopInit() {
        superstructure.stopClaw().schedule(); // Make sure coral doesn't eject in case state goes to EJECT_CORAL
        autoChooser.cleanup();
    }

    public void disabledInit() {
        if (drive.isCoastAfterAutoEnd()) {
            drive.coast(); // Coasts drivetrain in disabled mode if post-auto coasting is enabled
        }
    }

    public void disabledPeriodic() {
        autoWarmupExecutor.execute(); // Warming up autos
        autoChooser.periodic();
        superstructure.seedWristPosition();
    }

    @Override
    public void test() {
        // // Uncomment to test (Waits 5 sec, auto aligns to nearest branch & does reef alignment, waits 1 sec after finished, and then homes & goes to global localization)

        // RobotModeTriggers.teleop().onTrue(
        //     Commands.sequence(
        //         Commands.waitSeconds(5),
        //         autoScoreCommands
        //             .coralAutoScore()
        //             .alongWith(Commands.runOnce(vision::reefAlignment))
        //             .alongWith(leds.scoreCoral()),
        //         Commands.waitSeconds(1),
        //         superstructure
        //             .ejectCoral()
        //             .andThen(new WaitAfterCoralEject())
        //             .andThen(superstructure.homeAfterL4())
        //             .andThen(() -> superstructure.setCurrentMode(Mode.CORAL_INTAKE))
        //             .alongWith(Commands.runOnce(vision::globalLocalization))
        //     )
        // );
    }
}