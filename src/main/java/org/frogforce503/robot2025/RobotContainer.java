package org.frogforce503.robot2025;

import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.frogforce503.lib.io.JoystickInputs;
import org.frogforce503.lib.util.DoublePressTracker;
import org.frogforce503.lib.util.ErrorUtil;
import org.frogforce503.lib.util.FFSelectCommand;
import org.frogforce503.lib.util.LoggedJVM;
import org.frogforce503.lib.util.TriggerUtil;
import org.frogforce503.lib.vision.apriltag_detection.VisionMeasurement;
import org.frogforce503.robot2025.auto.AutoChooser;
import org.frogforce503.robot2025.auto.WarmupExecutor;
import org.frogforce503.robot2025.commands.ClimbingCommands;
import org.frogforce503.robot2025.commands.EjectCoralOnReefAndStow;
import org.frogforce503.robot2025.commands.IntakeAlgaeFromGround;
import org.frogforce503.robot2025.commands.IntakeAlgaeFromHandoff;
import org.frogforce503.robot2025.commands.IntakeAlgaeFromReef;
import org.frogforce503.robot2025.commands.ScoreAlgaeInBarge;
import org.frogforce503.robot2025.commands.ScoreAlgaeInProcessor;
import org.frogforce503.robot2025.commands.ScoreCoralOnReef;
import org.frogforce503.robot2025.commands.SafelyStowAndIntakeCoralFromStation;
import org.frogforce503.robot2025.commands.drive.TeleopSwerveCommand;
import org.frogforce503.robot2025.subsystems.climber.Climber;
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
import org.frogforce503.robot2025.subsystems.superstructure.SuperstructureMode;
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
import org.frogforce503.robot2025.subsystems.superstructure.intakepivot.IntakePivot;
import org.frogforce503.robot2025.subsystems.superstructure.intakepivot.IntakePivotIO;
import org.frogforce503.robot2025.subsystems.superstructure.intakepivot.IntakePivotIOSim;
import org.frogforce503.robot2025.subsystems.superstructure.intakepivot.IntakePivotIOSpark;
import org.frogforce503.robot2025.subsystems.superstructure.intakeroller.IntakeRoller;
import org.frogforce503.robot2025.subsystems.superstructure.intakeroller.IntakeRollerIO;
import org.frogforce503.robot2025.subsystems.superstructure.intakeroller.IntakeRollerIOSim;
import org.frogforce503.robot2025.subsystems.superstructure.intakeroller.IntakeRollerIOSpark;
import org.frogforce503.robot2025.subsystems.superstructure.sensors.CoralSensorIOBeamBreak;
import org.frogforce503.robot2025.subsystems.superstructure.wrist.Wrist;
import org.frogforce503.robot2025.subsystems.superstructure.wrist.WristIO;
import org.frogforce503.robot2025.subsystems.superstructure.wrist.WristIOSim;
import org.frogforce503.robot2025.subsystems.superstructure.wrist.WristIOSpark;
import org.frogforce503.robot2025.subsystems.vision.Vision;
import org.frogforce503.robot2025.subsystems.vision.VisionSimulator;
import org.frogforce503.robot2025.subsystems.vision.VisionConstants.CameraName;
import org.frogforce503.robot2025.subsystems.vision.apriltag_detection.AprilTagIO;
import org.frogforce503.robot2025.subsystems.vision.apriltag_detection.AprilTagIOPhotonSim;
import org.frogforce503.robot2025.subsystems.vision.apriltag_detection.AprilTagIOPhotonVision;
import org.frogforce503.robot2025.subsystems.vision.object_detection.ObjectDetectionIO;
import org.frogforce503.test.UnitTest;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
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

    // Field Info
    private final FieldInfo field = new FieldInfo();

    // Auto
    private final AutoChooser autoChooser;

    // Sim
    private final GameViz gameViz;
    private final VisionSimulator visionViz = new VisionSimulator();

    // Controllers
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);
    private final Trigger driverLeftPaddle = driver.leftPaddle();
    private final Trigger driverRightPaddle = driver.rightPaddle();
    private final Supplier<JoystickInputs> driverInputs = () -> new JoystickInputs(driver);

    // Other
    private final WarmupExecutor warmupExecutor;

    private final Consumer<VisionMeasurement> visionEstimateConsumer =
        visionMeasurement ->
            drive.acceptVisionMeasurement(visionMeasurement);

    private final LoggedJVM loggedJVM = new LoggedJVM();

    // Overrides
    private final LoggedNetworkBoolean autoDrivingEnabled =
        new LoggedNetworkBoolean("Auto Driving Enabled", true);
    
    public RobotContainer() {
        Elevator elevator = null;
        Arm arm = null;
        Wrist wrist = null;
        Claw claw = null;
        IntakePivot intakePivot = null;
        IntakeRoller intakeRoller = null;

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
                            new AprilTagIOPhotonVision(CameraName.UPPER_FRONT_RIGHT, Robot.bot.getVisionConfig().FRONT_RIGHT_CAMERA_TO_CENTER()),
                            new AprilTagIOPhotonVision(CameraName.LOWER_FRONT_RIGHT, Robot.bot.getVisionConfig().LOWER_FRONT_RIGHT_CAMERA_TO_CENTER()),
                            new AprilTagIOPhotonVision(CameraName.ELEVATOR_BACK, Robot.bot.getVisionConfig().ELEVATOR_BACK_CAMERA_TO_CENTER())
                        },
                        new ObjectDetectionIO[] {});
                elevator = new Elevator(new ElevatorIOSpark());
                arm = new Arm(new ArmIOSpark());
                wrist = new Wrist(new WristIOSpark());
                claw = new Claw(new ClawIOSpark());
                intakePivot = new IntakePivot(new IntakePivotIOSpark());
                intakeRoller = new IntakeRoller(new IntakeRollerIOSpark());
                climber = new Climber(new ClimberIOSpark());
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
                            new AprilTagIOPhotonVision(CameraName.UPPER_FRONT_RIGHT, Robot.bot.getVisionConfig().FRONT_RIGHT_CAMERA_TO_CENTER()),
                            new AprilTagIOPhotonVision(CameraName.LOWER_FRONT_RIGHT, Robot.bot.getVisionConfig().LOWER_FRONT_RIGHT_CAMERA_TO_CENTER()),
                            new AprilTagIOPhotonVision(CameraName.ELEVATOR_BACK, Robot.bot.getVisionConfig().ELEVATOR_BACK_CAMERA_TO_CENTER())
                        },
                        new ObjectDetectionIO[] {});
                elevator = new Elevator(new ElevatorIOSpark());
                arm = new Arm(new ArmIOSpark());
                wrist = new Wrist(new WristIOSpark());
                claw = new Claw(new ClawIOSpark());
                intakePivot = new IntakePivot(new IntakePivotIOSpark());
                intakeRoller = new IntakeRoller(new IntakeRollerIOSpark());
                climber = new Climber(new ClimberIOSpark());
                leds = new Leds(new LedsIOCANdle());
            }
            case SimBot -> {
                drive = new Drive(new DriveIOBasicSim(), field);
                vision =
                    new Vision(
                        visionEstimateConsumer,
                        drive::getCurrentPose,
                        new AprilTagIO[] {
                            new AprilTagIOPhotonSim(CameraName.FRONT_LEFT, Robot.bot.getVisionConfig().FRONT_LEFT_CAMERA_TO_CENTER(), visionViz),
                            new AprilTagIOPhotonSim(CameraName.UPPER_FRONT_RIGHT, Robot.bot.getVisionConfig().FRONT_RIGHT_CAMERA_TO_CENTER(), visionViz),
                            new AprilTagIOPhotonSim(CameraName.LOWER_FRONT_RIGHT, Robot.bot.getVisionConfig().LOWER_FRONT_RIGHT_CAMERA_TO_CENTER(), visionViz),
                            new AprilTagIOPhotonSim(CameraName.ELEVATOR_BACK, Robot.bot.getVisionConfig().ELEVATOR_BACK_CAMERA_TO_CENTER(), visionViz)
                        },
                        new ObjectDetectionIO[] {});
                elevator = new Elevator(new ElevatorIOSim());
                arm = new Arm(new ArmIOSim());
                wrist = new Wrist(new WristIOSim());
                claw = new Claw(new ClawIOSim());
                intakePivot = new IntakePivot(new IntakePivotIOSim());
                intakeRoller = new IntakeRoller(new IntakeRollerIOSim());
                climber = new Climber(new ClimberIOSim());
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
                elevator = new Elevator(new ElevatorIO() {});
                arm = new Arm(new ArmIO() {});
                wrist = new Wrist(new WristIO() {});
                claw = new Claw(new ClawIO() {});
                intakePivot = new IntakePivot(new IntakePivotIO() {});
                intakeRoller = new IntakeRoller(new IntakeRollerIO() {});
                climber = new Climber(new ClimberIO() {});
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
                intakePivot,
                intakeRoller,
                new CoralSensorIOBeamBreak(),
                drive::getCurrentPose);
    
        // Create offset manager
        offsetManager =
            new OffsetManager(
                Constants.getMode() == Constants.Mode.REPLAY
                    ? new OffsetsIO() {}
                    : new OffsetsIOServer());

        // Create auto chooser
        autoChooser =
            new AutoChooser(
                drive,
                field,
                superstructure);

        // Create viz
        gameViz = new GameViz(drive, field);

        warmupExecutor = new WarmupExecutor(drive, autoChooser);

        drive.setDefaultCommand(new TeleopSwerveCommand(drive, field, driverInputs.get()));

        // Triggers
        Trigger camerasConnected = new Trigger(() -> true); // TODO: Make a method for this in Vision.java

        // If cameras disconnected for 5 seconds, then leds will blink red for the rest of the match
        camerasConnected
            .debounce(5.0, DebounceType.kBoth)
            .onFalse(
                Commands.runOnce(
                    () -> leds.setCameraDisconnected(true))
                        .ignoringDisable(true));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        driver.leftTrigger().whileTrue(
            new FFSelectCommand<>(
                Map.of(
                    SuperstructureMode.CORAL_INTAKE, new SafelyStowAndIntakeCoralFromStation(drive, field, vision, superstructure, leds),
                    SuperstructureMode.ALGAE_GROUND, new IntakeAlgaeFromGround(),
                    SuperstructureMode.ALGAE_HANDOFF, new IntakeAlgaeFromHandoff(),
                    SuperstructureMode.ALGAE_PLUCK_HIGH, new IntakeAlgaeFromReef(true),
                    SuperstructureMode.ALGAE_PLUCK_LOW, new IntakeAlgaeFromReef(false)
                ),
                superstructure::getCurrentMode));

        driver.leftBumper().whileTrue(new ScoreCoralOnReef());
        driver.rightBumper().whileTrue(new ScoreCoralOnReef());

        driver.rightTrigger().whileTrue(
            new FFSelectCommand<>(
                Map.of(
                    SuperstructureMode.L1, new EjectCoralOnReefAndStow(),
                    SuperstructureMode.L2, new EjectCoralOnReefAndStow(),
                    SuperstructureMode.L3, new EjectCoralOnReefAndStow(),
                    SuperstructureMode.L4, new EjectCoralOnReefAndStow(),
                    SuperstructureMode.PROCESSOR, new ScoreAlgaeInProcessor(),
                    SuperstructureMode.BARGE, new ScoreAlgaeInBarge()
                ),
                superstructure::getCurrentMode));

        // Preset Selection
        bindPresets(driver.y(), SuperstructureMode.L1, SuperstructureMode.BARGE, SuperstructureMode.ALGAE_PLUCK_HIGH);
        bindPresets(driver.b(), SuperstructureMode.L2, null, SuperstructureMode.ALGAE_HANDOFF);
        bindPresets(driver.a(), SuperstructureMode.L3, SuperstructureMode.PROCESSOR, SuperstructureMode.ALGAE_PLUCK_LOW);
        bindPresets(driver.x(), SuperstructureMode.L4, null, SuperstructureMode.ALGAE_GROUND);

        // Climbing commands
        bindClimbing(driver.povLeft(), ClimbingCommands.setPivotDown(superstructure)); // Step 1 of climbing
        bindClimbing(driver.povUp(), ClimbingCommands.bringPivotUp(superstructure)); // Step 2 of climbing
        
        driver.povRight()
            .onTrue(ClimbingCommands.fastWind(climber)) // Step 3 of climbing
            .onFalse(ClimbingCommands.slowWind(superstructure, climber));

        // Overrides
        driver.back().onTrue(Commands.runOnce(drive::toggleSlowMode));
        driver.start().onTrue(Commands.runOnce(drive::toggleRobotRelative));
        operator.povUp().onTrue(Commands.runOnce(drive::resetRotation));

        operator.leftTrigger().onTrue(Commands.runOnce(superstructure::seedWristPosition));
    }

    private void bindPresets(
        Trigger trigger,
        SuperstructureMode modeWithCoral,
        SuperstructureMode modeWithAlgae,
        SuperstructureMode modeWithoutAnything
    ) {
        trigger
            .onTrue(
                Commands.runOnce(() -> {
                    if (modeWithCoral != null && superstructure.isHasCoral()) {
                        superstructure.setCurrentMode(modeWithCoral);
                    } else if (modeWithAlgae != null && (superstructure.isHasAlgaeInClaw() || superstructure.isHasAlgaeInIntake())) {
                        superstructure.setCurrentMode(modeWithAlgae);
                    } else if (modeWithoutAnything != null) {
                        superstructure.setCurrentMode(modeWithoutAnything);
                    }
                })
                .ignoringDisable(true));
    }

    private void bindClimbing(Trigger trigger, Command climbCommand) {
        trigger
            .whileTrue(climbCommand)
            .whileFalse(
                Commands.runOnce(() -> {
                    superstructure.stop();
                    climber.stop();
            }));
    }

    public void robotPeriodic() {
        if (RobotBase.isSimulation()) {
            visionViz.update(drive.getCurrentPose());
        }

        loggedJVM.update();
    }

    public void autonomousInit() {
        autoChooser.startAuto();
    }

    public void teleopInit() {
        superstructure.getClaw().stop(); // Make sure coral doesn't eject in case state goes to EJECT_CORAL
        autoChooser.cleanup();
    }

    public void disabledInit() {
        if (drive.isCoastAfterAutoEnd()) {
            drive.coast(); // Coasts drivetrain in disabled mode if post-auto coasting is enabled
        }
        warmupExecutor.disabledInit();
    }

    public void disabledPeriodic() {
        autoChooser.periodic();
        superstructure.seedWristPosition();
        warmupExecutor.disabledPeriodic();
    }

    @Override
    public void test() {
        // // Uncomment to test (Waits 5 sec, auto aligns to nearest branch & does reef alignment, waits 1 sec after finished, and then homes & goes to global localization)
        // CharacterizationExecutor executor = new CharacterizationExecutor(drive);
        
        // RobotModeTriggers.teleop().onTrue(
        //     Commands.sequence(
        //         executor.wheelRadiusCharacterization().withTimeout(20)
        //     )
        // );

        // DataLogManager.start();
        // DriverStation.startDataLog(DataLogManager.getLog());

        // SysIdExecutor e = new SysIdExecutor(superstructure.getArm(), volts -> superstructure.getArm().runVolts(volts.in(Volts)));

        // RobotModeTriggers.teleop().onTrue(
        //     Commands.waitSeconds(3).alongWith(e.sysIdQuasistatic(Direction.kForward))
        // );
    }
}