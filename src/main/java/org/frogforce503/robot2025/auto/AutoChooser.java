package org.frogforce503.robot2025.auto;

import java.util.HashMap;
import java.util.List;
import java.util.function.Supplier;

import org.frogforce503.lib.auto.AutoFactoryBuilder;
import org.frogforce503.lib.auto.AutoMode;
import org.frogforce503.lib.util.SwitchableChooser;
import org.frogforce503.robot2025.auto.blue.BlueBabyAuton;
import org.frogforce503.robot2025.auto.red.RedBabyAuton;
import org.frogforce503.robot2025.auto.test.ChoreoWarmupAuto;
import org.frogforce503.robot2025.commands.AutoIntakeCommands;
import org.frogforce503.robot2025.commands.AutoScoreCommands;
import org.frogforce503.robot2025.fields.FieldInfo;
import org.frogforce503.robot2025.subsystems.drive.Drive;
import org.frogforce503.robot2025.subsystems.superstructure.Superstructure;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardString;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;

public class AutoChooser {
    private final Drive drive;
    private final FieldInfo field;
    private final Superstructure superstructure;

    private final AutoFactory autoFactory;

    private LoggedDashboardChooser<StartingLocation> startingSideSelector;
    private LoggedDashboardChooser<Alliance> colorSelector;
    private SwitchableChooser<String> routineChooser;

    private LoggedDashboardBoolean commitAuton;
    private LoggedDashboardString selectedAutoNameDisplay;
    private LoggedDashboardBoolean autoReadyDisplay;

    private AutoMode selectedAuto;
    private Command selectedAutoCommand;

    private Alliance lastAlliance = null;
    private StartingLocation lastStartingSide = null;
    private String lastRoutine = "";
    
    private
        HashMap<
            Alliance,
            HashMap<
                StartingLocation,
                HashMap<
                    String,
                    Supplier<AutoMode>>>> AUTO_MAP = new HashMap<>();

    public AutoChooser(
        Drive drive,
        FieldInfo field,
        Superstructure superstructure,
        AutoIntakeCommands autoIntakeCommands,
        AutoScoreCommands autoScoreCommands
    ) {
        this.drive = drive;
        this.field = field;
        this.superstructure = superstructure;

        this.autoFactory = new AutoFactoryBuilder(drive).buildFactory();

        this.colorSelector = new LoggedDashboardChooser<>("AutoChooser/Alliance Color");

        // --------- ALLIANCE COLOR AUTOMATICALLY SELECTS TO BLUE --------- //

        this.colorSelector.addDefaultOption("BLUE", Alliance.Blue);
        this.colorSelector.addOption("RED", Alliance.Red);

        // --------- ALLIANCE COLOR AUTOMATICALLY SELECTS TO RED --------- //

        // this.colorSelector.addDefaultOption("RED", Alliance.Red);
        // this.colorSelector.addOption("BLUE", Alliance.Blue);

        // --------------------------------------------------------------- //

        this.startingSideSelector = new LoggedDashboardChooser<>("AutoChooser/Starting Location");
        this.startingSideSelector.addDefaultOption("LEFT", StartingLocation.LEFT);
        this.startingSideSelector.addOption("CENTER", StartingLocation.CENTER);
        this.startingSideSelector.addOption("RIGHT", StartingLocation.RIGHT);

        this.routineChooser = new SwitchableChooser<>("AutoChooser/Routine");
        
        this.commitAuton = new LoggedDashboardBoolean("AutoChooser/Commit Auton Config", false);
        this.selectedAutoNameDisplay = new LoggedDashboardString("AutoChooser/Selected Auto Name", "NO AUTO SELECTED");
        this.autoReadyDisplay = new LoggedDashboardBoolean("AutoChooser/Ready to run??", false);

        AUTO_MAP.put(Alliance.Red,
            new HashMap<StartingLocation, HashMap<String, Supplier<AutoMode>>>() {{
                put(StartingLocation.LEFT, new HashMap<String, Supplier<AutoMode>>() {{
                    
                }});
                put(StartingLocation.CENTER, new HashMap<String, Supplier<AutoMode>>() {{
                    put("RED-BABY-AUTON",
                        () ->
                            new RedBabyAuton(
                                drive,
                                field,
                                superstructure,
                                autoFactory,
                                autoIntakeCommands,
                                autoScoreCommands));
                }});
                put(StartingLocation.RIGHT, new HashMap<String, Supplier<AutoMode>>() {{
                    
                }});
            }}
        );

        AUTO_MAP.put(Alliance.Blue,
            new HashMap<StartingLocation, HashMap<String, Supplier<AutoMode>>>() {{
                put(StartingLocation.LEFT, new HashMap<String, Supplier<AutoMode>>() {{
                    
                }});
                put(StartingLocation.CENTER, new HashMap<String, Supplier<AutoMode>>() {{
                    put("BLUE-BABY-AUTON",
                        () ->
                            new BlueBabyAuton(
                                drive,
                                field,
                                superstructure,
                                autoFactory,
                                autoIntakeCommands,
                                autoScoreCommands));
                }});
                put(StartingLocation.RIGHT, new HashMap<String, Supplier<AutoMode>>() {{
                    
                }});
            }}
        );
    }

    private void createAuto() {
        if (selectedAuto != null) {
            drive.setPose(
                selectedAuto.getStartingPose(drive::getCurrentPose));

            Timer autoTimer = new Timer();

            selectedAutoCommand =
                Commands
                    .deferredProxy(selectedAuto::routine)
                    .beforeStarting(autoTimer::restart)
                    .andThen(() -> {
                        System.out.println("Auto " + selectedAuto.getName() + " finished in " + autoTimer.get() + " seconds."); 
                        autoTimer.stop();
                    });
            
            drawPathOnField();

            if (RobotBase.isSimulation()) {
                superstructure.getMeasuredVisualizer().setupAuto();
                superstructure.getSetpointVisualizer().setupAuto();
            }

            superstructure.setHasCoral(true);
        }
    }

    public void startAuto() {
        if (selectedAutoCommand != null) {
            selectedAutoCommand.schedule();
        }
    }

    public void cleanup() {
        reset();

        if (selectedAuto != null) {
            selectedAutoCommand.cancel();
        }
    }

    public void periodic() {
        boolean setupChanged =
            startingSideSelector.get() != lastStartingSide ||
            colorSelector.get() != lastAlliance;

        boolean routineChanged =
            routineChooser.get() == null ||
            !routineChooser.get().equals(lastRoutine);
            
        if (setupChanged || routineChanged) {
            reset();
        }

        // Select alliance color only when in simulation, else use DriverStation app to choose
        if (RobotBase.isSimulation()) {
            field.overrideAllianceColor(colorSelector.get());
        }
        
        if (commitAuton.get()) {
            System.out.println("Commit Button Pressed - Returned from AutoChooser.java");

            StartingLocation side = startingSideSelector.get();
            Alliance color = colorSelector.get();

            var choice =
                AUTO_MAP
                    .get(color)
                    .get(side)
                    .get(routineChooser.get());

            if (choice != null) {
                AutoMode selected = choice.get();
                selectedAuto = selected;
                createAuto();
                
                selectedAutoNameDisplay.set(routineChooser.get());
                autoReadyDisplay.set(true);
            }

            commitAuton.set(false);
        }

        lastStartingSide = startingSideSelector.get();
        lastAlliance = colorSelector.get();
        lastRoutine = routineChooser.get();
    }

    private void reset() {
        selectedAuto = null;
        autoReadyDisplay.set(false);
        selectedAutoNameDisplay.set("NO AUTO SELECTED");

        field
            .getObject("Trajectory")
            .setPoses(new Pose2d[] {});

        routineChooser.setOptions(
            AUTO_MAP
                .get(colorSelector.get())
                .get(startingSideSelector.get())
                .keySet()
                .toArray(String[]::new));
    }

    /** Warms up the auto chooser by running a test Choreo path for 5 seconds on robotInit. */
    public void scheduleWarmupCommand() {
        new ChoreoWarmupAuto(drive, field, autoFactory)
            .routine()
                .withTimeout(5)
                .andThen(new PrintCommand("Warmup Auto Finished"))
                .ignoringDisable(true)
                .withName("Warmup Choreo Autos")
            .schedule();
    }

    private void drawPathOnField() {
        List<Pose2d> poses =
            selectedAuto
                .getRoute()
                .getPoses();

        String pointsList = "[";

        int i = 0;
        int granularity = 25;
        
        for (Pose2d pose : poses) {
            if (i % granularity == 0) {
                if (i != 0) {
                    pointsList += ", ";
                }
                pointsList += "[" + ((int) (pose.getX() / 100)) + ", " + (int) (pose.getY() / 100) + "]";
            }
            i++;
        }

        pointsList += "]";

        field
            .getObject("Trajectory")
            .setPoses(poses.toArray(Pose2d[]::new));

        Logger.recordOutput("Swerve/SelectedAuto", pointsList);
    }

    public enum StartingLocation {
        LEFT,
        CENTER,
        RIGHT
    }
}