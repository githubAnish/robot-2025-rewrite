package org.frogforce503.robot2025.auto;

import java.util.List;
import java.util.function.Supplier;

import org.frogforce503.lib.auto.builder.ChoreoFactoryBuilder;
import org.frogforce503.lib.util.ErrorUtil;
import org.frogforce503.lib.util.SwitchableChooser;
import org.frogforce503.robot2025.Constants;
import org.frogforce503.robot2025.FieldInfo;
import org.frogforce503.robot2025.auto.AutoMap.StartingLocation;
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
import lombok.Getter;

public class AutoChooser {
    private final Drive drive;
    private final FieldInfo field;
    private final Superstructure superstructure;

    private final AutoFactory autoFactory;

    private LoggedDashboardChooser<Alliance> colorSelector;
    private LoggedDashboardChooser<StartingLocation> startingSideSelector;
    private SwitchableChooser<String> routineChooser;

    private LoggedDashboardBoolean commitAuton;
    private LoggedDashboardString selectedAutoNameDisplay;
    private LoggedDashboardBoolean autoReadyDisplay;

    private AutoMode selectedAuto;
    private Command selectedAutoCommand;

    private Alliance lastAlliance = null;
    private StartingLocation lastStartingSide = null;
    private String lastRoutine = "";

    @Getter private AutoMap autoMap = new AutoMap();

    private final AutoWarmupExecutor autoWarmupExecutor = new AutoWarmupExecutor(this);

    public AutoChooser(
        Drive drive,
        FieldInfo field,
        Superstructure superstructure
    ) {
        this.drive = drive;
        this.field = field;
        this.superstructure = superstructure;

        this.autoFactory = new ChoreoFactoryBuilder(drive).buildFactory();

        this.colorSelector = new LoggedDashboardChooser<>("AutoChooser/Alliance Color");
        this.colorSelector.addDefaultOption("Red", Alliance.Red);
        this.colorSelector.addOption("Blue", Alliance.Blue);

        this.startingSideSelector = new LoggedDashboardChooser<>("AutoChooser/Starting Location");
        this.startingSideSelector.addDefaultOption("LEFT", StartingLocation.LEFT);
        this.startingSideSelector.addOption("CENTER", StartingLocation.CENTER);
        this.startingSideSelector.addOption("RIGHT", StartingLocation.RIGHT);

        this.routineChooser = new SwitchableChooser<>("AutoChooser/Routine");
        
        this.commitAuton = new LoggedDashboardBoolean("AutoChooser/Commit Auton Config", false);
        this.selectedAutoNameDisplay = new LoggedDashboardString("AutoChooser/Selected Auto Name", "NO AUTO SELECTED");
        this.autoReadyDisplay = new LoggedDashboardBoolean("AutoChooser/Ready to run??", false);
    }

    private void reset() {
        selectedAuto = null;
        autoReadyDisplay.set(false);
        selectedAutoNameDisplay.set("NO AUTO SELECTED");

        field.getObject("Trajectory").setPoses(new Pose2d[] {});
        Logger.recordOutput("Drive/Trajectory", new Pose2d[] {});

        routineChooser.setOptions(
            autoMap
                .get(colorSelector.get())
                .get(startingSideSelector.get())
                .keySet()
                .toArray(String[]::new));
    }

    private void createAuto() {
        if (selectedAuto != null) {
            drive.setPose(
                selectedAuto.getStartingPose(drive::getCurrentPose));

            Timer autoTimer = new Timer();

            // Create auto command
            selectedAutoCommand =
                selectedAuto
                    .routine()
                    .beforeStarting(autoTimer::restart)
                    .andThen(() -> {
                        System.out.println("Auto " + selectedAuto.getName() + " finished in " + autoTimer.get() + " seconds."); 
                        autoTimer.stop();
                    });
            
            // Display Path
            List<Pose2d> poses = selectedAuto.getRoute().getPoses();
    
            field.getObject("Trajectory").setPoses(poses);
            Logger.recordOutput("Drive/Trajectory", poses.toArray(Pose2d[]::new));

            // Initialize superstructure starting state
            if (RobotBase.isSimulation()) {
                superstructure.getVisualizer().setupAuto();
            }

            superstructure.setHasCoral(true);
        }
    }

    public void startAuto() {
        if (selectedAutoCommand != null) {
            selectedAutoCommand.schedule();
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

        // Select alliance color only when in simulation, else use DriverStation app to choose (only when this feature is enabled)
        if (RobotBase.isSimulation() && Constants.selectAllianceFromDS) {
            field.setAlliance(colorSelector.get());
        }
        
        if (commitAuton.get()) {
            System.out.println("Commit Button Pressed" + ErrorUtil.attachJavaClassName(AutoChooser.class));

            Alliance color = field.getAlliance();
            StartingLocation side = startingSideSelector.get();
            Supplier<AutoMode> choice = autoMap.getAuto(color, side, routineChooser.get());

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

    public void cleanup() {
        reset();

        if (selectedAuto != null) {
            selectedAutoCommand.cancel();
        }
    }
}