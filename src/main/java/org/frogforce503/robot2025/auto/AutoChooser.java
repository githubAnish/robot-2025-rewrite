package org.frogforce503.robot2025.auto;

import java.util.EnumMap;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import org.frogforce503.lib.auto.builder.ChoreoFactoryBuilder;
import org.frogforce503.lib.auto.route.BaseRoute;
import org.frogforce503.lib.util.SwitchableChooser;
import org.frogforce503.robot2025.FieldInfo;
import org.frogforce503.robot2025.subsystems.drive.Drive;
import org.frogforce503.robot2025.subsystems.superstructure.Superstructure;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoChooser {
    // Requirements
    private final Drive drive;
    private final Superstructure superstructure;
    private final AutoFactory autoFactory;

    // Selectors
    private LoggedDashboardChooser<Alliance> allianceSelector = new LoggedDashboardChooser<>("Autochooser/Alliance");
    private LoggedDashboardChooser<StartingLocation> startingLocationSelector = new LoggedDashboardChooser<>("Autochooser/Starting Location");
    private SwitchableChooser routineSelector = new SwitchableChooser("Autochooser/Routine");

    // Auto Map
    private final Map<Alliance, Map<StartingLocation, Map<String, Supplier<AutoMode>>>> autoMap = new EnumMap<>(Alliance.class);

    // State
    private AutoMode auto;
    private Command autoCommand;

    private Alliance lastAlliance;
    private StartingLocation lastStartingLocation;
    private String lastRoutine;

    private final LoggedNetworkString selectedAutoDisplay = new LoggedNetworkString("/SmartDashboard/Autochooser/Selected Auto");
    private final LoggedNetworkBoolean autoReadyToRunDisplay = new LoggedNetworkBoolean("SmartDashboard/Autochooser/Ready to Run?");

    public AutoChooser(
        Drive drive,
        Superstructure superstructure
    ) {
        this.drive = drive;
        this.superstructure = superstructure;
        this.autoFactory = new ChoreoFactoryBuilder(drive).buildFactory();

        // Initialize selectors
        allianceSelector.addDefaultOption("Blue", Alliance.Blue);
        allianceSelector.addOption("Red", Alliance.Red);

        startingLocationSelector.addDefaultOption("Left", StartingLocation.Left);
        startingLocationSelector.addOption("Center", StartingLocation.Center);
        startingLocationSelector.addOption("Right", StartingLocation.Right);

        // Add auto choices below
        autoMap
            .put(Alliance.Blue, new EnumMap<>(StartingLocation.class) {{
                put(StartingLocation.Left, new HashMap<String, Supplier<AutoMode>>() {{
                    
                }});
                
                put(StartingLocation.Center, new HashMap<String, Supplier<AutoMode>>() {{
                    
                }});
                
                put(StartingLocation.Right, new HashMap<String, Supplier<AutoMode>>() {{
                    
                }});
            }});

        autoMap
            .put(Alliance.Red, new EnumMap<>(StartingLocation.class) {{
                put(StartingLocation.Left, new HashMap<String, Supplier<AutoMode>>() {{
                    
                }});
                
                put(StartingLocation.Center, new HashMap<String, Supplier<AutoMode>>() {{
                    
                }});
                
                put(StartingLocation.Right, new HashMap<String, Supplier<AutoMode>>() {{
                    
                }});
            }});
    }

    // Helper methods
    private void logTrajectory(Pose2d[] trajectory) {
        FieldInfo.getObject("Trajectory").setPoses(trajectory);
        Logger.recordOutput("Drive/Trajectory", trajectory);
    }

    // Clears existing auto & UI state
    private void reset() {
        // Clear auto & auto command
        auto = null;

        if (autoCommand != null) {
            autoCommand.cancel();
            autoCommand = null;
        }
        
        // Clear display
        selectedAutoDisplay.set("None");
        autoReadyToRunDisplay.set(false);
        logTrajectory(new Pose2d[] {});

        // Reset options
        Alliance alliance = allianceSelector.get();
        StartingLocation startingLocation = startingLocationSelector.get();

        if (alliance == null || startingLocation == null) {
            routineSelector.setOptions(new String[] {});
            return;
        }

        routineSelector.setOptions(
            autoMap
                .get(alliance)
                .get(startingLocation)
                .keySet()
                .toArray(String[]::new));
    }

    private void createAuto() {
        if (auto == null) {
            return;
        }

        final BaseRoute route = auto.getRoute();

        // Set initial pose
        drive.setPose(
            route
                .getInitialPose()
                .orElse(drive.getCurrentPose())); // Set initial pose

        // Create auto command
        Timer autoTimer = new Timer();

        autoCommand =
            Commands.sequence(
                Commands.runOnce(autoTimer::restart),
                auto.routine(),
                Commands.runOnce(() -> {
                    System.out.println("Auto " + auto.getName() + " finished in " + autoTimer.get() + " seconds."); 
                    autoTimer.stop();
                })
            );
        
        // Display Path
        Pose2d[] poses = route.getPoses().toArray(Pose2d[]::new);
        logTrajectory(poses);

        // Initialize robot starting state
        superstructure.setHasCoral(true);
    }

    // Main actions
    public void startAuto() {
        if (autoCommand != null) {
            autoCommand.schedule();
        }
    }

    public void periodic() {
        // Get selections
        Alliance alliance = allianceSelector.get();
        StartingLocation startingLocation = startingLocationSelector.get();
        String routine = routineSelector.get();

        // Prevent null during boot
        if (alliance == null || startingLocation == null) {
            return;
        }

        // Check if selections changed
        boolean allianceChanged = alliance != lastAlliance;
        boolean startingLocationChanged = startingLocation != lastStartingLocation;
        boolean routineChanged = routine != null && !routine.equals(lastRoutine);

        // Change routine chooser options if alliance or starting location changed
        if (allianceChanged || startingLocationChanged) {
            reset();
        }

        // Set field alliance if alliance changed
        if (allianceChanged) {
            FieldInfo.setAllianceOverride(alliance);
        }

        // Update auto if anything changed
        if (allianceChanged || startingLocationChanged || routineChanged) {
            var autoSupplier =
                autoMap
                    .get(allianceSelector.get())
                    .get(startingLocationSelector.get())
                    .get(routineSelector.get());

            if (autoSupplier != null) {
                if (autoCommand != null) {
                    autoCommand.cancel();
                }

                auto = autoSupplier.get();
                createAuto();
                selectedAutoDisplay.set(routineSelector.get());
                autoReadyToRunDisplay.set(true);
            }
        }

        lastAlliance = alliance;
        lastStartingLocation = startingLocation;
        lastRoutine = routine;
    }

    public void close() {
        reset();

        if (auto != null) {
            autoCommand.cancel();
        }
    }

    private enum StartingLocation {
        Left,
        Center,
        Right
    }
}