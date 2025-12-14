package org.frogforce503.robot2025.auto;

import java.util.List;

import org.frogforce503.lib.auto.choreo.AutoFactoryConfigurator;
import org.frogforce503.lib.reefscape.ProximityUtil;
import org.frogforce503.robot2025.FieldInfo;
import org.frogforce503.robot2025.subsystems.drive.Drive;
import org.frogforce503.robot2025.subsystems.superstructure.Superstructure;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoChooser {
    // Requirements
    private final Drive drive;
    private final Superstructure superstructure;
    private final AutoFactory autoFactory;

    // Dashboard
    private final LoggedDashboardChooser<AutoMode> routineChooser = new LoggedDashboardChooser<>("Auto");

    // State
    private Command autoCommand;
    private AutoMode lastSelectedAuto;

    public AutoChooser(Drive drive, Superstructure superstructure) {
        this.drive = drive;
        this.superstructure = superstructure;

        this.autoFactory = AutoFactoryConfigurator.configureChoreo(drive);
        AutoFactoryConfigurator.configurePathPlanner(drive);

        configureAutos();
    }

    private void configureAutos() {
        routineChooser.addDefaultOption("Test", null);
    }

    private void logTrajectory(Pose2d... trajectory) {
        FieldInfo.getObject("Trajectory").setPoses(trajectory);
        Logger.recordOutput("Drive/Trajectory", trajectory);
    }

    // Public methods
    public void startAuto() {
        final AutoMode selectedAuto = routineChooser.get();

        if (selectedAuto == null) {
            return;
        }

        autoCommand = selectedAuto.getCommand();

        if (autoCommand != null) {
            autoCommand.schedule();   
        }
    }

    public void periodic() {
        final AutoMode selectedAuto = routineChooser.get();

        if (selectedAuto == null) {
            logTrajectory(); // Clear poses
        } else if (selectedAuto != lastSelectedAuto) {
            List<Pose2d> trajPoses = selectedAuto.getPoses();
            Pose2d start = trajPoses.get(0);

            logTrajectory(trajPoses.toArray(Pose2d[]::new));
        
            // Reset pose if drive close to trajectory start
            if (ProximityUtil.getDistanceFromPose(drive, start) <= Units.inchesToMeters(6)) {
                drive.setPose(start);
            }
        }

        lastSelectedAuto = selectedAuto;
    }

    public void close() {
        if (autoCommand != null) {
            autoCommand.cancel();
        }
    }
}