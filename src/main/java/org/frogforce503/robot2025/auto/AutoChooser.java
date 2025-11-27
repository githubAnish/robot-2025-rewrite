package org.frogforce503.robot2025.auto;

import org.frogforce503.lib.auto.builder.ChoreoFactoryBuilder;
import org.frogforce503.robot2025.FieldInfo;
import org.frogforce503.robot2025.subsystems.drive.Drive;
import org.frogforce503.robot2025.subsystems.superstructure.Superstructure;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoChooser {
    // Requirements
    private final Drive drive;
    private final FieldInfo field;
    private final Superstructure superstructure;
    private final AutoFactory autoFactory;

    // State
    private LoggedDashboardChooser<AutoMode> selector = new LoggedDashboardChooser<>("Auto Mode");
    private AutoMode auto;
    private Command autoCommand;

    public AutoChooser(
        Drive drive,
        FieldInfo field,
        Superstructure superstructure
    ) {
        this.drive = drive;
        this.field = field;
        this.superstructure = superstructure;
        this.autoFactory = new ChoreoFactoryBuilder(drive).buildFactory();

        // Add auto choices below
    }

    private void logTrajectory(Pose2d[] trajectory) {
        field.getObject("Trajectory").setPoses(trajectory);
        Logger.recordOutput("Drive/Trajectory", trajectory);
    }

    public void startAuto() {
        if (autoCommand != null) {
            autoCommand.schedule();
        }
    }

    public void periodic() {
        auto = selector.get();
        
        if (auto != null) {
            drive.setPose(auto.getRoute().getInitialPoseOrElseGet(drive::getCurrentPose)); // Set initial pose

            Timer autoTimer = new Timer();

            // Create auto command
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
            Pose2d[] poses = auto.getRoute().getPoses().toArray(Pose2d[]::new);
            logTrajectory(poses);

            // Initialize superstructure starting state
            superstructure.setHasCoral(true);
        }
    }

    public void cleanup() {
        auto = null;
        logTrajectory(new Pose2d[] {});

        if (auto != null) {
            autoCommand.cancel();
        }
    }
}