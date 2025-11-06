package org.frogforce503.lib.subsystem;

import org.frogforce503.lib.logging.LoggerUtil;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;

/** A thin wrapper around WPILib's {@link SubsystemBase} class to create command-based subsystems with common FF boilerplate methods. */
public abstract class FFSubsystemBase extends SubsystemBase {
    @Getter protected LoggedNetworkBoolean coastOverride =
        new LoggedNetworkBoolean("Coast Mode/" + this.getName(), false);

    protected final Alert coastModeWhileRunning =
        new Alert(this.getName() + " is in coast mode while running!", AlertType.kError);

    @Override
    public void periodic() {
        LoggerUtil.recordCurrentCommand(this);

        // Set coast mode
        if (DriverStation.isDisabled()) {
            setBrakeMode(!coastOverride.get());
        }

        coastModeWhileRunning
            .set(coastOverride.get() && !DriverStation.isDisabled());
    };

    /** Override this method to apply to your tunable subsystem. */
    protected BooleanConsumer tuningExecutor() {
        return tuningEnabled -> {};
    };

    protected abstract void setBrakeMode(boolean enabled);

    public abstract void stop();
}