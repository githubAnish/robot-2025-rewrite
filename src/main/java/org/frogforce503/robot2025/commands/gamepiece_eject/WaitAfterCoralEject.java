package org.frogforce503.robot2025.commands.gamepiece_eject;

import edu.wpi.first.wpilibj2.command.WaitCommand;

/** Waits 0.25 (by default) seconds after ejecting coral to home the superstructure. */
public class WaitAfterCoralEject extends WaitCommand {
    /** Use this constructor if the default 0.25 seconds is what is wanted. */
    public WaitAfterCoralEject() {
        super(0.25);
    }

    /** Use this constructor if the default 0.25 seconds isn't what is wanted. */
    public WaitAfterCoralEject(double seconds) {
        super(seconds);
    }

    public double getDefaultSeconds() {
        return 0.25;
    }
}
