package org.frogforce503.robot2025.commands.gamepiece_eject;

import edu.wpi.first.wpilibj2.command.WaitCommand;

/** Waits 0.5 (by default) seconds after ejecting coral to home the superstructure. */
public class WaitAfterAlgaeEject extends WaitCommand {
    /** Use this constructor if the default 0.5 seconds is what is wanted. */
    public WaitAfterAlgaeEject() {
        super(0.5);
    }

    /** Use this constructor if the default 0.5 seconds isn't what is wanted. */
    public WaitAfterAlgaeEject(double seconds) {
        super(seconds);
    }

    public double getDefaultSeconds() {
        return 0.5;
    }
}
