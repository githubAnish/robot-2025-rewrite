package org.frogforce503.robot2025;

import org.frogforce503.robot2025.subsystems.drive.Drive;

/** Simulates the field, including interaction with & movement of game elements. Implement physics simulation here. */
public class GameViz {
    private final Drive drive;
    
    public GameViz(Drive drive) {
        this.drive = drive;
    }
}