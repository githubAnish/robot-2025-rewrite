package org.frogforce503.robot2025;

import org.frogforce503.robot2025.subsystems.drive.Drive;

/** Simulates the field, including interaction with & movement of game elements. Implement physics simulation here. */
public class GameViz {
    private final Drive drive;
    private final FieldInfo field;
    
    public GameViz(Drive drive, FieldInfo field) {
        this.drive = drive;
        this.field = field;
    }
}