package org.frogforce503.robot.subsystems.offsets;

public enum Direction {
    LEFT,
    RIGHT,
    FORWARD,
    BACKWARD;

    public boolean equals(String direction) {
        return this.name().equals(direction);
    }
}