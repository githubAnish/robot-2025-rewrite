package org.frogforce503.robot2025.subsystems.offsets;

public enum Direction {
    LEFT,
    RIGHT,
    FORWARD,
    BACKWARD;

    public boolean equals(String direction) {
        return this.name().equals(direction);
    }
}