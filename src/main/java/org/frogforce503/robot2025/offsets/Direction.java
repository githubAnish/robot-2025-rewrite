package org.frogforce503.robot2025.offsets;

public enum Direction {
    LEFT,
    RIGHT,
    FORWARD,
    BACKWARD;

    public boolean equalsTo(String direction) {
        return this.name().equals(direction);
    }
}