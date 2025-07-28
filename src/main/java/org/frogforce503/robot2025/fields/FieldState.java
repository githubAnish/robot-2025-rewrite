package org.frogforce503.robot2025.fields;

import java.util.Set;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import lombok.Getter;
import lombok.Setter;

public class FieldState {
    @Setter @Getter private GameState gameState = GameState.DISABLED;

    private Alliance allianceColor = Alliance.Red;
    private boolean allianceColorBeenOverriden = false;

    public boolean isFMSInfoAvailable() {
        Set<String> info = NetworkTableInstance.getDefault().getTable("FMSInfo").getKeys();
        System.out.println("FMS KEYS: " + info);
        return !info.isEmpty();
    }

    public Alliance getAlliance() {
        return 
            allianceColorBeenOverriden || DriverStation.getAlliance().isEmpty()
                ? this.allianceColor
                : DriverStation.getAlliance().get();
    }

    public void overrideAllianceColor(Alliance color) {
        allianceColorBeenOverriden = true;
        this.allianceColor = color;
    }

    public enum GameState {
        DISABLED,
        AUTON,
        TELEOP,
        TEST
    }
}