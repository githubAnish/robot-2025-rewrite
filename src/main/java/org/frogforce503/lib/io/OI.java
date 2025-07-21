package org.frogforce503.lib.io;

import org.frogforce503.robot2025.commands.DriveCommands;
import org.frogforce503.robot2025.subsystems.drive.Drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;

/** Non-command based wrapper class to handle joystick bindings. Used by Frog Force 503 before 2024. */
public class OI {
    private final Drive drive;

    private GenericHID driver = new GenericHID(0);
    private GenericHID operator = new GenericHID(1);
    private GenericHID overrideController = new GenericHID(2);

    private final int buttonA = 1;
    private final int buttonB = 2;
    private final int buttonX = 3;
    private final int buttonY = 4;
    private final int buttonLB = 5;
    private final int buttonRB = 6;
    private final int buttonSelect = 7;
    private final int buttonMenu = 8;
    private final int buttonLeftJoystick = 9;
    private final int buttonRightJoystick = 10;

    public OI(Drive drive) {
        this.drive = drive;

        configureButtonBindings();
    }

    public void configureButtonBindings() {
        // Zero gyro
        whenPressed(driver, buttonB, drive::resetGyro); // put back
    }

    public void whileHeld(GenericHID controller, int button, Runnable whileHeld) {
        if (controller.getRawButton(button)) {
            whileHeld.run();
        }
    }

    public void whenPressed(GenericHID controller, int button, Runnable onPress) {
        if (controller.getRawButtonPressed(button)) {
            onPress.run();
        }
    }

    public void whenReleased(GenericHID controller, int button, Runnable onRelease) {
        if (controller.getRawButtonReleased(button)) {
            onRelease.run();
        }
    }

    public void whenTriggerPressed(boolean current, boolean last, Runnable onPress) {
        if (!last && current) {
            onPress.run();
        }
    }

    public void whenTriggerReleased(boolean current, boolean last, Runnable onRelease) {
        if (last && !current) {
            onRelease.run();
        }
    }

    public void whileTriggerHeld(boolean current,  Runnable whileHeld) {
        if (current) {
            whileHeld.run();
        }
    }

    public void whenPressedAndReleased(GenericHID controller, int button, Runnable onPress, Runnable whenReleased) {
        whenPressed(controller, button, onPress);
        whenReleased(controller, button, whenReleased);
    }
    

    public void whenPressedHeldAndReleased(GenericHID controller, int button, Runnable onPress, Runnable whileHeld, Runnable whenReleased) {
        whenPressed(controller, button, onPress);
        whileHeld(controller, button, whileHeld);
        whenReleased(controller, button, whenReleased);
    }

    public void whenTriggerPressedAndReleased(boolean current, boolean last, Runnable whenPressed, Runnable whenRelesaed) {
        whenTriggerPressed(current, last, whenPressed);
        whenTriggerReleased(current, last, whenRelesaed);
    }


    public void whenTriggerPressedHeldAndReleased(boolean current, boolean last, Runnable whenPressed, Runnable whileHeld, Runnable whenRelesaed) {
        whenTriggerPressed(current, last, whenPressed);
        whileTriggerHeld(current, whileHeld);
        whenTriggerReleased(current, last, whenRelesaed);
    }

    public void whileHeldAndReleased(GenericHID controller, int button, Runnable whileHeld, Runnable whenReleased) {
        whileHeld(controller, button, whileHeld);
        whenReleased(controller, button, whenReleased);
    }

    public boolean getButton(GenericHID controller, int button) {
        return controller.getRawButton(button);
    }

    public double getDriverLeftYValue() {
        return driver.getRawAxis(1);
    }

    public double getDriverLeftXValue() {
        return driver.getRawAxis(0);
    }

    public double getDriverRightYValue() {
        return driver.getRawAxis(5);
    }

    public double getDriverRightXValue() {
        return driver.getRawAxis(4);
    }

    public boolean getDriverLeftTrigger() {
        return driver.getRawAxis(2) >= 0.2;
    }

    public boolean getDriverRightTrigger() {
        return driver.getRawAxis(3) >= 0.2;
    }

    public boolean getOverrideRightTrigger() {
        return overrideController.getRawAxis(3) >= 0.5;
    }

    public double getOperatorLeftYValue() {
        return overrideController.getRawAxis(1);
    }

    public double getOperatorLeftXValue() {
        return overrideController.getRawAxis(0);
    }

    public boolean driverXpressed() {
        return getButton(driver, buttonX);
    }

    public boolean driverYpressed() {
        return getButton(driver, buttonY);
    }

    public boolean tryingToDrive() {
        return tryingToTranslate() || tryingToTurn();
    }
    
    public boolean tryingToTranslate() {
        return new Translation2d(getDriverLeftXValue(), getDriverLeftYValue()).getNorm() >= DriveCommands.DEADBAND;
    }

    public boolean tryingToTurn() {
        return Math.abs(getDriverRightXValue()) > DriveCommands.DEADBAND;
    }

    public void flushOICache() {
        for (int i = 0; i <= 2; i++) {
            GenericHID controller = new GenericHID(i);

            for (int j = 1; j <= 12; j++) {
                controller.getRawButton(j);
            }
        }
    }
}