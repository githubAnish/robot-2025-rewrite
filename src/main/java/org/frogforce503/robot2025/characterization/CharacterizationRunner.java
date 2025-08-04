package org.frogforce503.robot2025.characterization;

import static edu.wpi.first.units.Units.Volts;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;

import org.frogforce503.robot2025.subsystems.drive.Drive;
import org.frogforce503.robot2025.subsystems.drive.DriveConstants;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class CharacterizationRunner {
    private final double FF_START_DELAY = 2.0; // Secs
    private final double FF_RAMP_RATE = 0.1; // Volts/Sec
    private final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
    private final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

    private final Drive drive;

    public CharacterizationRunner(Drive drive) {
        this.drive = drive;
    }

    private SysIdRoutine generateNewRoutine(
        Velocity<VoltageUnit> rampRate,
        Voltage stepVoltage,
        Time timeout
    ) {
        return
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    rampRate,
                    stepVoltage,
                    timeout,
                    state ->
                        Logger.recordOutput("Swerve/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                    voltage ->
                        drive.runCharacterization(voltage.in(Volts)),
                    null, // No log consumer, since data is recorded by AdvantageKit
                    drive));
    }
    
    /** Returns a command to run a quasistatic test in the specified direction. */
    public Command sysIdQuasistatic(
        Velocity<VoltageUnit> rampRate,
        Voltage stepVoltage,
        Time timeout,
        SysIdRoutine.Direction direction
    ) {
        SysIdRoutine routine =
            generateNewRoutine(rampRate, stepVoltage, timeout);

        return
            Commands.run(() -> drive.runCharacterization(0.0))
                .withTimeout(1.0)
                .andThen(routine.quasistatic(direction));
    }
    
    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdDynamic(
        Velocity<VoltageUnit> rampRate,
        Voltage stepVoltage,
        Time timeout,
        SysIdRoutine.Direction direction
    ) {
        SysIdRoutine routine =
            generateNewRoutine(rampRate, stepVoltage, timeout);

        return
            Commands.run(() -> drive.runCharacterization(0.0))
                .withTimeout(1.0)
                .andThen(routine.dynamic(direction));
    }

    /**
     * Measures the velocity feedforward constants for the drive motors.
     *
     * <p>This command should only be used in voltage control mode.
     */
    public Command feedforwardCharacterization() {
        List<Double> velocitySamples = new LinkedList<>();
        List<Double> voltageSamples = new LinkedList<>();
        Timer timer = new Timer();

        return Commands.sequence(
            // Reset data
            Commands.runOnce(
                () -> {
                    velocitySamples.clear();
                    voltageSamples.clear();
                }),

            // Allow modules to orient
            Commands.run(() -> drive.runCharacterization(0.0), drive).withTimeout(FF_START_DELAY),

            // Start timer
            Commands.runOnce(timer::restart),

            // Accelerate and gather data
            Commands.run(
                () -> {
                    double voltage = timer.get() * FF_RAMP_RATE;
                    drive.runCharacterization(voltage);
                    velocitySamples.add(drive.getFFCharacterizationVelocity());
                    voltageSamples.add(voltage);
                },
                drive)

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                        int n = velocitySamples.size();
                        double sumX = 0.0;
                        double sumY = 0.0;
                        double sumXY = 0.0;
                        double sumX2 = 0.0;
                        for (int i = 0; i < n; i++) {
                            sumX += velocitySamples.get(i);
                            sumY += voltageSamples.get(i);
                            sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                            sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                        }
                        double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                        double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                        NumberFormat formatter = new DecimalFormat("#0.00000");
                        System.out.println("********** Drive FF Characterization Results **********");
                        System.out.println("\tkS: " + formatter.format(kS));
                        System.out.println("\tkV: " + formatter.format(kV));
                    }));
    }

    /** Measures the robot's wheel radius by spinning in a circle. */
    public Command wheelRadiusCharacterization() {
        SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
        WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

        return Commands.parallel(
            // Drive control sequence
            Commands.sequence(
                // Reset acceleration limiter
                Commands.runOnce(() -> limiter.reset(0.0)),

                // Turn in place, accelerating up to full speed
                Commands.run(
                    () -> {
                        double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                        drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                    },
                    drive)),

            // Measurement sequence
            Commands.sequence(
                // Wait for modules to fully orient before starting measurement
                Commands.waitSeconds(1.0),

                // Record starting measurement
                Commands.runOnce(
                    () -> {
                        state.positions = drive.getWheelRadiusCharacterizationPositions();
                        state.lastAngle = drive.getGyroRotation();
                        state.gyroDelta = 0.0;
                    }),

                // Update gyro delta
                Commands.run(
                    () -> {
                        var rotation = drive.getGyroRotation();
                        state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                        state.lastAngle = rotation;

                        double[] positions = drive.getWheelRadiusCharacterizationPositions();
                        double wheelDelta = 0.0;
                        for (int i = 0; i < 4; i++) {
                            wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                        }
                        double wheelRadius =
                            (state.gyroDelta * DriveConstants.driveBaseRadius) / wheelDelta;

                        Logger.recordOutput("Swerve/WheelDelta", wheelDelta);
                        Logger.recordOutput("Swerve/WheelRadius", wheelRadius);
                    })

                    // When cancelled, calculate and print results
                    .finallyDo(
                        () -> {
                            double[] positions = drive.getWheelRadiusCharacterizationPositions();
                            double wheelDelta = 0.0;
                            for (int i = 0; i < 4; i++) {
                                wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                            }
                            double wheelRadius =
                                (state.gyroDelta * DriveConstants.driveBaseRadius) / wheelDelta;

                            NumberFormat formatter = new DecimalFormat("#0.000000000000000000000000000");
                            System.out.println(
                                "********** Wheel Radius Characterization Results **********");
                            System.out.println(
                                "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                            System.out.println(
                                "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                            System.out.println(
                                "\tWheel Radius: "
                                    + formatter.format(wheelRadius)
                                    + " meters, "
                                    + formatter.format(Units.metersToInches(wheelRadius))
                                    + " inches");
                        })));
    }

    private class WheelRadiusCharacterizationState {
        double[] positions = new double[4];
        Rotation2d lastAngle = Rotation2d.kZero;
        double gyroDelta = 0.0;
    }
}