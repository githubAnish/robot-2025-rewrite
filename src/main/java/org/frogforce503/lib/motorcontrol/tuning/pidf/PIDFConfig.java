package org.frogforce503.lib.motorcontrol.tuning.pidf;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/**
 * Configuration class for PIDF (Proportional, Integral, Derivative, Feedforward) control gains.
 * This class provides a unified way to store and manage PIDF parameters, including support for
 * extracting feedforward values from WPILib's feedforward classes such as {@link SimpleMotorFeedforward},
 * {@link ArmFeedforward}, and {@link ElevatorFeedforward}.
 * 
 * <p>It includes multiple constructors for initializing PIDF values with or without feedforward
 * components, as well as utility methods to convert the stored values back into feedforward objects.
 * 
 * <p>Usage examples:
 * <ul>
 *   <li>Initialize with specific PIDF values: {@code new PIDFConfig(1.0, 0.0, 0.1, 0.5, 0.0, 2.0, 0.1)}</li>
 *   <li>Extract values from a {@link SimpleMotorFeedforward} object: {@code new PIDFConfig(1.0, 0.0, 0.1, feedforward)}</li>
 *   <li>Convert back to a feedforward object: {@code config.toSimpleMotorFeedforward()}</li>
 * </ul>
 */
public record PIDFConfig(
        double kP,
        double kI,
        double kD,
        double kS,
        double kG,
        double kV,
        double kA
) {
        /** Sets PID and extracts feedforward values from a {@link SimpleMotorFeedforward}. */
        public PIDFConfig(double kP, double kI, double kD, SimpleMotorFeedforward feedforward) {
                this(kP, kI, kD, feedforward.getKs(), 0.0, feedforward.getKv(), feedforward.getKa());
        }

        /** Sets PID and extracts feedforward values from an {@link ArmFeedforward}. */
        public PIDFConfig(double kP, double kI, double kD, ArmFeedforward feedforward) {
                this(kP, kI, kD, feedforward.getKs(), feedforward.getKg(), feedforward.getKv(), feedforward.getKa());
        }

        /** Sets PID and extracts feedforward values from an {@link ElevatorFeedforward}. */
        public PIDFConfig(double kP, double kI, double kD, ElevatorFeedforward feedforward) {
                this(kP, kI, kD, feedforward.getKs(), feedforward.getKg(), feedforward.getKv(), feedforward.getKa());
        }

        /** Sets PID and zero feedfoward. */
        public PIDFConfig(double kP, double kI, double kD) {
                this(kP, kI, kD, 0.0, 0.0, 0.0, 0.0);
        }

        /** Sets zero PID and zero feedforward. */
        public PIDFConfig() {
                this(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        }

        public SimpleMotorFeedforward getSimpleMotorFF() {
                return new SimpleMotorFeedforward(kS(), kV(), kA());
        }

        public ArmFeedforward toArmFF() {
                return new ArmFeedforward(kS(), kG(), kV(), kA());
        }

        public ElevatorFeedforward toElevatorFF() {
                return new ElevatorFeedforward(kS(), kG(), kV(), kA());
        }

        public PIDController toPIDController() {
                return new PIDController(kP(), kI(), kD());
        }
}