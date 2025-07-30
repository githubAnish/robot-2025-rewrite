package org.frogforce503.lib.control.pidf;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;

public record PIDFConfig(
        double kP,
        double kI,
        double kD,
        double kS,
        double kG,
        double kV,
        double kA
) {
        /** Sets PID and extracts feedforward values from an {@code ArmFeedforward}. */
        public PIDFConfig(double kP, double kI, double kD, ArmFeedforward feedforward) {
                this(kP, kI, kD, feedforward.getKs(), feedforward.getKg(), feedforward.getKv(), feedforward.getKa());
        }

        /** Sets PID and extracts feedforward values from an {@code ElevatorFeedforward}. */
        public PIDFConfig(double kP, double kI, double kD, ElevatorFeedforward feedforward) {
                this(kP, kI, kD, feedforward.getKs(), feedforward.getKg(), feedforward.getKv(), feedforward.getKa());
        }

        /** Sets PID and zero feedfoward. */
        public PIDFConfig(double kP, double kI, double kD) {
                this(kP, kI, kD, 0.0, 0.0, 0.0, 0.0);
        }
}