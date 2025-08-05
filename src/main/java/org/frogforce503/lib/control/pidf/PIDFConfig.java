package org.frogforce503.lib.control.pidf;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

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

        public SimpleMotorFeedforward toSimpleMotorFeedforward() {
                return new SimpleMotorFeedforward(kS(), kV(), kA());
        }

        public ArmFeedforward toArmFeedforward() {
                return new ArmFeedforward(kS(), kG(), kV(), kA());
        }

        public ElevatorFeedforward toElevatorFeedforward() {
                return new ElevatorFeedforward(kS(), kG(), kV(), kA());
        }
}