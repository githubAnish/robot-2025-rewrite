package org.frogforce503.lib.auto.follower;

import edu.wpi.first.math.controller.PIDController;
import lombok.Builder;

@Builder
public record AutoPIDController(
    PIDController autoXController,
    PIDController autoYController,
    PIDController autoThetaController) {}