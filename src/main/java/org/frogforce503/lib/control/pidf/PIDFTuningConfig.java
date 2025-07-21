package org.frogforce503.lib.control.pidf;

import org.frogforce503.lib.util.LoggedTunableNumber;

public record PIDFTuningConfig(
        LoggedTunableNumber kP,
        LoggedTunableNumber kI,
        LoggedTunableNumber kD,
        LoggedTunableNumber kS,
        LoggedTunableNumber kG,
        LoggedTunableNumber kV,
        LoggedTunableNumber kA) {}