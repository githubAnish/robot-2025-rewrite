package org.frogforce503.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class ScoreCoralOnReef extends Command {
    public ScoreCoralOnReef() {}

    // from fieldconstnats branch, the scoring location is .plus(GeomUtil.toTransform2d(Units.inchesToMeters(20.5), 0)) plus the offset manager offsets
    // more precisely 20.6158288496
}