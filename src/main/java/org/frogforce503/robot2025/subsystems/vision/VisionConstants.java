package org.frogforce503.robot2025.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    public static final Matrix<N3, N1> fixedStdDevs = VecBuilder.fill(0.8, 0.8, Units.degreesToRadians(30));   
}