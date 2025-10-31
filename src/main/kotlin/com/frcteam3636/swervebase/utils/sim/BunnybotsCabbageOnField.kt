package com.frcteam3636.swervebase.utils.sim

import com.frcteam3636.swervebase.utils.math.inches
import com.frcteam3636.swervebase.utils.math.kilograms
import edu.wpi.first.math.geometry.Pose2d
import org.dyn4j.geometry.Circle
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation.GamePieceInfo


class BunnybotsCabbageOnField(initialPose: Pose2d) : GamePieceOnFieldSimulation(BUNNYBOTS_CABBAGE_INFO, initialPose) {
}

// Numbers copied from algae.
val BUNNYBOTS_CABBAGE_INFO = GamePieceInfo(
    "Cabbage",
    Circle(0.176),
    16.inches,
    0.4.kilograms,
    1.8,
    5.0,
    0.8
)
