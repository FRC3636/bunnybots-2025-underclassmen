package com.frcteam3636.swervebase.utils.sim

import com.frcteam3636.swervebase.utils.math.grams
import com.frcteam3636.swervebase.utils.math.inches
import edu.wpi.first.math.geometry.Pose2d
import org.dyn4j.geometry.Rectangle
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation.GamePieceInfo

class BunnybotsCarrotOnField(initialPose: Pose2d) : GamePieceOnFieldSimulation(BUNNYBOTS_CARROT_INFO, initialPose) {
}

val BUNNYBOTS_CARROT_INFO = GamePieceInfo(
    "Carrot",
    Rectangle(0.0635, 0.18034),
    2.5.inches,
    7.6.grams,
    5.6,
    8.0,
    0.1
)