package com.frcteam3636.swervebase.utils.sim

import com.frcteam3636.swervebase.utils.math.degrees
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import org.ironmaple.simulation.SimulatedArena

// From the perspective of https://docs.advantagescope.org/assets/images/coordinate-system-blue-wall-cb4489de5e0472f383794a0a0ee5d3c5.png
val BOTTOM_LEFT_CORNER = Translation2d(0.0, 0.0)
val BOTTOM_RIGHT_CORNER = Translation2d(17.548, 0.0)
val TOP_LEFT_CORNER = Translation2d(0.0, 8.052)
val TOP_RIGHT_CORNER = Translation2d(17.548, 8.052)

class Arena2025Bunnybots : SimulatedArena(CarrotChaosObstacleMap()) {
    public class CarrotChaosObstacleMap : FieldMap() {
        init {
            addBorderLine(BOTTOM_LEFT_CORNER, BOTTOM_RIGHT_CORNER)
            addBorderLine(BOTTOM_LEFT_CORNER, TOP_LEFT_CORNER)
            addBorderLine(TOP_RIGHT_CORNER, BOTTOM_RIGHT_CORNER)
            addBorderLine(TOP_RIGHT_CORNER, TOP_LEFT_CORNER)

            // TODO: Add feeder stations
        }
    }

    init {
        // Add feeder simulations here
    }

    override fun placeGamePiecesOnField() {
        // TODO: Actual positions
        super.addGamePiece(BunnybotsCarrotOnField(Pose2d(5.0, 5.0, Rotation2d(0.0.degrees))))
    }

    override fun getGamePiecesByType(type: String?): List<Pose3d?>? {
        return super.getGamePiecesByType(type);

        // Possibly visualization for carrots in the feeder? idk
    }
}