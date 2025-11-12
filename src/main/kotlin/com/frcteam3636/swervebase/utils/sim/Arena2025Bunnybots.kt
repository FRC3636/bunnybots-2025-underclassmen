package com.frcteam3636.swervebase.utils.sim

import com.frcteam3636.swervebase.utils.math.degrees
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import org.ironmaple.simulation.SimulatedArena

internal val NUM_CARROTS_IN_ROW = 40

val FIELD_WIDTH = 17.548
val FIELD_HEIGHT = 8.052

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

            addRectangularObstacle(2.4384, 2.4384, Pose2d(4.1, FIELD_HEIGHT / 2.0, Rotation2d.kZero))
            addRectangularObstacle(2.4384, 2.4384, Pose2d(FIELD_WIDTH - 4.3, FIELD_HEIGHT / 2.0, Rotation2d.kZero))
        }
    }

    init {
        // Add feeder simulations here
    }

    override fun placeGamePiecesOnField() {
        val rows: ArrayList<ArrayList<BunnybotsCarrotOnField>> = arrayListOf(arrayListOf(), arrayListOf(), arrayListOf(), arrayListOf(), arrayListOf())
        rows[0].add(BunnybotsCarrotOnField(Pose2d(FIELD_WIDTH / 2.0 - 0.6, 0.3, Rotation2d(90.0.degrees))))
        rows[1].add(BunnybotsCarrotOnField(Pose2d(FIELD_WIDTH / 2.0 - 0.3, 0.3, Rotation2d(90.0.degrees))))
        rows[2].add(BunnybotsCarrotOnField(Pose2d(FIELD_WIDTH / 2.0, 0.3, Rotation2d(90.0.degrees))))
        rows[3].add(BunnybotsCarrotOnField(Pose2d(FIELD_WIDTH / 2.0 + 0.3, 0.3, Rotation2d(90.0.degrees))))
        rows[4].add(BunnybotsCarrotOnField(Pose2d(FIELD_WIDTH / 2.0 + 0.6, 0.3, Rotation2d(90.0.degrees))))
        for (row in rows) {
            val initializer = row[0]
            val x = initializer.poseOnField.x
            var y = initializer.poseOnField.y
            for (i in 0..<NUM_CARROTS_IN_ROW) {
                y += 0.19
                row.add(BunnybotsCarrotOnField(Pose2d(x, y, Rotation2d(90.0.degrees))))
            }
        }

        for (row in rows) {
            for (item in row) {
                super.addGamePiece(item)
            }
        }

        super.addGamePiece(BunnybotsCabbageOnField(Pose2d(5.7, FIELD_HEIGHT / 2.0, Rotation2d(0.0.degrees))))
        super.addGamePiece(BunnybotsCabbageOnField(Pose2d(FIELD_WIDTH - 5.7, FIELD_HEIGHT / 2.0, Rotation2d(0.0.degrees))))
    }

    override fun getGamePiecesByType(type: String?): List<Pose3d?>? {
        return super.getGamePiecesByType(type)

        // Possibly visualization for carrots in the feeder? idk
    }
}