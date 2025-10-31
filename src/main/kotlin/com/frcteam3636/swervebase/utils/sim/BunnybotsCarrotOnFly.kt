package com.frcteam3636.swervebase.utils.sim

import com.frcteam3636.swervebase.utils.math.inMeters
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import org.ironmaple.simulation.SimulatedArena
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation
import org.ironmaple.simulation.gamepieces.GamePieceProjectile
import java.util.function.DoubleSupplier
import kotlin.math.max

class BunnybotsCarrotOnFly(
    robotPosition: Translation2d,
    shooterPositionOnRobot: Translation2d,
    chassisSpeeds: ChassisSpeeds,
    shooterFacing: Rotation2d,
    initialHeight: Distance,
    launchingSpeed: LinearVelocity,
    shooterAngle: Angle
) : GamePieceProjectile(
    BUNNYBOTS_CARROT_INFO,
    robotPosition,
    shooterPositionOnRobot,
    chassisSpeeds,
    shooterFacing,
    initialHeight,
    launchingSpeed,
    shooterAngle
) {
    override fun addGamePieceAfterTouchGround(simulatedArena: SimulatedArena?) {
        if (!super.becomesGamePieceOnGroundAfterTouchGround) {
            return;
        }

        val supplier: DoubleSupplier = DoubleSupplier {
            (BUNNYBOTS_CARROT_INFO.gamePieceHeight()
                .inMeters() / 2.0).coerceAtLeast(getPositionAtTime(super.launchedTimer.get()).z)
        }
        simulatedArena?.addGamePiece(GamePieceOnFieldSimulation(
            BUNNYBOTS_CARROT_INFO,
            supplier,
            Pose2d(getPositionAtTime(launchedTimer.get()).toTranslation2d(), initialLaunchingVelocityMPS.angle),
            super.initialLaunchingVelocityMPS
        ))
    }
}