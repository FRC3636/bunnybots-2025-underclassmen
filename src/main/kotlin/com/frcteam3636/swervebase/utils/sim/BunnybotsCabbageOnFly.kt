package com.frcteam3636.swervebase.utils.sim

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import org.ironmaple.simulation.gamepieces.GamePieceProjectile

class BunnybotsCabbageOnFly(
    robotPosition: Translation2d,
    shooterPositionOnRobot: Translation2d,
    chassisSpeeds: ChassisSpeeds,
    shooterFacing: Rotation2d,
    initialHeight: Distance,
    launchingSpeed: LinearVelocity,
    shooterAngle: Angle
) : GamePieceProjectile(
    BUNNYBOTS_CABBAGE_INFO,
    robotPosition,
    shooterPositionOnRobot,
    chassisSpeeds,
    shooterFacing,
    initialHeight,
    launchingSpeed,
    shooterAngle
) {
    init {
        super.withTouchGroundHeight(0.8)
        super.enableBecomesGamePieceOnFieldAfterTouchGround()
    }
}