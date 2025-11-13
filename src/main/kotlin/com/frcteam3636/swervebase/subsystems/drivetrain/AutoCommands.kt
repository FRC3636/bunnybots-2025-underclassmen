package com.frcteam3636.swervebase.subsystems.drivetrain

import choreo.auto.AutoFactory
import com.frcteam3636.swervebase.subsystems.intake.Intake
import com.frcteam3636.swervebase.subsystems.shooter.Shooter
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands

class AutoCommands(var autoFactory: AutoFactory) {
    fun leftOneCarrot(): Command = Commands.sequence(
        autoFactory.trajectoryCmd("LeftPathStart"),
        Shooter.outtakeAllCarrots()
    )

    fun leftOneCycle(): Command = Commands.sequence(
        leftOneCarrot(),
        Commands.race(
            autoFactory.trajectoryCmd("LeftPathCycle"),
            Intake.intake(),
        ),
        Shooter.outtakeAllCarrots()
    )

    fun middlePreloadCycle(): Command = Commands.sequence(
        autoFactory.trajectoryCmd("MiddleStart"),
        Shooter.outtakeAllCarrots()
    )

    fun middleHug(): Command = Commands.sequence(
        middlePreloadCycle(),
        Commands.race(
            autoFactory.trajectoryCmd("MiddleHug"),
            Intake.intake()
        ),
        Shooter.outtakeAllCarrots()
    )

    fun middleFar(): Command = Commands.sequence(
        middlePreloadCycle(),
        Commands.race(
            autoFactory.trajectoryCmd("MiddleFar"),
            Intake.intake()
        ),
        Shooter.outtakeAllCarrots()
    )

    fun rightPreloadCycle(): Command = Commands.sequence(
        autoFactory.trajectoryCmd("RightPathStart"),
        Shooter.outtakeAllCarrots()
    )

    fun rightOneCycle(): Command = Commands.sequence(
        rightPreloadCycle(),
        Commands.race(
            autoFactory.trajectoryCmd("RightPathCycle"),
            Intake.intake(),
        ),
        Shooter.outtakeAllCarrots()
    )
}