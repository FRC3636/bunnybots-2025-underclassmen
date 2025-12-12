package com.frcteam3636.swervebase.subsystems.drivetrain

import choreo.auto.AutoFactory
import com.frcteam3636.swervebase.Robot
import com.frcteam3636.swervebase.subsystems.intake.Intake
import com.frcteam3636.swervebase.subsystems.shooter.Shooter
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands

class AutoCommands(var autoFactory: AutoFactory) {
    fun testAuto(): Command = autoFactory.trajectoryCmd("RightPathStart")

    fun leftOneCarrot(): Command = Commands.sequence(
        autoFactory.trajectoryCmd("LeftPathStart"),
        Robot.outtakeUntilEmpty()
    )

    fun leftOneCycle(): Command = Commands.sequence(
        leftOneCarrot(),
        Commands.deadline(
            autoFactory.trajectoryCmd("LeftPathCycle"),
            Robot.intakeUntilFull()
        ),
        Robot.outtakeUntilEmpty()
    )

    fun middlePreloadCycle(): Command = Commands.sequence(
        autoFactory.trajectoryCmd("MiddleStart"),
        Robot.outtakeUntilEmpty()
    )

    fun middleHug(): Command = Commands.sequence(
        middlePreloadCycle(),
        Commands.deadline(
            autoFactory.trajectoryCmd("MiddleHug"),
            Robot.intakeUntilFull()
        ),
        Robot.outtakeUntilEmpty()
    )

    fun middleFar(): Command = Commands.sequence(
        middlePreloadCycle(),
        Commands.deadline(
            autoFactory.trajectoryCmd("MiddleFar"),
            Robot.intakeUntilFull()
        ),
        Robot.outtakeUntilEmpty()
    )

    fun rightPreloadCycle(): Command = Commands.sequence(
        autoFactory.trajectoryCmd("RightPathStart"),
        Robot.outtakeUntilEmpty()
    )

    fun rightOneCycle(): Command = Commands.sequence(
        rightPreloadCycle(),
        Commands.deadline(
            autoFactory.trajectoryCmd("RightPathCycle"),
            Robot.intakeUntilFull()
        ),
        Robot.outtakeUntilEmpty()
    )
}