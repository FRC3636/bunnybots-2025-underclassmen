package com.frcteam3636.swervebase.subsystems.indexer

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.littletonrobotics.junction.Logger

object Indexer: Subsystem {
    private val io = IndexerIOReal()

    var inputs = LoggedIndexerInputs()

    val isCarrotDetected: Trigger = Trigger {
        inputs.isCarrotDetected
    }

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("indexer", inputs)
    }

    fun intake() : Command = Commands.startEnd(
        {
            io.setSpeed(0.7)
        },
        {
            io.setSpeed(0.0)
        }
    )

    fun outtake() : Command = Commands.startEnd(
        {
            io.setSpeed(-0.5)
        },
        {
            io.setSpeed(0.0)
        }
    )

}