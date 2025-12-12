package com.frcteam3636.swervebase.subsystems.indexer

import com.ctre.phoenix6.BaseStatusSignal
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.littletonrobotics.junction.Logger
import java.util.Timer

object Indexer: Subsystem {
    private val io = IndexerIOReal()

    private var wasDetected = false
    var inputs = LoggedIndexerInputs()

    val isCarrotDetected: Trigger = Trigger {
        inputs.isCarrotDetected
    }

    private var detectedTimer = edu.wpi.first.wpilibj.Timer()
    var isIndexerEmpty = false

    override fun periodic() {
        if (inputs.isCarrotDetected) {
            wasDetected = true
            detectedTimer.stop()
            detectedTimer.reset()
        }
        if (!inputs.isCarrotDetected && wasDetected) {
            wasDetected = false
            detectedTimer.reset()
            detectedTimer.start()
        }
        if (detectedTimer.advanceIfElapsed(0.5)) {
            isIndexerEmpty = true
        }
        else {
            isIndexerEmpty = false
        }

        io.updateInputs(inputs)
        Logger.processInputs("indexer", inputs)
    }

    fun intake() : Command = Commands.startEnd(
        {
            io.setSpeed(-0.7)
        },
        {
            io.setSpeed(0.0)
        }
    )

    fun outtake() : Command = Commands.startEnd(
        {
            io.setSpeed(0.5)
        },
        {
            io.setSpeed(0.0)
        }
    )

    fun getStatusSignals(): MutableList<BaseStatusSignal> {
        return io.getStatusSignals()
    }
}