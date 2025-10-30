package com.frcteam3636.swervebase.subsystems.indexer

import com.frcteam3636.swervebase.utils.math.volts
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger

class Indexer : Subsystem {
    private val io = IndexerIOReal()

    var inputs = LoggedIndexerInputs()

    var isIntakeRunning = false;

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("indexer", inputs)
    }

    fun intake() : Command = Commands.sequence(
        runOnce { io.setVoltage(2.0.volts) },
        Commands.waitUntil { inputs.isCarrotDetected },
        runOnce { io.setVoltage(1.0.volts) },
        Commands.waitUntil { !inputs.isCarrotDetected },
        runOnce { io.setSpeed(-0.02) }
    ).onlyWhile { isIntakeRunning }.withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf)
}