package com.frcteam3636.swervebase.subsystems.indexer

import com.frcteam3636.swervebase.Robot
import com.frcteam3636.swervebase.utils.math.volts
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger

class Indexer : Subsystem {
    private val io = IndexerIOReal()
//    private val io = when (Robot.model) {
//        Robot.model.SIMULATION -> IndexerIOSim()
//        Robot.model.COMPETITION -> IndexerIOReal()
//    }

    var inputs = LoggedIndexerInputs()

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("indexer", inputs)
    }

    fun intake() : Command = Commands.startEnd(
        { io.setVoltage(2.0.volts)},
        {io.setVoltage(0.0.volts)}
    )

    fun outTake() : Command = Commands.startEnd(
        { io.setVoltage(-2.0.volts)},
        {io.setVoltage(0.0.volts)}
    )

}