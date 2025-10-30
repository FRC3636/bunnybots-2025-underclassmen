package com.frcteam3636.swervebase.subsystems.intake

import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands

class Intake : Subsystem {
    private val io = IntakeIOReal()

    var inputs = LoggedIntakeInputs()

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("intake", inputs)
    }

    fun intake() : Command = Commands.sequence(

    )

}