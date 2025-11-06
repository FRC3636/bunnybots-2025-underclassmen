package com.frcteam3636.swervebase.subsystems.intake

import com.frcteam3636.swervebase.Robot
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands

object Intake: Subsystem {
    private val io = when (Robot.model) {
        Robot.Model.SIMULATION -> IntakeIOSim()
        Robot.Model.COMPETITION -> IntakeIOReal()
    }

    var inputs = LoggedIntakeInputs()

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("intake", inputs)
    }

    fun intake() : Command = Commands.sequence(
    )

}