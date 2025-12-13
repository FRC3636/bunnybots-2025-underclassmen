package com.frcteam3636.swervebase.subsystems.intake

import com.frcteam3636.swervebase.utils.math.volts
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands

object Intake: Subsystem {
    private val io = IntakeIOReal()

    var inputs = LoggedIntakeInputs()

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("intake", inputs)
    }

    fun intake() : Command = Commands.startEnd(
        {
            io.setVoltage((4.0).volts)
        },
        {
            io.setVoltage((0.0).volts)
        }
    )

    fun outtake() : Command = Commands.startEnd(
        {
            io.setVoltage((-2.0).volts)
        },
        {
            io.setVoltage((0.0).volts)
        }
    )

}