package com.frcteam3636.swervebase.subsystems.shooter

import com.frcteam3636.swervebase.Robot
import com.frcteam3636.swervebase.utils.math.volts
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger

object Shooter : Subsystem {

    private val io = when (Robot.model) {
        Robot.Model.SIMULATION -> ShooterIOSim()
        Robot.Model.COMPETITION -> ShooterIOReal()
    }

    val inputs = LoggedShooterInputs()

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("indexer", inputs)
    }

    fun intake() : Command = Commands.startEnd(
        { io.setVoltage(2.0.volts) },
        { io.setVoltage(0.0.volts) }
    )

    fun outtake() : Command = Commands.startEnd(
        { io.setVoltage((-2.0).volts) },
        { io.setVoltage(0.0.volts) }
    )

}