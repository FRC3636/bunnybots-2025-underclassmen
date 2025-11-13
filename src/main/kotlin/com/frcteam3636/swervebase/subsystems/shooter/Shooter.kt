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
        { io.setVoltage(4.0.volts, 3.6.volts) },
        { io.setVoltage(0.0.volts) }
    )

    fun outtake() : Command = Commands.startEnd(
        { io.setVoltage((-4.0).volts, (-3.6).volts) },
        { io.setVoltage(0.0.volts) }
    )

    // Here is my sample code, should be adjusted further on to either
    // be smart and wait for all to be shot, or just to use a good time
    // value...
    fun outtakeAllCarrots() : Command = Commands.race(
        outtake(),
        Commands.waitSeconds(3.5)
    )
}