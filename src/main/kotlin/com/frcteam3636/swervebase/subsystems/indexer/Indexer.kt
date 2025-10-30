package com.frcteam3636.swervebase.subsystems.indexer

import com.frcteam3636.swervebase.Robot
import com.frcteam3636.swervebase.subsystems.drivetrain.Drivetrain
import com.frcteam3636.swervebase.subsystems.drivetrain.Drivetrain.Constants.MODULE_POSITIONS
import com.frcteam3636.swervebase.subsystems.drivetrain.DrivetrainIOReal
import com.frcteam3636.swervebase.subsystems.drivetrain.DrivetrainIOSim
import com.frcteam3636.swervebase.subsystems.drivetrain.DrivingTalon
import com.frcteam3636.swervebase.subsystems.drivetrain.Mk5nSwerveModule
import com.frcteam3636.swervebase.subsystems.drivetrain.TurningTalon
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Subsystem
import org.littletonrobotics.junction.Logger

class Indexer : Subsystem {
    private val io = DrivetrainIOReal()

    var inputs = LoggedIndexerInputs()

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("indexer", inputs)
    }

    fun intake() : Command = Commands.sequence(
        // TODO()
    )
}