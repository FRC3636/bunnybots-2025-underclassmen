package com.frcteam3636.swervebase

import com.frcteam3636.swervebase.subsystems.drivetrain.Drivetrain
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard

object Dashboard {
    val field = Field2d()
    val autoChooser = SendableChooser<AutoModes>().apply {
        for (autoMode in AutoModes.entries) {
            if (autoMode == AutoModes.None) {
                setDefaultOption(autoMode.autoName, autoMode)
            }
            else {
                addOption(autoMode.autoName, autoMode)
            }
        }
    }

    fun initialize() {
        SmartDashboard.putData(autoChooser)
    }

    fun update() {
        field.robotPose = Drivetrain.estimatedPose
    }
}

// Are these names too long?
enum class AutoModes(val autoName: String) {
    None("None"),
    TestAuto("Test Auto")
}