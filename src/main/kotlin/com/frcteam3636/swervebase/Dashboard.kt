package com.frcteam3636.swervebase

import com.frcteam3636.swervebase.subsystems.drivetrain.Drivetrain
import com.pathplanner.lib.auto.AutoBuilder
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
    LeftOneCarrot("Left One Carrot"),
    LeftOneCycle("Left One Carrot + One Cycle"),
    MiddleOneCarrot("Middle One Carrot"),
    MiddleHug("One Middle Starting Carrot + Hug Middle Wall Cycle"),
    MiddleFar("One Middle Starting Carrot + Hug Far Wall"),
    RightOneCarrot("Right One Carrot"),
    RightOneCycle("Right One Carrot + One Cycle"),
}