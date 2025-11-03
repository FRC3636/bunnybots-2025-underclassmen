package com.frcteam3636.swervebase.subsystems.shooter

import com.frcteam3636.swervebase.REVMotorControllerId
import com.frcteam3636.swervebase.SparkMax
import com.frcteam3636.swervebase.utils.math.rotationsPerSecond
import com.frcteam3636.swervebase.utils.math.rpm
import com.revrobotics.spark.SparkBase.PersistMode
import com.revrobotics.spark.SparkBase.ResetMode
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.units.measure.Voltage
import org.team9432.annotation.Logged
import kotlin.apply

@Logged
open class ShooterInputs {
    var shooterMotorVelocity = 0.rotationsPerSecond
}

interface ShooterIO {
    fun setSpeed(percent: Double)
    fun setVoltage(voltage: Voltage)
    fun updateInputs(inputs: ShooterInputs)
}

class ShooterIOReal : ShooterIO {

    private var shooterMotor = SparkMax(REVMotorControllerId.ShooterMotor, SparkLowLevel.MotorType.kBrushless).apply {
        val innerConfig = SparkMaxConfig().apply {
            idleMode(SparkBaseConfig.IdleMode.kBrake)
        }
        configure(innerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
    }

    override fun setSpeed(percent: Double) {
        shooterMotor.set(percent)
    }

    override fun setVoltage(voltage: Voltage) {
        shooterMotor.setVoltage(voltage)
    }

    override fun updateInputs(inputs: ShooterInputs) {
        inputs.shooterMotorVelocity = shooterMotor.encoder.velocity.rpm
    }
}

class ShooterIOSim: ShooterIO {

    override fun setSpeed(percent: Double) {
        TODO("Not yet implemented")
    }

    override fun setVoltage(voltage: Voltage) {
        TODO("Not yet implemented")
    }

    override fun updateInputs(inputs: ShooterInputs) {
        TODO("Not yet implemented")
    }
}