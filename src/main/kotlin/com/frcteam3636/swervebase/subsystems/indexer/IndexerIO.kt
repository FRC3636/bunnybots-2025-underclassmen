package com.frcteam3636.swervebase.subsystems.indexer

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
open class IndexerInputs {
    var isCarrotDetected: Boolean = false
    var indexerMotorVelocity = 0.rotationsPerSecond
}

interface IndexerIO {
    fun setSpeed(percent: Double)
    fun setVoltage(voltage: Voltage)
    fun updateInputs(inputs: IndexerInputs)
}

class IndexerIOReal : IndexerIO {

    private var indexerMotor = SparkMax(REVMotorControllerId.IndexerMotor, SparkLowLevel.MotorType.kBrushless).apply {
        val innerConfig = SparkMaxConfig().apply {
            idleMode(SparkBaseConfig.IdleMode.kBrake)
        }
        configure(innerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
    }

    override fun setSpeed(percent: Double) {
        indexerMotor.set(percent)
    }

    override fun setVoltage(voltage: Voltage) {
        indexerMotor.setVoltage(voltage)
    }

    override fun updateInputs(inputs: IndexerInputs) {
        //TODO("How is a carrot detected?")
        inputs.indexerMotorVelocity = indexerMotor.encoder.velocity.rpm
    }
}

class IndexerIOSim: IndexerIO {

    override fun setSpeed(percent: Double) {
        //TODO("Not yet implemented")
    }

    override fun setVoltage(voltage: Voltage) {
        //TODO("Not yet implemented")
    }

    override fun updateInputs(inputs: IndexerInputs) {
        //TODO("Not yet implemented")
    }
}