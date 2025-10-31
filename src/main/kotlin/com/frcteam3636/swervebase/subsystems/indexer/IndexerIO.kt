package com.frcteam3636.swervebase.subsystems.indexer

import com.frcteam3636.swervebase.REVMotorControllerId
import com.frcteam3636.swervebase.SparkMax
import com.frcteam3636.swervebase.utils.math.rotationsPerSecond
import com.revrobotics.spark.SparkBase.PersistMode
import com.revrobotics.spark.SparkBase.ResetMode
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkBase.*
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.units.measure.Voltage
import org.team9432.annotation.Logged

@Logged
open class IndexerInputs {
    var isCarrotDetected: Boolean = false
    var IndexerMotorVelocity = 0.rotationsPerSecond
}

interface IndexerIO {
    fun setSpeed(percent: Double)
    fun setVoltage(voltage: Voltage)
    fun updateInputs(inputs: IndexerInputs)
}

class IndexerIOReal : IndexerIO {

    private var indexerMotor = SparkMax(REVMotorControllerId.IndexerMotor, SparkLowLevel.MotorType.kBrushless).apply {
        val innerConfig = SparkMaxConfig.apply {
            idleMode(idleMode.kBrake)
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
        inputs.IndexerMotorVelocity = indexerMotor.velocity.value
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