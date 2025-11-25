package com.frcteam3636.swervebase.subsystems.indexer

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.CANrangeConfiguration
import com.ctre.phoenix6.signals.UpdateModeValue
import com.frcteam3636.swervebase.CANrange
import com.frcteam3636.swervebase.CTREDeviceId
import com.frcteam3636.swervebase.REVMotorControllerId
import com.frcteam3636.swervebase.SparkMax
import com.frcteam3636.swervebase.utils.math.amps
import com.frcteam3636.swervebase.utils.math.celsius
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
    var indexerCurrent = 0.amps
    var indexerTemperature = 0.celsius
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
    private var canRange = CANrange(CTREDeviceId.CANRangeIndexer).apply {
        configurator.apply(
            CANrangeConfiguration().apply {
                ProximityParams.ProximityThreshold = 0.1 // From the senior bot, may be different?
                ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz
            }
        )
    }

    private val detectedSignal = canRange.isDetected

    init {
        BaseStatusSignal.setUpdateFrequencyForAll(100.0, detectedSignal)
        canRange.optimizeBusUtilization()
    }

    override fun setSpeed(percent: Double) {
        indexerMotor.set(percent)
    }

    override fun setVoltage(voltage: Voltage) {
        indexerMotor.setVoltage(voltage)
    }

    override fun updateInputs(inputs: IndexerInputs) {
        inputs.indexerMotorVelocity = indexerMotor.encoder.velocity.rpm
        inputs.indexerCurrent = indexerMotor.outputCurrent.amps
        inputs.indexerTemperature = indexerMotor.motorTemperature.celsius
        inputs.isCarrotDetected = detectedSignal.value
    }
}