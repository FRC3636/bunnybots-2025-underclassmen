package com.frcteam3636.swervebase.subsystems.indexer

import com.frcteam3636.swervebase.REVMotorControllerId
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Voltage
import org.team9432.annotation.Logged

@Logged
open class IndexerInputs {
    var isCarrotDetected: Boolean = false
}

interface IndexerIO {
    fun setSpeed(percent: Double)
    fun setCurrent(current: Current)
    fun setVoltage(voltage: Voltage)
    fun updateInputs(inputs: IndexerInputs)
}

class IndexerIOReal : IndexerIO {
    // I'm kind of just guessing here ...
    private var indexerMotor = SparkMax(REVMotorControllerId.IndexerMotor.num, SparkLowLevel.MotorType.kBrushless);

    override fun setSpeed(percent: Double) {
        TODO("Not yet implemented")
    }

    override fun setCurrent(current: Current) {
        TODO("Not yet implemented")
    }

    override fun setVoltage(voltage: Voltage) {
        TODO("Not yet implemented")
    }

    override fun updateInputs(inputs: IndexerInputs) {
        TODO("How is a carrot detected?")
    }
}