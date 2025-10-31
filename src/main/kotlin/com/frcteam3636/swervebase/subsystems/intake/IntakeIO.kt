package com.frcteam3636.swervebase.subsystems.intake

import com.frcteam3636.swervebase.subsystems.indexer.IndexerInputs
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Voltage
import org.team9432.annotation.Logged

@Logged
open class IntakeInputs {
}

interface IntakeIO {
    fun setSpeed(percent: Double)
    fun setCurrent(current: Current)
    fun setVoltage(voltage: Voltage)
    fun updateInputs(inputs: IntakeInputs)
}

class IntakeIOReal : IntakeIO {
    override fun setSpeed(percent: Double) {
        TODO("Not yet implemented")
    }

    override fun setCurrent(current: Current) {
        TODO("Not yet implemented")
    }

    override fun setVoltage(voltage: Voltage) {
        TODO("Not yet implemented")
    }

    override fun updateInputs(inputs: IntakeInputs) {

    }
}

class IntakeIOSim : IntakeIO {
    override fun setSpeed(percent: Double) {
        TODO("Not yet implemented")
    }

    override fun setCurrent(current: Current) {
        TODO("Not yet implemented")
    }

    override fun setVoltage(voltage: Voltage) {
        TODO("Not yet implemented")
    }

    override fun updateInputs(inputs: IntakeInputs) {

    }
}