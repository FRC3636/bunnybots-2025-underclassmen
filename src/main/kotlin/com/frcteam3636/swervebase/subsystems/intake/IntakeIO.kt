package com.frcteam3636.swervebase.subsystems.intake

import com.frcteam3636.swervebase.REVMotorControllerId
import com.frcteam3636.swervebase.SparkFlex
import com.frcteam3636.swervebase.SparkMax
import com.frcteam3636.swervebase.subsystems.indexer.IndexerInputs
import com.frcteam3636.swervebase.utils.math.meters
import com.frcteam3636.swervebase.utils.math.rotationsPerSecond
import com.frcteam3636.swervebase.utils.math.rpm
import com.revrobotics.servohub.ServoHub.ResetMode
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkBase.PersistMode
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkFlexConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Voltage
import org.ironmaple.simulation.IntakeSimulation
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation
import org.team9432.annotation.Logged
import kotlin.math.abs

@Logged
open class IntakeInputs {
    var intakeMotorVelocity = 0.rotationsPerSecond
}

interface IntakeIO {
    fun setSpeed(percent: Double)
    fun setVoltage(voltage: Voltage)
    fun updateInputs(inputs: IntakeInputs)
}

class IntakeIOReal : IntakeIO {

    private var intakeMotor = SparkMax(REVMotorControllerId.IndexerMotor, SparkLowLevel.MotorType.kBrushless).apply {
        val innerConfig = SparkMaxConfig().apply {
            idleMode(SparkBaseConfig.IdleMode.kBrake)
        }
        configure(innerConfig, SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
    }

    override fun setSpeed(percent: Double) {
        intakeMotor.set(percent)
    }

    override fun setVoltage(voltage: Voltage) {
        intakeMotor.setVoltage(voltage)
    }

    override fun updateInputs(inputs: IntakeInputs) {
        inputs.intakeMotorVelocity = intakeMotor.encoder.velocity.rpm
    }
}
