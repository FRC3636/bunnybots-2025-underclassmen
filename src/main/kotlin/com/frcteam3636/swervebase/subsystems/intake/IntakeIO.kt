package com.frcteam3636.swervebase.subsystems.intake

import com.frcteam3636.swervebase.subsystems.indexer.IndexerInputs
import com.frcteam3636.swervebase.utils.math.meters
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Voltage
import org.ironmaple.simulation.IntakeSimulation
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation
import org.team9432.annotation.Logged
import kotlin.math.abs

@Logged
open class IntakeInputs {
}

interface IntakeIO {
    fun setSpeed(percent: Double)
    fun setVoltage(voltage: Voltage)
    fun updateInputs(inputs: IntakeInputs)
}

class IntakeIOReal : IntakeIO {
    override fun setSpeed(percent: Double) {
        TODO("Not yet implemented")
    }

    override fun setVoltage(voltage: Voltage) {
        TODO("Not yet implemented")
    }

    override fun updateInputs(inputs: IntakeInputs) {

    }
}

class IntakeIOSim : IntakeIO {
//    private val intakeSimulation: IntakeSimulation = IntakeSimulation.OverTheBumperIntake(
//        "Carrot",
//        // Get AbstractDriveTrainSimulation from Drivetrain when that is implemented...
//        0.7.meters,
//        0.2.meters,
//        IntakeSimulation.IntakeSide.BACK,
//        1
//    )

//    override fun setSpeed(percent: Double) {
//        // I don't like doing this
//        if (abs(percent) < 0.1) {
//            intakeSimulation.stopIntake();
//        }
//        else {
//            intakeSimulation.startIntake();
//        }
//    }

    override fun setSpeed(percent: Double) {
        TODO("Not yet implemented")
    }

    override fun setVoltage(voltage: Voltage) {
        TODO("Unimplemented, what even is a voltage")
    }

    override fun updateInputs(inputs: IntakeInputs) {

    }
}