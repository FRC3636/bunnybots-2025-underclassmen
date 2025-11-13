package com.frcteam3636.swervebase

import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.Pigeon2
import com.ctre.phoenix6.hardware.TalonFX
import com.revrobotics.spark.SparkFlex
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax

private val canivoreBus = CANBus("*")

enum class CTREDeviceId(val num: Int, val bus: CANBus) {
    FrontLeftDrivingMotor(1, canivoreBus),
    BackLeftDrivingMotor(2, canivoreBus),
    BackRightDrivingMotor(3, canivoreBus),
    FrontRightDrivingMotor(4, canivoreBus),


    PigeonGyro(20, canivoreBus),
}

enum class REVMotorControllerId(val num: Int) {
    FrontLeftDrivingMotor(1),
    BackLeftDrivingMotor(2),
    BackRightDrivingMotor(3),
    FrontRightDrivingMotor(4),

    FrontLeftTurningMotor(5),
    BackLeftTurningMotor(6),
    BackRightTurningMotor(7),
    FrontRightTurningMotor(8),

    IntakeMotor(10),
    IndexerMotor(11),
    UpperShooterMotor(12),
    LowerShooterMotor(13),

}

fun CANcoder(id: CTREDeviceId) = CANcoder(id.num, id.bus)
fun TalonFX(id: CTREDeviceId) = TalonFX(id.num, id.bus)
fun SparkMax(id: REVMotorControllerId, type: SparkLowLevel.MotorType) = SparkMax(id.num, type)
fun SparkFlex(id: REVMotorControllerId, type: SparkLowLevel.MotorType) = SparkFlex(id.num, type)
fun Pigeon2(id: CTREDeviceId) = Pigeon2(id.num, id.bus)
