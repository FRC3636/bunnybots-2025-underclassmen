package com.frcteam3636.swervebase.subsystems.drivetrain

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.SignalLogger
import com.frcteam3636.swervebase.CTREDeviceId
import com.frcteam3636.swervebase.Robot
import com.frcteam3636.swervebase.subsystems.drivetrain.Drivetrain.Constants.BRAKE_POSITION
import com.frcteam3636.swervebase.subsystems.drivetrain.Drivetrain.Constants.FREE_SPEED
import com.frcteam3636.swervebase.subsystems.drivetrain.Drivetrain.Constants.JOYSTICK_DEADBAND
import com.frcteam3636.swervebase.subsystems.drivetrain.Drivetrain.Constants.MODULE_POSITIONS
import com.frcteam3636.swervebase.subsystems.drivetrain.Drivetrain.Constants.PATH_FOLLOWING_ROTATION_GAINS
import com.frcteam3636.swervebase.subsystems.drivetrain.Drivetrain.Constants.PATH_FOLLOWING_TRANSLATION_GAINS
import com.frcteam3636.swervebase.subsystems.drivetrain.Drivetrain.Constants.ROTATION_SENSITIVITY
import com.frcteam3636.swervebase.subsystems.drivetrain.Drivetrain.Constants.TRANSLATION_SENSITIVITY
import com.frcteam3636.swervebase.utils.fieldRelativeTranslation2d
import com.frcteam3636.swervebase.utils.math.*
import com.frcteam3636.swervebase.utils.swerve.Corner
import com.frcteam3636.swervebase.utils.swerve.PerCorner
import com.frcteam3636.swervebase.utils.swerve.cornerStatesToChassisSpeeds
import com.frcteam3636.swervebase.utils.swerve.toCornerSwerveModuleStates
import com.frcteam3636.swervebase.utils.translation2d
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.commands.PathfindingCommand
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.pathfinding.Pathfinding
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import org.littletonrobotics.junction.Logger
import java.util.*
import java.util.concurrent.locks.ReentrantLock
import kotlin.jvm.optionals.getOrNull
import kotlin.math.abs
import kotlin.math.absoluteValue
import kotlin.math.pow
import kotlin.math.withSign

/** A singleton object representing the drivetrain. */
object Drivetrain : Subsystem {
    private val io = when (Robot.model) {
        Robot.Model.SIMULATION -> DrivetrainIOSim()
        Robot.Model.COMPETITION -> DrivetrainIOReal(
            MODULE_POSITIONS.zip(Drivetrain.Constants.KRAKEN_MODULE_CAN_IDS)
                .map { (corner, ids) ->
                    val (driveId, turnId, encoderId) = ids
                    Mk5nSwerveModule(
                        DrivingTalon(driveId),
                        TurningTalon(turnId, encoderId, corner.magnetOffset),
                        corner.position.rotation
                    )
                })
    }
    val inputs = LoggedDrivetrainInputs()

    private val mt2Algo = LimelightAlgorithm.MegaTag2({
        poseEstimator.estimatedPosition.rotation
    }, {
        inputs.gyroVelocity
    })

    val odometryLock = ReentrantLock()

    private val absolutePoseIOs = when (Robot.model) {
        Robot.Model.SIMULATION -> mapOf(
            "Limelight" to CameraSimPoseProvider("limelight", Transform3d()),
        )

        else -> mapOf(
//            "Limelight Rear" to LimelightPoseProvider(
//                "limelight-rear",
//                algorithm = mt2Algo
//            ),
        )
    }.mapValues { Pair(it.value, AbsolutePoseProviderInputs()) }

    /** Helper for converting a desired drivetrain velocity into the speeds and angles for each swerve module */
    private val kinematics =
        SwerveDriveKinematics(
            *Constants.MODULE_POSITIONS
                .map { it.position.translation }
                .toTypedArray()
        )

    /** Helper for estimating the location of the drivetrain on the field */
    val poseEstimator =
        SwerveDrivePoseEstimator(
            kinematics, // swerve drive kinematics
            inputs.gyroRotation, // initial gyro rotation
            inputs.measuredPositions.toTypedArray(), // initial module positions
            Pose2d(), // initial pose
            VecBuilder.fill(0.02, 0.02, 0.005),
            // Overwrite each measurement
            VecBuilder.fill(0.0, 0.0, 0.0)
        )

    /** Whether every sensor used for pose estimation is connected. */
    val allPoseProvidersConnected
        get() = absolutePoseIOs.values.all { it.second.connected }


    init {
        Pathfinding.setPathfinder(
            LocalADStarAK()
        )

//        AutoBuilder.configure(
//            this::estimatedPose,
//            this::estimatedPose::set,
//            this::measuredChassisSpeeds,
//            this::desiredChassisSpeeds::set,
//            PPHolonomicDriveController(
//                PATH_FOLLOWING_TRANSLATION_GAINS,
//                PATH_FOLLOWING_ROTATION_GAINS
//            ),
//            RobotConfig.fromGUISettings(),
//            // Mirror path when the robot is on the red alliance (the robot starts on the opposite side of the field)
//            { DriverStation.getAlliance() == Optional.of(DriverStation.Alliance.Red) },
//            this
//        )

        if (Robot.model != Robot.Model.SIMULATION) {
            PathfindingCommand.warmupCommand().schedule()
        }

        if (io is DrivetrainIOSim) {
            poseEstimator.resetPose(io.swerveDriveSimulation.simulatedDriveTrainPose)
            io.registerPoseProviders(absolutePoseIOs.values.map { it.first })
        }
    }

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Drivetrain", inputs)

        // Update absolute pose sensors and add their measurements to the pose estimator
        for ((name, ioPair) in absolutePoseIOs) {
            val (sensorIO, inputs) = ioPair

            sensorIO.updateInputs(inputs)
            Logger.processInputs("Drivetrain/Absolute Pose/$name", inputs)

            Logger.recordOutput("Drivetrain/Absolute Pose/$name/Has Measurement", inputs.measurement != null)
            inputs.measurement?.let {
                poseEstimator.addAbsolutePoseMeasurement(it)
                Logger.recordOutput("Drivetrain/Absolute Pose/$name/Measurement", it)
                Logger.recordOutput("Drivetrain/Last Added Pose", it.pose)
                Logger.recordOutput("Drivetrain/Absolute Pose/$name/Pose", it.pose)
            }
        }

        // Use the new measurements to update the pose estimator
        poseEstimator.update(
            inputs.gyroRotation,
            inputs.measuredPositions.toTypedArray()
        )

        Logger.recordOutput("Drivetrain/Pose Estimator/Estimated Pose", poseEstimator.estimatedPosition)
        Logger.recordOutput("Drivetrain/Estimated Pose", estimatedPose)
        Logger.recordOutput("Drivetrain/Chassis Speeds", measuredChassisSpeeds)
        Logger.recordOutput("Drivetrain/Desired Chassis Speeds", desiredChassisSpeeds)

        Logger.recordOutput(
            "Drivetrain/TagPoses", *FIELD_LAYOUT.tags
                .filter { tag ->
                    absolutePoseIOs.values.any { it.second.observedTags.contains(tag.ID) }
                }
                .map { it.pose }
                .toTypedArray())
    }

    /** The desired speeds and angles of the swerve modules. */
    private var desiredModuleStates
        get() = io.desiredStates
        set(value) {
            synchronized(this) {
                val stateArr = value.toTypedArray()
                SwerveDriveKinematics.desaturateWheelSpeeds(stateArr, FREE_SPEED)

                io.desiredStates = PerCorner.fromConventionalArray(stateArr)
                Logger.recordOutput("Drivetrain/Desired States", *stateArr)
            }
        }

    /**
     * The current speed of chassis relative to the ground,
     * assuming that the wheels have perfect traction with the ground.
     */
    val measuredChassisSpeeds get() = kinematics.cornerStatesToChassisSpeeds(inputs.measuredStates)

    /**
     * The chassis speeds that the drivetrain is attempting to move at.
     *
     * Note that the speeds are relative to the chassis, not the field.
     */
    private var desiredChassisSpeeds
        get() = kinematics.cornerStatesToChassisSpeeds(desiredModuleStates)
        set(value) {
            val discretized = ChassisSpeeds.discretize(value, Robot.period)
            desiredModuleStates = kinematics.toCornerSwerveModuleStates(discretized)
        }

    /** The estimated pose of the robot on the field, using the yaw value measured by the gyro. */
    var estimatedPose: Pose2d
        get() {
            return poseEstimator.estimatedPosition
        }
        private set(value) {
            poseEstimator.resetPosition(
                inputs.gyroRotation,
                inputs.measuredPositions.toTypedArray(),
                value
            )
        }


    fun getStatusSignals(): MutableList<BaseStatusSignal> {
        return io.getStatusSignals()
    }

    private fun isInDeadband(translation: Translation2d) =
        abs(translation.x) < JOYSTICK_DEADBAND && abs(translation.y) < JOYSTICK_DEADBAND

    private fun drive(translationInput: Translation2d, rotationInput: Translation2d) {
        if (isInDeadband(translationInput) && isInDeadband(rotationInput)) {
            // No joystick input - stop moving!
            desiredModuleStates = BRAKE_POSITION
        } else {
            desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                calculateInputCurve(translationInput.x) * FREE_SPEED.baseUnitMagnitude() * TRANSLATION_SENSITIVITY,
                calculateInputCurve(translationInput.y) * FREE_SPEED.baseUnitMagnitude() * TRANSLATION_SENSITIVITY,
                rotationInput.y * TAU * ROTATION_SENSITIVITY,
                estimatedPose.rotation
            )
        }
    }

    private fun calculateInputCurve(input: Double): Double {
        val exponent = 1.7

        return input.absoluteValue.pow(exponent).withSign(input)
    }

    fun driveWithJoysticks(translationJoystick: Joystick, rotationJoystick: Joystick): Command =
        run {
            // Directly accessing Joystick.x/y gives inverted values - use a `Translation2d` instead.
            drive(translationJoystick.fieldRelativeTranslation2d, rotationJoystick.translation2d)
        }

    @Suppress("unused")
    fun driveWithController(controller: CommandXboxController): Command =
        run {
            val translationInput = Translation2d(controller.leftX, controller.leftY)
            val rotationInput = Translation2d(controller.rightX, controller.rightY)

            drive(translationInput, rotationInput)
        }

    fun zeroGyro(isReversed: Boolean = false, offset: Rotation2d = Rotation2d.kZero) {
        // Tell the gyro that the robot is facing the other alliance.
        var zeroPos = when (DriverStation.getAlliance().getOrNull()) {
            DriverStation.Alliance.Red -> Rotation2d.k180deg
            else -> Rotation2d.kZero
        }

        if (isReversed) {
            zeroPos += Rotation2d.k180deg
        }

        estimatedPose = Pose2d(estimatedPose.translation, zeroPos + offset)
//        io.setGyro(zeroPos)
    }

    var sysID = SysIdRoutine(
        SysIdRoutine.Config(
            0.5.voltsPerSecond, 2.volts, null, {
                SignalLogger.writeString("state", it.toString())
            }), SysIdRoutine.Mechanism(
            io::runCharacterization,
            null,
            this,
        )
    )

    fun sysIdQuasistatic(direction: SysIdRoutine.Direction) = run {
        io.runCharacterization(0.volts)
    }.withTimeout(1.0).andThen(sysID.quasistatic(direction))!!

    fun sysIdDynamic(direction: SysIdRoutine.Direction) = run {
        io.runCharacterization(0.volts)
    }.withTimeout(1.0).andThen(sysID.dynamic(direction))!!

    internal object Constants {
        // Translation/rotation coefficient for teleoperated driver controls
        /** Unit: Percent of max robot speed */
        const val TRANSLATION_SENSITIVITY = 1.0 // FIXME: Increase

        /** Unit: Rotations per second */
        const val ROTATION_SENSITIVITY = 0.8

        val ROBOT_LENGTH = 25.5.inches
        val ROBOT_WIDTH = 25.5.inches

        val BUMPER_WIDTH = 30.inches
        val BUMPER_LENGTH = 30.inches

        const val JOYSTICK_DEADBAND = 0.075

        val FRONT_RIGHT_MAGNET_OFFSET = TunerConstants.FrontRight!!.EncoderOffset
        val FRONT_LEFT_MAGNET_OFFSET = TunerConstants.FrontLeft!!.EncoderOffset
        val BACK_RIGHT_MAGNET_OFFSET = TunerConstants.BackRight!!.EncoderOffset
        val BACK_LEFT_MAGNET_OFFSET = TunerConstants.BackLeft!!.EncoderOffset

        val MODULE_POSITIONS = PerCorner(
            frontLeft = Corner(
                Pose2d(
                    Translation2d(ROBOT_LENGTH, ROBOT_WIDTH) / 2.0, Rotation2d.fromDegrees(0.0)
                ), FRONT_LEFT_MAGNET_OFFSET
            ),
            frontRight = Corner(
                Pose2d(
                    Translation2d(ROBOT_LENGTH, -ROBOT_WIDTH) / 2.0, Rotation2d.fromDegrees(180.0)
                ), FRONT_RIGHT_MAGNET_OFFSET
            ),
            backLeft = Corner(
                Pose2d(
                    Translation2d(-ROBOT_LENGTH, ROBOT_WIDTH) / 2.0, Rotation2d.fromDegrees(0.0)
                ), BACK_LEFT_MAGNET_OFFSET
            ),
            backRight = Corner(
                Pose2d(
                    Translation2d(-ROBOT_LENGTH, -ROBOT_WIDTH) / 2.0, Rotation2d.fromDegrees(180.0)
                ), BACK_RIGHT_MAGNET_OFFSET
            ),
        )

        // Chassis Control
        val FREE_SPEED = 6.06.metersPerSecond

        val PATH_FOLLOWING_TRANSLATION_GAINS = PIDGains(5.0).toPPLib()
        val PATH_FOLLOWING_ROTATION_GAINS = PIDGains(5.0).toPPLib()

        // CAN IDs
        val KRAKEN_MODULE_CAN_IDS =
            PerCorner(
                frontLeft =
                    Triple(
                        CTREDeviceId.FrontLeftDrivingMotor,
                        CTREDeviceId.FrontLeftTurningMotor,
                        CTREDeviceId.FrontLeftTurningEncoder
                    ),
                frontRight =
                    Triple(
                        CTREDeviceId.FrontRightDrivingMotor,
                        CTREDeviceId.FrontRightTurningMotor,
                        CTREDeviceId.FrontRightTurningEncoder
                    ),
                backLeft =
                    Triple(
                        CTREDeviceId.BackLeftDrivingMotor,
                        CTREDeviceId.BackLeftTurningMotor,
                        CTREDeviceId.BackLeftTurningEncoder
                    ),
                backRight =
                    Triple(
                        CTREDeviceId.BackRightDrivingMotor,
                        CTREDeviceId.BackRightTurningMotor,
                        CTREDeviceId.BackRightTurningEncoder
                    ),
            )

        /** A position with the modules radiating outwards from the center of the robot, preventing movement. */
        val BRAKE_POSITION =
            MODULE_POSITIONS.map { module -> SwerveModuleState(0.0, module.position.translation.angle) }
    }
}
