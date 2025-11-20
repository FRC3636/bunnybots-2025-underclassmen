package com.frcteam3636.swervebase.subsystems.drivetrain


import com.frcteam3636.swervebase.Robot
import com.frcteam3636.swervebase.utils.LimelightHelpers
import com.frcteam3636.swervebase.utils.math.degrees
import com.frcteam3636.swervebase.utils.math.inSeconds
import com.frcteam3636.swervebase.utils.math.meters
import com.frcteam3636.swervebase.utils.math.seconds
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Time
import edu.wpi.first.util.struct.Struct
import edu.wpi.first.util.struct.StructSerializable
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.simulation.PhotonCameraSim
import org.photonvision.simulation.SimCameraProperties
import java.nio.ByteBuffer
import java.util.concurrent.locks.ReentrantLock
import kotlin.concurrent.thread


class AbsolutePoseProviderInputs {

    var measurement: Array<AbsolutePoseMeasurement?> = arrayOf()
    var observedTags: IntArray = intArrayOf()
    var latestTargetObservation = TargetObservation(Rotation2d.kZero, Rotation2d.kZero)
    var connected = false
}

interface AbsolutePoseProvider {
    fun updateInputs(inputs: AbsolutePoseProviderInputs)
}

data class LimelightMeasurement(
    var poseMeasurement: AbsolutePoseMeasurement? = null,
    var observedTags: IntArray = intArrayOf(),
    var shouldReject: Boolean = false,
) {
    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false

        other as LimelightMeasurement

        if (poseMeasurement != other.poseMeasurement) return false
        if (!observedTags.contentEquals(other.observedTags)) return false
        if (shouldReject != other.shouldReject) return false

        return true
    }

    override fun hashCode(): Int {
        var result = poseMeasurement?.hashCode() ?: 0
        result = 31 * result + observedTags.contentHashCode()
        return result
    }
}

class LimelightPoseProvider(
    private val name: String,
    private val yawGetter: () -> Rotation2d,
    private val velocityGetter: () -> AngularVelocity,
) : AbsolutePoseProvider {
    // References:
    // https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-swerve-pose-estimation
    // https://docs.limelightvision.io/docs/docs-limelight/apis/limelight-lib#4-field-localization-with-megatag

    private var observedTags = intArrayOf()
    private var measurements = arrayOf<AbsolutePoseMeasurement>()
    private var shouldReject: Boolean = false
    private var lock = ReentrantLock()
    private var lastSeenHb: Double = 0.0
    private var isThrottled = false
    private var currentPipeline = SEARCH_PIPELINE
    private var wasIMUChanged = false
    private var cornerCount = 0
    val gyroVelocity: AngularVelocity get() = velocityGetter()
    val gyroAngle: Rotation2d get() = yawGetter()
    private val table = NetworkTableInstance.getDefault().getTable(name)
    private val hbSubscriber = table.getDoubleTopic("hb").subscribe(0.0)
    private val txSubscriber = table.getDoubleTopic("tx").subscribe(0.0)
    private val tySubscriber = table.getDoubleTopic("ty").subscribe(0.0)
    private val tvSubscriber = table.getIntegerTopic("tv").subscribe(0)
    private val megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(doubleArrayOf())
    private val megatag2Subscriber = table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(doubleArrayOf())
    private val targetPoseRobotSpaceSubscriber = table.getDoubleArrayTopic("targetpose_robotspace").subscribe(doubleArrayOf(0.0, 0.0, 0.0))
    private val gyroPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish()
    private val throttlePublisher = table.getIntegerTopic("throttle_set").publish()
    private val imuModePublisher = table.getIntegerTopic("imumode_set").publish()
    private val cropPublisher = table.getDoubleArrayTopic("crop").publish()
    private val pipelinePublisher = table.getIntegerTopic("pipeline").publish()
    private var loopsSinceLastSeen: Int = 0

    init {
        thread(isDaemon = true) {
            while (true) {
                val temp = updateCurrentMeasurement()
                try {
                    lock.lock()
                    for (measurement in temp) {
                        measurements += measurement.poseMeasurement!!
                        observedTags += measurement.observedTags
                    }
                }
                finally {
                    lock.unlock()
                }
                Thread.sleep(Robot.period.toLong())
            }
        }
    }

    private fun updateCurrentMeasurement(): Array<LimelightMeasurement> {
        val measurement = LimelightMeasurement()
        if (Robot.beforeFirstEnable) {
            imuModePublisher.accept(1.toLong())
            gyroPublisher.accept(doubleArrayOf(gyroAngle.degrees, 0.0, 0.0, 0.0, 0.0, 0.0))
            NetworkTableInstance.getDefault().flush()
        }

        for (rawSample in megatag1Subscriber.readQueue()) {
            
        }

//        if ((!Robot.beforeFirstEnable) && currentAlgorithm == LimelightAlgorithm.MegaTag) {
//            currentAlgorithm = megaTagV2
//        }
//
//        when (val algorithm = currentAlgorithm) {
//            is LimelightAlgorithm.MegaTag ->
//                LimelightHelpers.getBotPoseEstimate_wpiBlue(name)?.let { estimate ->
//                    measurement.observedTags = estimate.rawFiducials.mapNotNull { it?.id }.toIntArray()
//
//                    // Reject zero tag or low-quality one tag readings
//                    if (estimate.tagCount == 0) return measurement
//                    if (estimate.tagCount == 1) {
//                        val fiducial = estimate.rawFiducials[0]
//                        if (fiducial == null
//                            || fiducial.ambiguity > AMBIGUITY_THRESHOLD
//                            || fiducial.distToCamera > MAX_SINGLE_TAG_DISTANCE
//                        ) return measurement
//                    }
//
//                    measurement.poseMeasurement = AbsolutePoseMeasurement(
//                        estimate.pose,
//                        estimate.timestampSeconds.seconds,
//                        VecBuilder.fill(.5, .5, .25)
//                    )
//                }
//
//            is LimelightAlgorithm.MegaTag2 -> {
//                LimelightHelpers.SetRobotOrientation(
//                    name,
//                    algorithm.gyroPosition.degrees,
//                    // The Limelight sample code leaves these as zero, and the API docs call them "Unnecessary."
//                    0.0, 0.0, 0.0, 0.0, 0.0
//                )
//
//                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name)?.let { estimate ->
//                    measurement.observedTags = estimate.rawFiducials.mapNotNull { it?.id }.toIntArray()
//                    val highSpeed = algorithm.gyroVelocity.abs(DegreesPerSecond) > 720.0
//                    if (estimate.tagCount == 0 || highSpeed) return measurement
//
//                    measurement.poseMeasurement = AbsolutePoseMeasurement(
//                        estimate.pose,
//                        estimate.timestampSeconds.seconds,
//                        // This value is pulled directly from the Limelight docs (linked at the top of this class)
//                        VecBuilder.fill(.5, .5, 9999999.0)
//                    )
//                }
            }

        }

        return measurement
    }

    override fun updateInputs(inputs: AbsolutePoseProviderInputs) {
        lock.lock()
        inputs.measurement = measurement
        inputs.observedTags = observedTags
        lock.unlock()

        // We assume the camera has disconnected if there are no new updates for several ticks.
        val hb = hbSub.get()
        inputs.connected = hb > lastSeenHb || loopsSinceLastSeen < CONNECTED_TIMEOUT
        if (hb == lastSeenHb)
            loopsSinceLastSeen++
        else
            loopsSinceLastSeen = 0
        lastSeenHb = hb
    }

    companion object {
        /**
         * The acceptable distance for a single-April-Tag reading.
         *
         * This is a somewhat conservative limit, but it is only applied when using the old MegaTag v1 algorithm.
         * It's possible it could be increased if it's too restrictive.
         */
        private val MAX_SINGLE_TAG_DISTANCE = 4.5.meters

        /**
         * The acceptable ambiguity for a single-tag reading on MegaTag v1.
         */
        private const val AMBIGUITY_THRESHOLD = 0.7

        /**
         * The amount of time (in robot ticks) an update before considering the camera to be disconnected.
         */
        private const val CONNECTED_TIMEOUT = 250.0
    }
}
//
//class PhoenixOdometryThread : Thread("PhoenixOdometry") {
//    init {
//        isDaemon = true
//    }
//
//    private val timestampQueues: MutableList<Queue<Double>> = ArrayList<Queue<Double>>()
//    private val signalsLock: Lock = ReentrantLock() // Prevents conflicts when registering signals
//    private var phoenixSignals: Array<BaseStatusSignal> = emptyArray() // Phoenix API does not accept a mutable list
//    private val phoenixQueues = ArrayList<Queue<Double>>()
//
//    override fun start() {
//        if (!timestampQueues.isEmpty()) {
//            super.start()
//        }
//    }
//
//    fun registerSignal(signal: StatusSignal<Angle>): Queue<Double> {
//        val queue: Queue<Double> = ArrayBlockingQueue(20)
//
//        signalsLock.lock()
//        Drivetrain.odometryLock.lock()
//        try {
//            val newSignals = Array(phoenixSignals.size + 1) { i ->
//                if (i < phoenixSignals.size) phoenixSignals[i] else signal
//            }
//            phoenixSignals = newSignals
//            phoenixQueues.add(queue)
//        } finally {
//            signalsLock.unlock()
//            Drivetrain.odometryLock.unlock()
//        }
//
//        return queue
//    }
//
//    fun makeTimestampQueue(): Queue<Double> {
//        val queue = ArrayBlockingQueue<Double>(20)
//        Drivetrain.odometryLock.lock()
//        try {
//            timestampQueues.add(queue)
//        } finally {
//            Drivetrain.odometryLock.unlock()
//        }
//        return queue
//    }
//
//    override fun run() {
//        while (true) {
//            signalsLock.lock()
//            try {
//                if (!phoenixSignals.isEmpty()) {
//                    BaseStatusSignal.waitForAll(2.0 / 250.0, *phoenixSignals)
//                }
//            } catch (e: InterruptedException) {
//                e.printStackTrace()
//            } finally {
//                signalsLock.unlock()
//            }
//
//            Drivetrain.odometryLock.lock()
//            try {
//                var timestamp = RobotController.getFPGATime() / 1e6
//                var latency = 0.0
//                for (signal in phoenixSignals) {
//                    latency += signal.timestamp.latency
//                }
//                if (!phoenixSignals.isEmpty())
//                    timestamp -= latency / phoenixSignals.size
//
//                for (i in 0 ..<phoenixSignals.size) {
//                    phoenixQueues[i].offer(phoenixSignals[i].valueAsDouble)
//                }
//
//                for (i in 0..<timestampQueues.size) {
//                    timestampQueues[i].offer(timestamp)
//                }
//            } finally {
//                Drivetrain.odometryLock.unlock()
//            }
//        }
//    }
//
//    companion object {
//        private val instance = PhoenixOdometryThread()
//
//        fun getInstance(): PhoenixOdometryThread {
//            return instance
//        }
//    }
//}

@Suppress("unused")
class CameraSimPoseProvider(name: String, val chassisToCamera: Transform3d) : AbsolutePoseProvider {
    private val camera = PhotonCamera(name)
    private val simProperties = SimCameraProperties().apply {
        setCalibration(1280, 800, Rotation2d(LIMELIGHT_FOV))
        fps = 20.0
        avgLatencyMs = 51.0
        latencyStdDevMs = 5.0
    }
    val sim = PhotonCameraSim(camera, simProperties)

    private val estimator =
        PhotonPoseEstimator(
            FIELD_LAYOUT,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            chassisToCamera
        )

    override fun updateInputs(inputs: AbsolutePoseProviderInputs) {
        inputs.connected = true
        inputs.measurement = null
        val unreadResults = camera.allUnreadResults
        val latestResult = unreadResults.lastOrNull()
        if (latestResult != null) {
            estimator.update(latestResult).ifPresent {
                inputs.measurement = AbsolutePoseMeasurement(
                    it.estimatedPose.toPose2d(),
                    it.timestampSeconds.seconds,
                    VecBuilder.fill(0.7, 0.7, 9999999.0)
                )
            }
            inputs.observedTags = latestResult.targets.map {
                it.fiducialId
            }.toIntArray()
        }
    }
}

data class AbsolutePoseMeasurement(
    val pose: Pose2d,
    val timestamp: Time,
    /**
     * Standard deviations of the vision pose measurement (x position in meters, y position in meters, and heading in
     * radians). Increase these numbers to trust the vision pose measurement less.
     */
    val stdDeviation: Matrix<N3, N1>
) : StructSerializable {
    companion object {
        @JvmField
        @Suppress("unused")
        val struct = AbsolutePoseMeasurementStruct()
    }
}

fun SwerveDrivePoseEstimator.addAbsolutePoseMeasurement(measurement: AbsolutePoseMeasurement) {
    addVisionMeasurement(
        measurement.pose,
        measurement.timestamp.inSeconds(),
        measurement.stdDeviation
    )
}

class AbsolutePoseMeasurementStruct : Struct<AbsolutePoseMeasurement> {
    override fun getTypeClass(): Class<AbsolutePoseMeasurement> = AbsolutePoseMeasurement::class.java
    override fun getTypeName(): String {
        return "struct:AbsolutePoseMeasurement"
    }

    override fun getTypeString(): String = "struct:AbsolutePoseMeasurement"
    override fun getSize(): Int = Pose3d.struct.size + Struct.kSizeDouble + 3 * Struct.kSizeDouble
    override fun getSchema(): String = "Pose2d pose; double timestamp; double stdDeviation[3];"
    override fun unpack(bb: ByteBuffer): AbsolutePoseMeasurement =
        AbsolutePoseMeasurement(
            pose = Pose2d.struct.unpack(bb),
            timestamp = bb.double.seconds,
            stdDeviation = VecBuilder.fill(bb.double, bb.double, bb.double)
        )

    override fun pack(bb: ByteBuffer, value: AbsolutePoseMeasurement) {
        Pose2d.struct.pack(bb, value.pose)
        bb.putDouble(value.timestamp.inSeconds())
        bb.putDouble(value.stdDeviation[0, 0])
        bb.putDouble(value.stdDeviation[1, 0])
        bb.putDouble(value.stdDeviation[2, 0])
    }
}

val LIMELIGHT_FOV = 75.76079874010732.degrees
