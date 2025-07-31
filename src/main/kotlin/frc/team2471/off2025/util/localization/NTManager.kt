package frc.team2471.off2025.util.localization

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.networktables.*
import edu.wpi.first.util.struct.StructBuffer
import frc.team2471.off2025.util.localization.CameraInfo
import frc.team2471.off2025.util.vision.Fiducial
import frc.team2471.off2025.util.vision.QuixVisionCamera
import org.littletonrobotics.junction.AutoLog
import org.littletonrobotics.junction.Logger
import java.util.*
import java.util.concurrent.atomic.AtomicReference

/**
 * The NTManager class is responsible for managing network table interactions related to robot
 * localization. It handles publishing and subscribing to various topics such as measurements,
 * targets, and camera information. It also maintains the latest pose estimate and updates inputs
 * accordingly.
 */
class NTManager {
    private val mLocalizerTable: NetworkTable

    // Combined Robot to DS odometry and vision measurements
    private val mMeasurementsPub: DoubleArrayPublisher

    // Robot to DS targets
    private val mTargetPub: StructArrayPublisher<Fiducial?>

    // Robot to Camera transforms and intrinsics
    private val mCameraInfosPublisher: StructArrayPublisher<CameraInfo?>

    // Use an AtomicReference to make updating the value thread-safe
    private val mLatestPoseEstimate = AtomicReference<PoseEstimate?>()

    private val mInputs = PoseEstimateInputsAutoLogged()

    @AutoLog
    open class PoseEstimateInputs {
        @JvmField var id: Int = 0
        @JvmField var pose: Pose2d = Pose2d()
        @JvmField var hasVision: Boolean = false
    }

    /**
     * Manages the NetworkTables (NT) interactions for localization. This class sets up publishers and
     * subscribers for various localization-related topics.
     *
     *
     * It initializes the following publishers:
     *
     *
     *  * Measurements publisher for double array topic "measurements"
     *  * Targets publisher for struct array topic "targets" with Fiducial structure
     *  * Cameras publisher for struct array topic "cameras" with CameraInfo structure
     *
     *
     *
     * It also sets up a listener for the "estimates" topic to update the latest pose estimate.
     *
     *
     * Usage:
     *
     * <pre>`NTManager ntManager = new NTManager();
    `</pre> *
     */
    init {
        val inst = NetworkTableInstance.getDefault()
        mLocalizerTable = inst.getTable("localizer")
        mMeasurementsPub = mLocalizerTable.getDoubleArrayTopic("measurements").publish(PubSubOption.sendAll(true))
        mTargetPub = mLocalizerTable.getStructArrayTopic<Fiducial?>("targets", Fiducial.struct)
            .publish(PubSubOption.sendAll(true))
        mCameraInfosPublisher = mLocalizerTable.getStructArrayTopic<CameraInfo?>("cameras", CameraInfo.struct)
            .publish(PubSubOption.sendAll(true))

        // Setup listener for when the estimate is updated.
        val estimatesSub = mLocalizerTable.getStructTopic("estimates", PoseEstimate.struct)
            .subscribe(PoseEstimate(), PubSubOption.sendAll(true))
        val poseEstimateStructBuffer = StructBuffer.create(PoseEstimate.struct)
        inst.addListener(estimatesSub, EnumSet.of<NetworkTableEvent.Kind>(NetworkTableEvent.Kind.kValueAll)) { event: NetworkTableEvent? ->
            mLatestPoseEstimate.set(poseEstimateStructBuffer.read(event!!.valueData.value.getRaw()))
        }
    }

    /**
     * Publishes a measurement to the network table.
     *
     * @param measurement The measurement to be published.
     * @param id The identifier for the measurement.
     */
    fun publishMeasurement(measurement: Measurement, id: Int) {
        mMeasurementsPub.set(measurement.toArray(id))
        NetworkTableInstance.getDefault().flush()
    }

    /**
     * Publishes an array of fiducial targets to the network table.
     *
     * @param targets An array of Fiducial objects representing the targets to be published.
     */
    fun publishTargets(targets: Array<Fiducial>) {
        mTargetPub.set(targets)
    }

    /**
     * Publishes information about a list of cameras to the network table.
     *
     * @param cameras An ArrayList of QuixVisionCamera objects representing the cameras to be
     * published. Each camera's transform, camera matrix, and distortion coefficients will be used
     * to create a CameraInfo object, which will then be published.
     */
    fun publishCameras(cameras: ArrayList<QuixVisionCamera>) {
        val infos = ArrayList<CameraInfo?>()
        for (camera in cameras) {
            infos.add(
                CameraInfo(camera.transform, camera.cameraMatrix, camera.distCoeffs)
            )
        }
        val array = arrayOfNulls<CameraInfo>(infos.size)
        infos.toArray<CameraInfo?>(array)
        mCameraInfosPublisher.set(array)
    }

    /**
     * Updates the input values with the latest pose estimate if available.
     *
     *
     * This method retrieves the latest pose estimate and updates the input values with the ID,
     * pose, and vision status from the estimate. If the latest pose estimate is null, the input
     * values are not updated. The updated inputs are then processed by the Logger.
     */
    fun updateInputs() {
        val latestEstimate = mLatestPoseEstimate.get()
        if (latestEstimate != null) {
            mInputs.id = latestEstimate.id
            mInputs.pose = latestEstimate.pose
            mInputs.hasVision = latestEstimate.hasVision()
        }
        Logger.processInputs("Inputs/NTManager", mInputs)
    }

    /** Get the latest estimate over NT.  */
    val latestPoseEstimate: PoseEstimate
        /**
         * Retrieves the latest pose estimate.
         *
         * @return A [PoseEstimate] object containing the latest pose information.
         */
        get() = PoseEstimate(mInputs.id, mInputs.pose, mInputs.hasVision)
}
