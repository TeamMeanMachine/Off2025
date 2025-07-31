package frc.team2471.off2025.util.localization

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import org.photonvision.targeting.TargetCorner

/**
 * The Measurement class represents a set of odometry and vision measurements. It includes the
 * robot's pose and associated uncertainties, as well as vision measurements from multiple cameras.
 */
// Odom measurement
/**
 * Constructs a new Measurement object with the specified pose.
 *
 * @param m_pose The initial pose of the measurement.
 */
class Measurement(private val m_pose: Pose2d) {
    private val mPoseSigmas: Pose2d = Pose2d(0.1, 0.1, Rotation2d.fromDegrees(0.1)) // TODO: Don't hardcode // TODO: Actually use this

    // Vision measurements
    private val mCameraIds: ArrayList<Int> = ArrayList()
    private val mTargetIds: ArrayList<Int> = ArrayList()

    // Fiducial corner ID is valid only for AprilTags. -1 denotes the center. IDs are numbered as
    // follows:
    // * -> +X  3 ----- 2
    // * |      |       |
    // * V      |       |
    // * +Y     0 ----- 1
    private val mFiducialCornerIds: ArrayList<Int> = ArrayList()
    private val mCorners: ArrayList<TargetCorner> = ArrayList()

    // Default to a moderate uncertainty.
    private var mPixelSigma = 50.0


    /**
     * Adds a vision measurement to the system.
     *
     * @param cameraID The ID of the camera that captured the measurement.
     * @param targetID The ID of the target being measured.
     * @param pixelXY The pixel coordinates of the target corner in the image.
     */
    fun addVisionMeasurement(cameraID: Int, targetID: Int, pixelXY: TargetCorner) {
        addVisionMeasurement(cameraID, targetID, -1, pixelXY)
    }

    /**
     * Adds a vision measurement to the localization system.
     *
     * @param cameraID The ID of the camera that captured the measurement.
     * @param targetID The ID of the target being measured.
     * @param fiducialCornerID The ID of the fiducial corner being measured.
     * @param pixelXY The pixel coordinates of the target corner in the image.
     */
    fun addVisionMeasurement(cameraID: Int, targetID: Int, fiducialCornerID: Int, pixelXY: TargetCorner) {
        mCameraIds.add(cameraID)
        mTargetIds.add(targetID)
        mFiducialCornerIds.add(fiducialCornerID)
        mCorners.add(pixelXY)
    }

    /**
     * Sets the uncertainty of the vision measurement.
     *
     * @param pixelSigma The standard deviation of the pixel measurement noise.
     */
    fun setVisionUncertainty(pixelSigma: Double) {
        mPixelSigma = pixelSigma
    }

    /**
     * Converts the measurement data to an array of doubles. Necessary for sending arbitrary-lengthed
     * data over NetworkTables.
     *
     * @param id The identifier for the measurement.
     * @return A double array containing the measurement data. The array consists of static data
     * followed by vision data for each camera. The structure of the array is as follows: -
     * data[0]: Measurement ID - data[1]: X coordinate of the pose - data[2]: Y coordinate of the
     * pose - data[3]: Rotation in radians of the pose - data[4]: X coordinate sigma of the pose -
     * data[5]: Y coordinate sigma of the pose - data[6]: Rotation sigma in radians of the pose -
     * data[7]: Pixel sigma - For each camera: - data[8 + 5 * i]: Camera ID - data[9 + 5 * i]:
     * Target ID - data[10 + 5 * i]: Fiducial corner ID - data[11 + 5 * i]: X coordinate of the
     * corner - data[12 + 5 * i]: Y coordinate of the corner
     */
    fun toArray(id: Int): DoubleArray {
        val kStaticDataLength = 8
        val kVisionDataLength = 5
        val data = DoubleArray(kStaticDataLength + kVisionDataLength * mCameraIds.size)
        data[0] = id.toDouble()
        data[1] = m_pose.x
        data[2] = m_pose.y
        data[3] = m_pose.rotation.radians
        data[4] = mPoseSigmas.x
        data[5] = mPoseSigmas.y
        data[6] = mPoseSigmas.rotation.radians
        data[7] = mPixelSigma
        for (i in mCameraIds.indices) {
            data[kStaticDataLength + kVisionDataLength * i] = mCameraIds[i].toDouble()
            data[kStaticDataLength + kVisionDataLength * i + 1] = mTargetIds[i].toDouble()
            data[kStaticDataLength + kVisionDataLength * i + 2] = mFiducialCornerIds[i].toDouble()
            data[kStaticDataLength + kVisionDataLength * i + 3] = mCorners[i].x
            data[kStaticDataLength + kVisionDataLength * i + 4] = mCorners[i].y
        }
        return data
    }
}
