package frc.team2471.off2025.util.localization

import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.Timer
import frc.team2471.off2025.util.quix.AlignmentUtilities
import frc.team2471.off2025.util.quix.Fiducials
import frc.team2471.off2025.util.quix.InterpolatableChassisSpeeds
import frc.team2471.off2025.util.vision.Fiducial
import frc.team2471.off2025.util.vision.PipelineVisionPacket
import frc.team2471.off2025.util.vision.QuixVisionCamera
import org.littletonrobotics.junction.Logger
import org.photonvision.targeting.PhotonTrackedTarget
import java.util.*
import java.util.concurrent.ConcurrentSkipListMap
import kotlin.math.abs
import kotlin.math.hypot
import kotlin.math.max

/**
 * The QuixSwerveLocalizer class is responsible for managing the localization of a swerve drive
 * robot. It integrates odometry and vision measurements to provide an accurate estimate of the
 * robot's pose.
 *
 *
 * This class handles:
 *
 *
 *  * Sending and receiving data from NetworkTables
 *  * Tracking AprilTags for vision-based localization
 *  * Maintaining buffers for odometry and chassis speeds
 *  * Updating the robot's pose using odometry and vision measurements
 *  * Publishing measurements and pose estimates to NetworkTables
 *
 *
 *
 * Key components:
 *
 *
 *  * NTManager: Manages NetworkTables communication
 *  * SwerveDriveOdometry: Handles odometry calculations
 *  * TimeInterpolatableBuffer: Buffers for interpolating odometry and chassis speeds
 *  * TreeMap: Stores measurements for processing
 *
 *
 *
 * Usage:
 *
 *
 *  * Initialize with kinematics, initial gyro angle, module positions, initial pose, targets,
 * and cameras
 *  * Call update() method with odometry, vision packets, and chassis speeds to update the pose
 *  * Use getPose() to retrieve the latest pose estimate
 *  * Use resetPose() to reset the localizer to a specific pose
 *
 *
 *
 * Note: This class assumes that the robot is equipped with vision cameras and AprilTags for
 * localization.
 *
 * @param kinematics The kinematics of the swerve drive.
 * @param initialGyroAngle The initial angle of the gyro.
 * @param modulePositions The positions of the swerve modules.
 * @param initialPose The initial pose of the robot.
 * @param targets The fiducial targets for vision-based localization.
 * @param cameras The vision cameras used for detecting fiducial targets.
 */
class QuixSwerveLocalizer(
    kinematics: SwerveDriveKinematics,
    initialGyroAngle: Rotation2d,
    modulePositions: Array<SwerveModulePosition>,
    initialPose: Pose2d,
    targets: Array<Fiducial>,
    val cameras: ArrayList<QuixVisionCamera>
) {
    // Manages sending and receiving from NetworkTables.
    private val networktable = NTManager()

    // If empty, uses all tags.
    private val tagsToTrack = HashSet<Int>()

    // ID of the current measurement. Used to sync between Robot and DriverStation.
    private var currentId = 0

    // Map of {id: time}
    private val idToTimeMap = HashMap<Int, Double>()

    // Map of {time : SwerveDriveOdometryMeasurement}
    private val timeToOdometryMap = ConcurrentSkipListMap<Double, SwerveOdometryMeasurement>()

    // Buffer of poses so we can get the interpolated pose at the time of a vision measurement.
    private val kBufferHistorySeconds = 10.0 // s
    private val rawOdometryPoseBuffer = TimeInterpolatableBuffer.createBuffer<Pose2d>(kBufferHistorySeconds)

    // Buffer of chassis speeds so we can get the interpolated chassis speed at the time of a vision
    // measurement.
    private val chassisSpeedsBuffer = TimeInterpolatableBuffer.createBuffer<InterpolatableChassisSpeeds>(kBufferHistorySeconds)

    // Map of {time: Pair<NTOdometryMeasurement, NTVisionMeasurement>}
    private val timeToMeasurementMap = TreeMap<Double, Measurement>()

    // ID of the last measurement that was updated.
    private var lastUpdatedId = -1

    // Continuous odometry from the last reset. Used as input to the localizer.
    private val rawOdometry = SwerveDriveOdometry(kinematics, initialGyroAngle, modulePositions, initialPose)

    // Odometry played back on top of the latest localization estimate.
    private val playbackOdometry = SwerveDriveOdometry(kinematics, initialGyroAngle, modulePositions, initialPose)

    // Odometry played back on top of the latest single-tag localization estimate.
    private val singleTagPlaybackOdometry = SwerveDriveOdometry(kinematics, initialGyroAngle, modulePositions, initialPose)

    // Latest raw localization estimate from DS.
    private var latestRawEstimate = PoseEstimate()

    // Measurements within |kMutableTimeBuffer| of the current time are not considered final.
    // This gives us a chance to associate new vision measurements with an past interpolated
    // odometry measurements.
    private val kMutableTimeBuffer = 0.05 // seconds

    init {
        println("created Localizer with initialGyroAngle: $initialGyroAngle and initialPose: $initialPose")

        println("playback odom ${playbackOdometry.poseMeters}")

        networktable.publishTargets(targets)
        trackAllTags()
    }

    /** Resets the localizer to the given pose.  */
    fun resetPose(gyroAngle: Rotation2d, modulePositions: Array<SwerveModulePosition>, pose: Pose2d) {
        println("reset pose to gyro: $gyroAngle pose: $pose")
        rawOdometry.resetPosition(gyroAngle, modulePositions, pose)
        playbackOdometry.resetPosition(gyroAngle, modulePositions, pose)
        singleTagPlaybackOdometry.resetPosition(gyroAngle, modulePositions, pose)
    }

    fun trackAllTags() {
        tagsToTrack.clear()
        for (tag in Fiducials.aprilTagFiducials) {
            tagsToTrack.add(tag.id)
        }
    }

    fun setTagsToTrack(tagIDs: IntArray) {
        tagsToTrack.clear()
        for (id in tagIDs) {
            tagsToTrack.add(id)
        }
    }

    val odometryPose: Pose2d
        /** Raw odometry pose.  */
        get() = rawOdometry.poseMeters

    val pose: Pose2d
        /** Localizer latency-compensated pose.  */
        get() = playbackOdometry.poseMeters

    val singleTagPose: Pose2d
        /** Single tag latency-compensated pose.  */
        get() = singleTagPlaybackOdometry.poseMeters

    val rawPose: Pose2d
        /** Localizer pose from DS. Use for plotting/debugging only.  */
        get() = latestRawEstimate.pose ?: Pose2d()

    /** Update with odometry and optional vision.  */
    fun update(odometry: SwerveOdometryMeasurement, visionPackets: ArrayList<PipelineVisionPacket>, chassisSpeeds: ChassisSpeeds) {
        networktable.publishCameras(cameras)

        val startTimestamp = Timer.getFPGATimestamp()
        val currentTime = Timer.getTimestamp()

        rawOdometry.update(odometry.gyroAngle, odometry.modulePositionStates)
        playbackOdometry.update(odometry.gyroAngle, odometry.modulePositionStates)
        singleTagPlaybackOdometry.update(odometry.gyroAngle, odometry.modulePositionStates)
        timeToOdometryMap.put(currentTime, odometry)

        val curPose = rawOdometry.poseMeters
        rawOdometryPoseBuffer.addSample(currentTime, curPose)
        chassisSpeedsBuffer.addSample(currentTime, InterpolatableChassisSpeeds.fromChassisSpeeds(chassisSpeeds))

        // Always save latest odometry.
        timeToMeasurementMap.put(currentTime, Measurement(curPose))

        // Save data from each camera.
        for (cameraID in visionPackets.indices) {
            val detectedTags = ArrayList<Translation3d>()
            val vision = visionPackets[cameraID]

            val measurementTime = vision.captureTimestamp
            if (measurementTime > 0.0) {
                Logger.recordOutput("Localizer/measurementLatency[$cameraID]", currentTime - measurementTime)
            }

            if (!vision.hasTargets) {
                val array = arrayOfNulls<Translation3d>(0)
                Logger.recordOutput<Translation3d>("Localizer/detectedTags[$cameraID]", *array)
                continue
            }

            // Merge with the existing measurement if it already exists.
            var existingMeasurement = timeToMeasurementMap.get(measurementTime)

            // If there is no existing measurement, create a new one by interpolating pose.
            if (existingMeasurement == null) {
                val interpolatedPose = rawOdometryPoseBuffer.getSample(measurementTime).get()
                existingMeasurement = Measurement(interpolatedPose)
                timeToMeasurementMap.put(measurementTime, existingMeasurement)
            }

            // Set vision uncertainty based on chassis speeds.
            // The fast we are moving, the more uncertain we are.
            // TODO: Tune
            val interpolatedChassisSpeeds = chassisSpeedsBuffer.getSample(measurementTime).get()
            val pixelSigma: Double = max(
                100.0,
                5.0
                        + 10.0 * hypot(interpolatedChassisSpeeds.vxMetersPerSecond, interpolatedChassisSpeeds.vyMetersPerSecond)
                        + 20.0 * abs(interpolatedChassisSpeeds.omegaRadiansPerSecond)
            )
            existingMeasurement.setVisionUncertainty(pixelSigma)

            if (vision.targets != null) {
                for (target in vision.targets) {
                    if (tagsToTrack.contains(target.getFiducialId())) {
                        // Use AprilTag corners.
                        for (cornerID in target.getDetectedCorners().indices) {
                            existingMeasurement.addVisionMeasurement(
                                cameraID,
                                target.getFiducialId(),
                                cornerID,
                                target.getDetectedCorners()[cornerID]
                            )
                        }
                        if (target.getFiducialId() <= Fiducials.aprilTagFiducials.size) {
                            detectedTags.add(Pose3d(this.pose).transformBy(cameras[cameraID].transform).translation)
                            detectedTags.add(Fiducials.aprilTagFiducials[target.getFiducialId() - 1].pose.translation)
                        }
                    }
                }
            }
            val array = arrayOfNulls<Translation3d>(detectedTags.size)
            detectedTags.toArray<Translation3d>(array)
            Logger.recordOutput<Translation3d>("Localizer/detectedTags[$cameraID]", *array)
        }
        publishImmutableEntries()
        val endTimestamp = Timer.getFPGATimestamp()
        Logger.recordOutput("Localizer/Update seconds", (endTimestamp - startTimestamp))

        val singleStartTime = Timer.getFPGATimestamp()
        computeSingleTagPose()
        Logger.recordOutput("Localizer/SingleTag calc time", Timer.getFPGATimestamp() - singleStartTime)
    }


    /**
     * Uses the latest pose estimate over NetworkTables and replays the latest odometry on top of it.
     */
    fun updateWithLatestPoseEstimate() {
        val startTimestamp = Timer.getFPGATimestamp()
        networktable.updateInputs()
        val estimate = networktable.latestPoseEstimate

        // Save for plotting/debugging purposes.
        latestRawEstimate = estimate

        // Only incorporate estimate if it is new.
        if (idToTimeMap.isEmpty() || estimate.id == lastUpdatedId || idToTimeMap[estimate.id] == null || estimate.pose == null) {
            Logger.recordOutput("Localizer/UpdateWithLatestPoseEstimate seconds", (Timer.getFPGATimestamp() - startTimestamp))
            Logger.recordOutput("Localizer/visionCorrection (m)", 0.0)
            return
        }
        lastUpdatedId = estimate.id

        // Save the pose before correction.
        val preCorrectionPose = this.pose

        // Start playback odometry at the first time >= the current estimate.
        val estimateTime: Double = idToTimeMap[estimate.id]!!
        var curTime = timeToOdometryMap.ceilingKey(estimateTime)

        val measurement: SwerveOdometryMeasurement = timeToOdometryMap[curTime]!!
        playbackOdometry.resetPosition(measurement.gyroAngle, measurement.modulePositionStates, estimate.pose)

        // Traverse entries in |m_timeToOdometryMap| from |curTime| until the end to update
        // playback odometry.
        while (curTime != null) {
            val lastMeasurement: SwerveOdometryMeasurement = timeToOdometryMap.get(curTime)!!
            playbackOdometry.update(lastMeasurement.gyroAngle, lastMeasurement.modulePositionStates)
            curTime = timeToOdometryMap.higherKey(curTime)
        }
        Logger.recordOutput("Localizer/UpdateWithLatestPoseEstimate seconds", (Timer.getFPGATimestamp() - startTimestamp))

        // Log magnitude of correction
        val postCorrectionPose = this.pose
        Logger.recordOutput("Localizer/visionCorrection (m)", postCorrectionPose.minus(preCorrectionPose).translation.norm)
    }

    /** Handles NT publishing, ID finalization, and cleanup.  */
    private fun publishImmutableEntries() {
        val currentTime = Timer.getTimestamp()

        // Times are in ascending order.
        val times = ArrayList(timeToMeasurementMap.keys)
        for (time in times) {
            // Entries within |kMutableTimeBuffer| of the current time are not considered final.
            // Once we reach this point we are done.
            if (currentTime - time < kMutableTimeBuffer) {
                break
            }

            // Entries older than |kMutableTimeBuffer| are considered immutable.
            // Assign them an ID and publish them.
            val measurement: Measurement = timeToMeasurementMap[time]!!
            networktable.publishMeasurement(measurement, currentId)
            timeToMeasurementMap.remove(time)

            idToTimeMap.put(currentId, time)
            currentId += 1
        }
    }


    // Do single-tag 3D-distance + angle based localization
    // Based on
    // https://github.com/Mechanical-Advantage/RobotCode2025Public/blob/c2bb0e79c466be33074577d51243258ec3d39f44/src/main/java/org/littletonrobotics/frc2025/RobotState.java#L194
    // TODO: This is semi-hardcoded for 2025
    private fun computeSingleTagPose() {
        if (DriverStation.getAlliance().isEmpty) {
            return
        }

        val tagID = AlignmentUtilities.determineClosestTagID(this.pose, DriverStation.getAlliance().get() == Alliance.Blue)

        // Get latest measurement from either camera with a target.
        var latestTimestamp = 0.0
        var latestTarget: PhotonTrackedTarget? = null
        var cam: QuixVisionCamera? = null
        for (camera in cameras) {
            val measurement = camera.latestMeasurement
            if (measurement.captureTimestamp <= latestTimestamp) {
                continue
            }

            if (measurement.targets != null) {
                for (target in measurement.targets) {
                    if (target.fiducialId != tagID) {
                        continue
                    }
                    latestTimestamp = measurement.captureTimestamp
                    latestTarget = target
                    cam = camera
                }
            }
        }

        if (latestTarget == null || cam == null) {
            return
        }

        // Compute single tag pose
        val interpolatedPose = rawOdometryPoseBuffer.getSample(latestTimestamp).get()
        val interpolatedRotation = interpolatedPose.rotation
        val distance = latestTarget.bestCameraToTarget.translation.norm
        val robotToCam = cam.transform
        val camToTagTranslation = Pose3d(Translation3d.kZero, Rotation3d(0.0, Math.toRadians(-latestTarget.getPitch()), Math.toRadians(-latestTarget.getYaw())))
            .transformBy(Transform3d(Translation3d(distance, 0.0, 0.0), Rotation3d.kZero)).translation
            .rotateBy(Rotation3d(robotToCam.rotation.getX(), robotToCam.rotation.getY(), 0.0)).toTranslation2d()
        val camToTagRotation = interpolatedRotation.plus(robotToCam.rotation.toRotation2d().plus(camToTagTranslation.angle))

        val tagPose2d = Fiducials.aprilTagFiducials[latestTarget.fiducialId - 1].pose.toPose2d()
        if (tagPose2d == null) {
            return
        }

        val fieldToCameraTranslation = Pose2d(tagPose2d.translation, camToTagRotation.plus(Rotation2d.kPi))
                .transformBy(Transform2d(camToTagTranslation.norm, 0.0, Rotation2d.kZero)).translation
        val cameraPose = Pose3d.kZero.transformBy(robotToCam)
        var robotPose = Pose2d(fieldToCameraTranslation, interpolatedRotation.plus(cameraPose.toPose2d().rotation))
            .transformBy(Transform2d(cameraPose.toPose2d(), Pose2d.kZero))
        // Use gyro angle at time for robot rotation
        robotPose = Pose2d(robotPose.translation, interpolatedRotation)

        Logger.recordOutput("Swerve/SingleTagPoseRaw", robotPose)

        // Replay odometry on top of latest estimate
        var curTime = timeToOdometryMap.ceilingKey(latestTimestamp)
        if (curTime == null) {
            return
        }
        val measurement: SwerveOdometryMeasurement = timeToOdometryMap[curTime]!!
        singleTagPlaybackOdometry.resetPosition(measurement.gyroAngle, measurement.modulePositionStates, robotPose)

        // Traverse entries in |m_timeToOdometryMap| from |curTime| until the end to update
        // playback odometry.
        while (curTime != null) {
            val lastMeasurement: SwerveOdometryMeasurement = timeToOdometryMap[curTime]!!
            singleTagPlaybackOdometry.update(lastMeasurement.gyroAngle, lastMeasurement.modulePositionStates)
            curTime = timeToOdometryMap.higherKey(curTime)
        }
    }

    class SwerveOdometryMeasurement(val gyroAngle: Rotation2d?, val modulePositionStates: Array<SwerveModulePosition>)
}
