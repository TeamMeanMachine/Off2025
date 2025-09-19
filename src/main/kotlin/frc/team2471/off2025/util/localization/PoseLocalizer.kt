package frc.team2471.off2025.util.localization

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.Timer
import frc.team2471.off2025.util.control.LoopLogger
import frc.team2471.off2025.util.vision.Fiducial
import frc.team2471.off2025.util.vision.Fiducials
import frc.team2471.off2025.util.vision.PipelineVisionPacket
import frc.team2471.off2025.util.vision.QuixVisionCamera
import org.littletonrobotics.junction.Logger
import org.photonvision.targeting.PhotonTrackedTarget
import java.util.ArrayList
import java.util.HashMap
import java.util.HashSet
import java.util.NavigableMap
import java.util.TreeMap
import kotlin.jvm.optionals.getOrNull
import kotlin.math.abs
import kotlin.math.hypot
import kotlin.math.max

/**
 *  Class that handles combining vision, Quest, and odometry data into a single pose estimate. Inspired by Team 604's QuixSwerveLocalizer
 *
 *  Swerve odometry measurements are treated as relative pose sources with no latency. Measurements get continuously added onto each other.
 *
 *  Quest measurements are treated as an "absolute relative" pose source and correct for odometry drift. They offset the estimated pose by the difference between Quest pose change and odometry pose change.
 *
 *  Particle filter and single tag measurements are used as absolute pose sources. They offset the estimated pose by the camera estimate
 */
class PoseLocalizer(targets: Array<Fiducial>, val cameras: List<QuixVisionCamera>) {
    private val networkTable = NTManager()

    // If empty, uses all tags.
    private val tagsToTrack = HashSet<Int>()

    // Buffer of poses so we can get the interpolated pose at the time of a vision measurement.
    private val bufferHistorySeconds = 10.0
    // Buffer of only the swerve odometry pose.
    private val rawOdometryPoseBuffer = TimeInterpolatableBuffer.createBuffer<Pose2d>(bufferHistorySeconds)
    // Buffer of swerve odometry pose fused with Quest pose.
    private val fusedOdometryBuffer = TimeInterpolatableBuffer.createBuffer<Pose2d>(bufferHistorySeconds)
    // Buffer of swerve odometry, Quest, and single tag pose.
    private val singleTagOdometryBuffer = TimeInterpolatableBuffer.createBuffer<Pose2d>(bufferHistorySeconds)
    // Buffer of the swerve odometry, Quest, and vision measurements.
    private val visionOdometryBuffer = TimeInterpolatableBuffer.createBuffer<Pose2d>(bufferHistorySeconds)

    // Buffer of chassis speeds so we can get the interpolated chassis speed at the time of a vision measurement.
    private val chassisSpeedsBuffer = TimeInterpolatableBuffer.createBuffer<InterpolatableChassisSpeeds>(bufferHistorySeconds)


    // ID of the current measurement. Used to sync between Robot and Particle Filter.
    private var currentId = 0

    // Map of {id: time}
    private val idToTimeMap = HashMap<Int, Double>()

    // Map of {time: Pair<NTOdometryMeasurement, NTVisionMeasurement>}
    private val timeToMeasurementMap = TreeMap<Double, Measurement>()

    // ID of the last measurement that was updated.
    private var lastUpdatedId = -1

    // Latest raw localization estimate from Particle Filter.
    private var latestRawEstimate = PoseEstimate()

    // Measurements within |kMutableTimeBuffer| of the current time are not considered final.
    // This gives us a chance to associate new vision measurements with past interpolated odometry measurements.
    private val kMutableTimeBuffer = 0.05 // seconds

    var lastQuestMeasurement: QuestNavMeasurement? = null

    /** Only Swerve odometry */
    val rawOdometryPose: Pose2d
        get() = rawOdometryPoseBuffer.internalBuffer.lastEntry()?.value ?: Pose2d()
    /** Latest pose reported by the Quest */
    val rawQuestPose: Pose2d?
        get() = lastQuestMeasurement?.robotPose
    /** Swerve odometry fused with Quest measurements */
    val fusedOdometryPose: Pose2d
        get() = fusedOdometryBuffer.internalBuffer.lastEntry()?.value ?: Pose2d()
    /** Pose that only uses the closest apriltag to correct itself. Used for precision but needs good tag visibility. */
    val singleTagPose: Pose2d
        get() = singleTagOdometryBuffer.internalBuffer.lastEntry()?.value ?: Pose2d()
    /** Pose from the particle filter, used for full field localization but can be more jittery. */
    val pose: Pose2d
        get() = visionOdometryBuffer.internalBuffer.lastEntry()?.value ?: Pose2d()

    /** Lastest estimate from the particle filter, before latency compensation. */
    val rawPose: Pose2d
        get() = latestRawEstimate.pose ?: Pose2d()


    fun resetPose(pose: Pose2d) {
        clearAllBuffers()

        val currentTime = Timer.getTimestamp()
        rawOdometryPoseBuffer.addSample(currentTime, pose)
        fusedOdometryBuffer.addSample(currentTime, pose)
        singleTagOdometryBuffer.addSample(currentTime, pose)
        visionOdometryBuffer.addSample(currentTime, pose)
    }

    fun resetRotation(rotation: Rotation2d) {
        val storedRawOdomTranslation = rawOdometryPose.translation
        val storedFusedTranslation = fusedOdometryPose.translation
        val storedSingleTagTranslation = singleTagPose.translation
        val storedVisionOdometryTranslation = pose.translation

        clearAllBuffers()

        val currentTime = Timer.getTimestamp()
        rawOdometryPoseBuffer.addSample(currentTime, Pose2d(storedRawOdomTranslation, rotation))
        fusedOdometryBuffer.addSample(currentTime, Pose2d(storedFusedTranslation, rotation))
        singleTagOdometryBuffer.addSample(currentTime, Pose2d(storedSingleTagTranslation, rotation))
        visionOdometryBuffer.addSample(currentTime, Pose2d(storedVisionOdometryTranslation, rotation))
    }

    private fun clearAllBuffers() {
        rawOdometryPoseBuffer.clear()
        fusedOdometryBuffer.clear()
        singleTagOdometryBuffer.clear()
        visionOdometryBuffer.clear()

        lastQuestMeasurement = null
    }

    /**
     * Uses the latest pose estimate over NetworkTables and replays the latest odometry on top of it.
     *
     * Also integrates the latest quest pose and corrects the odometry pose.
     */
    fun updateWithLatestPoseEstimate(questEstimate: QuestNavMeasurement? = null) {
        val startTimestamp = Timer.getFPGATimestamp()

        LoopLogger.record("Before Quest correction")
        // Apply quest data to the buffers
        val prevQuestEstimateTime = lastQuestMeasurement?.dataTimestamp
        if (questEstimate != null && prevQuestEstimateTime != null && questEstimate.dataTimestamp > prevQuestEstimateTime && rawOdometryPoseBuffer.internalBuffer.firstKey() < prevQuestEstimateTime && questEstimate.dataTimestamp < rawOdometryPoseBuffer.internalBuffer.lastKey()) {
            val questEstimateTime = questEstimate.dataTimestamp
            val questDeltaPose = questEstimate.robotPose.minus(rawQuestPose)
            val odometryDeltaPose = rawOdometryPoseBuffer.getSample(questEstimateTime).getOrNull()?.minus(rawOdometryPoseBuffer.getSample(prevQuestEstimateTime).getOrNull())

            if (odometryDeltaPose != null) {
                val questCorrectionDelta = Transform2d(questDeltaPose.translation - odometryDeltaPose.translation, questDeltaPose.rotation - odometryDeltaPose.rotation)
                Logger.recordOutput("Localizer/Quest/questCorrectionDelta", questCorrectionDelta)

                fusedOdometryBuffer.internalBuffer.offsetFutureSamplesBy(questCorrectionDelta, questEstimateTime)
                singleTagOdometryBuffer.internalBuffer.offsetFutureSamplesBy(questCorrectionDelta, questEstimateTime)
                visionOdometryBuffer.internalBuffer.offsetFutureSamplesBy(questCorrectionDelta, questEstimateTime)
            } else {
                println("quest odometryDeltaPose = null")
            }
        }
        lastQuestMeasurement = questEstimate
        LoopLogger.record("After Quest correction")
        Logger.recordOutput("Localizer/Quest/QuestTimestamp", lastQuestMeasurement?.dataTimestamp ?: 0.0)
        Logger.recordOutput("Localizer/Quest/LatestOdomTimestamp", rawOdometryPoseBuffer.internalBuffer.lastKey() ?: 0.0)

        // Apply Vision data to buffers

        networkTable.updateInputs()
        LoopLogger.record("After nt updateInputs")

        val estimate = networkTable.latestPoseEstimate

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

        val visionEstimateTime: Double = idToTimeMap[estimate.id]!!
        if (visionOdometryBuffer.internalBuffer.firstKey() > visionEstimateTime) {
//            println("vision measurement is too far in the past")
            return
        }
        val odometryAtEstimateTime = visionOdometryBuffer.getSample(visionEstimateTime).getOrNull()
        if (odometryAtEstimateTime == null) {
            println("odometryAtEstimateTime is null")
            return
        }
        val odometryPoseOffset = estimate.pose.minus(odometryAtEstimateTime)
        visionOdometryBuffer.internalBuffer.offsetFutureSamplesBy(odometryPoseOffset, visionEstimateTime)

        Logger.recordOutput("Localizer/UpdateWithLatestPoseEstimate seconds", (Timer.getFPGATimestamp() - startTimestamp))

        // Log magnitude of correction
        val postCorrectionPose = this.pose
        Logger.recordOutput("Localizer/visionCorrection (m)", postCorrectionPose.minus(preCorrectionPose).translation.norm)
    }

    fun update(odometryPose: Pose2d, visionPackets: List<PipelineVisionPacket>, chassisSpeeds: ChassisSpeeds) {
        networkTable.publishCameras(cameras)
        LoopLogger.record("After nt publishCameras")

        val startTimestamp = Timer.getFPGATimestamp()
        val currentTime = Timer.getTimestamp()

        val poseDelta = odometryPose.minus(rawOdometryPose)

        // Add odometry pose delta to all buffers that use it
        val currOdomPose = rawOdometryPose.plus(poseDelta)
        val currFusedPose = fusedOdometryPose.plus(poseDelta)
        val currSingleTagPose = singleTagPose.plus(poseDelta)
        val currVisionPose = pose.plus(poseDelta)

        rawOdometryPoseBuffer.addSample(currentTime, currOdomPose)
        fusedOdometryBuffer.addSample(currentTime, currFusedPose)
        singleTagOdometryBuffer.addSample(currentTime, currSingleTagPose)
        visionOdometryBuffer.addSample(currentTime, currVisionPose)

        chassisSpeedsBuffer.addSample(currentTime, InterpolatableChassisSpeeds.fromChassisSpeeds(chassisSpeeds))

        timeToMeasurementMap[currentTime] = Measurement(currFusedPose) // Using fused pose for the particle filter odometry reference

        LoopLogger.record("After adding odom samples")

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
//                println("measurementTime: $measurementTime  lastQuestTimestamp: ${lastQuestMeasurement?.dataTimestamp}")
                existingMeasurement = Measurement(fusedOdometryBuffer.getSample(measurementTime).get())
                timeToMeasurementMap[measurementTime] = existingMeasurement
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
        LoopLogger.record("After camera for loop")
        publishImmutableEntries()
        LoopLogger.record("After pubImmutableEntries()")
        val endTimestamp = Timer.getFPGATimestamp()
        Logger.recordOutput("Localizer/Update seconds", (endTimestamp - startTimestamp))

        val singleStartTime = Timer.getFPGATimestamp()
        computeSingleTagPose()
        LoopLogger.record("After computeSingleTagPose()")
        Logger.recordOutput("Localizer/SingleTag calc time", Timer.getFPGATimestamp() - singleStartTime)
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
            networkTable.publishMeasurement(measurement, currentId)
            timeToMeasurementMap.remove(time)

            idToTimeMap[currentId] = time
            currentId += 1
        }
    }

    // Do single-tag 3D-distance + angle based localization
    // Based on
    // https://github.com/Mechanical-Advantage/RobotCode2025Public/blob/c2bb0e79c466be33074577d51243258ec3d39f44/src/main/java/org/littletonrobotics/frc2025/RobotState.java#L194
    // TODO: This is semi-hardcoded for 2025
    private fun computeSingleTagPose() {
        if (DriverStation.getAlliance().isEmpty) {
            Logger.recordOutput("Localizer/DetectedSingleTag", *arrayOf<Translation2d>())
            return
        }

        val tagID = Fiducials.determineClosestTagID(this.pose, DriverStation.getAlliance().get() == Alliance.Blue)

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
            Logger.recordOutput("Localizer/DetectedSingleTag", *arrayOf<Translation2d>())
            return
        }
        if (fusedOdometryBuffer.internalBuffer.firstKey() > latestTimestamp) {
            Logger.recordOutput("Localizer/DetectedSingleTag", *arrayOf<Translation2d>())
//            println("single tag result is too far in the past")
            return
        }

        // Compute single tag pose
        val interpolatedPose = fusedOdometryBuffer.getSample(latestTimestamp).get()
        val interpolatedRotation = interpolatedPose.rotation
        val distance = latestTarget.bestCameraToTarget.translation.norm
        val robotToCam = cam.transform
        val camToTagTranslation = Pose3d(Translation3d.kZero, Rotation3d(0.0, Math.toRadians(-latestTarget.getPitch()), Math.toRadians(-latestTarget.getYaw())))
            .transformBy(Transform3d(Translation3d(distance, 0.0, 0.0), Rotation3d.kZero)).translation
            .rotateBy(Rotation3d(robotToCam.rotation.getX(), robotToCam.rotation.getY(), 0.0)).toTranslation2d()
        val camToTagRotation = interpolatedRotation.plus(robotToCam.rotation.toRotation2d().plus(camToTagTranslation.angle))

        val tagPose2d = Fiducials.aprilTagFiducials[latestTarget.fiducialId - 1].pose.toPose2d()
        if (tagPose2d == null) {
            Logger.recordOutput("Localizer/DetectedSingleTag", *arrayOf<Translation2d>())
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


        val odometryAtEstimateTime = singleTagOdometryBuffer.getSample(latestTimestamp).getOrNull()
        if (odometryAtEstimateTime == null) {
            println("single tag odometryAtEstimateTime is null")
            Logger.recordOutput("Localizer/DetectedSingleTag", *arrayOf<Translation2d>())
            return
        }
        val odometryPoseOffset = robotPose.minus(odometryAtEstimateTime)
        singleTagOdometryBuffer.internalBuffer.offsetFutureSamplesBy(odometryPoseOffset, latestTimestamp)
        Logger.recordOutput("Localizer/DetectedSingleTag", *arrayOf(tagPose2d.translation))
    }

    init {
        println("Created PoseLocalizer with ${targets.size} targets and ${cameras.size} cameras")

        networkTable.publishTargets(targets)

        trackAllTags()
    }

    fun trackAllTags() {
        tagsToTrack.clear()
        for (tag in Fiducials.aprilTagFiducials) {
            tagsToTrack.add(tag.id)
        }
    }

    data class QuestNavMeasurement(val robotPose: Pose2d, val dataTimestamp: Double)

    /**
     *  Applies a transform to all pose samples with a timestamp >= [timestamp].
     *
     *  @param offset The offset to apply. In meters
     *  @param timestamp The oldest timestamp to start applying the offset at. In seconds.
     *
     *  @see NavigableMap.tailMap
     */
    fun NavigableMap<Double, Pose2d>.offsetFutureSamplesBy(offset: Transform2d, timestamp: Double) {
        // Exit early if the offset is zero
        if (offset.translation.x == 0.0 && offset.translation.y == 0.0 && offset.rotation.radians == 0.0) return

        // I think using a tailMap is faster
        tailMap(timestamp, true).replaceAll { _, pose -> pose.plus(offset) }

//        var higherKey = ceilingKey(timestamp)
//        while (higherKey != null) {
//            this[higherKey] = this[higherKey]!!.plus(offset)
//            higherKey = this.higherKey(higherKey)
//        }
    }
}