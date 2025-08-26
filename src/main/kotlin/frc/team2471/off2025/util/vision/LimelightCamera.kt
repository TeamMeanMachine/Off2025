package frc.team2471.off2025.util.vision

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.numbers.N8
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.Timer
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.photonvision.PhotonCamera
import org.photonvision.simulation.PhotonCameraSim
import org.photonvision.targeting.PhotonPipelineResult
import org.photonvision.targeting.PhotonTrackedTarget
import org.photonvision.targeting.TargetCorner
import java.util.Optional

class LimelightCamera(
    val cameraName: String,
    override val transform: Transform3d,
    override val cameraMatrix: Optional<Matrix<N3, N3>>,
    override val distCoeffs: Optional<Matrix<N8, N1>>
): QuixVisionCamera {
    private val pipelineConfigs = arrayOf(PipelineConfig())
    override val cameraSim: PhotonCameraSim = PhotonCameraSim(PhotonCamera(cameraName), pipelineConfigs[0].simCameraProp)

    private val loggingName: String = "Inputs/LimelightCamera [$cameraName]"

    private val inputs = LimelightCameraInputs()


    class LimelightCameraInputs : LoggableInputs {
        var latestResult: PhotonPipelineResult = PhotonPipelineResult()

        override fun toLog(table: LogTable) {
            table.put("Latest Result", latestResult)
        }

        override fun fromLog(table: LogTable) {
            latestResult = table.get("Latest Result", latestResult)
        }
    }

    override fun updateInputs() {
        val camToTag = LimelightHelpers.getTargetPose_CameraSpace(cameraName)
        val robotToTag = Transform3d(Pose3d(), LimelightHelpers.getTargetPose3d_RobotSpace(cameraName))
        val rawDetections = LimelightHelpers.getRawDetections(cameraName)

        val rawDetection = if (rawDetections.isNotEmpty()) rawDetections.first() else null
        if (camToTag.isNotEmpty() && rawDetection != null) {
            val target: PhotonTrackedTarget = PhotonTrackedTarget(
                camToTag[4],
                camToTag[3],
                LimelightHelpers.getTA(cameraName),
                camToTag[5],
                LimelightHelpers.getFiducialID(cameraName).toInt(),
                0,
                0.0F,
                robotToTag,
                robotToTag,
                0.0,
                mutableListOf<TargetCorner>(
                    TargetCorner(rawDetection.corner0_X, rawDetection.corner0_Y),
                    TargetCorner(rawDetection.corner1_X, rawDetection.corner1_Y),
                    TargetCorner(rawDetection.corner2_X, rawDetection.corner2_Y),
                    TargetCorner(rawDetection.corner3_X, rawDetection.corner3_Y),
                ),
                mutableListOf<TargetCorner>(
                    TargetCorner(rawDetection.corner0_X, rawDetection.corner0_Y),
                    TargetCorner(rawDetection.corner1_X, rawDetection.corner1_Y),
                    TargetCorner(rawDetection.corner2_X, rawDetection.corner2_Y),
                    TargetCorner(rawDetection.corner3_X, rawDetection.corner3_Y),
                )
            )
            val tl = LimelightHelpers.getLatency_Pipeline(cameraName) * 1000
            val cl = LimelightHelpers.getLatency_Capture(cameraName) * 1000
            inputs.latestResult = PhotonPipelineResult(
                0.0.toLong(),
                (RobotController.getTime() - tl - cl).toLong(),
                (RobotController.getTime()).toLong(),
                0.0.toLong(),
                mutableListOf<PhotonTrackedTarget>(target)
            )
        } else {
            inputs.latestResult = PhotonPipelineResult()
        }

        Logger.processInputs(loggingName, inputs)
    }

    override fun setPipelineIndex(index: Int) {}

    override val pipelineConfig: PipelineConfig
        get() = pipelineConfigs[0]

    override val fiducialType: Fiducial.Type
        get() = pipelineConfig.fiducialType

    override val latestMeasurement: PipelineVisionPacket
        get() {
            val startTimestamp = Timer.getFPGATimestamp()
            val result = inputs.latestResult
            val hasTargets = result.hasTargets()
            if (!hasTargets) {
                return PipelineVisionPacket(false, null, null, -1.0)
            }

            val endTimestamp = Timer.getFPGATimestamp()
            Logger.recordOutput("$loggingName/GetLatestMeasurementMs", (endTimestamp - startTimestamp) * 1000.0)

            return PipelineVisionPacket(
                hasTargets,
                result.getBestTarget(),
                result.getTargets(),
                result.timestampSeconds - 0.03
            )
        }
}