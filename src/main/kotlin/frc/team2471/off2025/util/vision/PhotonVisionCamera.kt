package frc.team2471.off2025.util.vision

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.numbers.N8
import edu.wpi.first.wpilibj.Timer
import org.ejml.simple.SimpleMatrix
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.photonvision.PhotonCamera
import org.photonvision.simulation.PhotonCameraSim
import org.photonvision.simulation.SimCameraProperties
import org.photonvision.targeting.PhotonPipelineResult
import java.util.*

class PhotonVisionCamera(
    cameraName: String,
    transform: Transform3d,
    pipelineConfigs: Array<PipelineConfig>
) : QuixVisionCamera {
    private val m_loggingName: String
    private val m_camera: PhotonCamera
    override val cameraSim: PhotonCameraSim
    override val transform: Transform3d
    private val m_pipelineConfigs: Array<PipelineConfig>

    private val m_inputs = PhotonCameraInputs()

    inner class PhotonCameraInputs : LoggableInputs {
        // TODO: Monitor performance and consider not logging the whole PhotonPipelineResult.
        var pipelineIndex: Int = 0
        var latestResult: PhotonPipelineResult = PhotonPipelineResult()
        var cameraMatrix: Optional<Matrix<N3, N3>> = Optional.empty<Matrix<N3, N3>>()
        var distCoeffs: Optional<Matrix<N8, N1>> = Optional.empty<Matrix<N8, N1>>()

        override fun toLog(table: LogTable) {
            table.put("PipelineIndex", pipelineIndex)
            table.put<PhotonPipelineResult?>("LatestResult", latestResult)
            table.put("CameraMatrixIsPresent", cameraMatrix.isPresent())
            if (cameraMatrix.isPresent()) {
                table.put("CameraMatrixData", cameraMatrix.get().getData())
            }
            table.put("DistCoeffsIsPresent", distCoeffs.isPresent())
            if (distCoeffs.isPresent()) {
                table.put("DistCoeffsData", distCoeffs.get().getData())
            }
        }

        override fun fromLog(table: LogTable) {
            pipelineIndex = table.get("PipelineIndex", pipelineIndex)
            latestResult = table.get("LatestResult", latestResult)
            cameraMatrix =
                if (table.get("CameraMatrixIsPresent", false))
                    Optional.of<Matrix<N3, N3>>(
                        Matrix<N3, N3>(
                            SimpleMatrix(3, 3, true, *table.get("CameraMatrixData", DoubleArray(9)))
                        )
                    )
                else
                    Optional.empty<Matrix<N3, N3>>()
            distCoeffs =
                if (table.get("DistCoeffsIsPresent", false))
                    Optional.of<Matrix<N8, N1>>(
                        Matrix<N8, N1>(
                            SimpleMatrix(8, 1, true, *table.get("DistCoeffsData", DoubleArray(8)))
                        )
                    )
                else
                    Optional.empty<Matrix<N8, N1>>()
        }
    }

    init {
        m_loggingName = "Inputs/PhotonVisionCamera [" + cameraName + "]"
        m_camera = PhotonCamera(cameraName)
        this.transform = transform
        m_pipelineConfigs = pipelineConfigs

        setPipelineIndex(0)

        // Setup sim
        val props = SimCameraProperties()
        // TODO: Sim more than one pipeline.
        props.setCalibration(
            pipelineConfigs[0].imageWidth,
            pipelineConfigs[0].imageHeight,
            pipelineConfigs[0].camIntrinsics,
            pipelineConfigs[0].distCoeffs
        )
        props.setCalibError(0.25, 0.08)
        props.fps = 20.0
        props.avgLatencyMs = 35.0
        props.latencyStdDevMs = 5.0
        this.cameraSim = PhotonCameraSim(m_camera, props)
        cameraSim.enableDrawWireframe(true)
        cameraSim.enableProcessedStream(true)
        cameraSim.enableRawStream(true)
    }

    override fun updateInputs() {
        m_inputs.pipelineIndex = m_camera.pipelineIndex
        // TODO: Handle all results, not just the latest.
        val latestResults = m_camera.allUnreadResults
        m_inputs.latestResult =
            if (latestResults.size > 0)
                latestResults.get(latestResults.size - 1)
            else
                PhotonPipelineResult()
        // Only update these once, since they shouldn't be changing.
        val cameraMatrix = m_camera.getCameraMatrix()
        if (m_inputs.cameraMatrix.isEmpty() && cameraMatrix.isPresent()) {
            m_inputs.cameraMatrix = cameraMatrix
        }
        val distCoeffs = cameraSim.getCamera().getDistCoeffs()
        if (m_inputs.distCoeffs.isEmpty() && distCoeffs.isPresent()) {
            m_inputs.distCoeffs = distCoeffs
        }
        Logger.processInputs(m_loggingName, m_inputs)
    }

    override fun setPipelineIndex(index: Int) {
        if (index > m_pipelineConfigs.size) {
            println("Invalid pipeline index: " + index)
            return
        }
        if (this.cameraSim != null) {
            //m_camera.setPipelineIndex(index)
        } else {
            //m_camera.setPipelineIndex(index)
        }
    }

    override val pipelineConfig: PipelineConfig
        get() = m_pipelineConfigs[m_inputs.pipelineIndex]

    override val cameraMatrix: Optional<Matrix<N3, N3>>
        get() = m_inputs.cameraMatrix

    override val distCoeffs: Optional<Matrix<N8, N1>>
        get() = m_inputs.distCoeffs

    override val fiducialType: Fiducial.Type
        get() = pipelineConfig.fiducialType

    override val latestMeasurement: PipelineVisionPacket
        get() {
            val startTimestamp = Timer.getFPGATimestamp()
            val result = m_inputs.latestResult
            val hasTargets = result.hasTargets()
            if (!hasTargets) {
                return PipelineVisionPacket(false, null, null, -1.0)
            }

            val endTimestamp = Timer.getFPGATimestamp()
            Logger.recordOutput(
                m_loggingName + "/GetLatestMeasurementMs", (endTimestamp - startTimestamp) * 1000.0
            )

            return PipelineVisionPacket(
                hasTargets,
                result.getBestTarget(),
                result.getTargets(),
                result.getTimestampSeconds() - 0.03
            )
        }
}
