package frc.team2471.off2025.util.vision

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import frc.team2471.off2025.util.isReal
import org.photonvision.estimation.TargetModel
import org.photonvision.simulation.VisionSystemSim
import org.photonvision.simulation.VisionTargetSim

class QuixVisionSim(cameras: ArrayList<QuixVisionCamera>, aprilTags: Array<Fiducial>) {
    private val m_visionSim = VisionSystemSim("main")

    init {
        if (!isReal) {
            for (camera in cameras) {
                when (camera.pipelineConfig.fiducialType) {
                    Fiducial.Type.APRILTAG -> {
                        for (tag in aprilTags) {
                            m_visionSim.addVisionTargets(
                                "apriltag",
                                VisionTargetSim(tag.pose, TargetModel.kAprilTag36h11, tag.id)
                            )
                        }
                    }
                    else -> {}
                }
                m_visionSim.addCamera(camera.cameraSim, camera.transform)
                println("vision sim has ${m_visionSim.cameraSims.size} cameras")
            }
        }
    }

    fun resetSimPose(pose: Pose2d?) {
        m_visionSim.resetRobotPose(pose)
    }

    fun updatePose(pose: Pose2d?) {
        m_visionSim.update(pose)
    }

    val simField: Field2d?
        get() = m_visionSim.debugField
}
