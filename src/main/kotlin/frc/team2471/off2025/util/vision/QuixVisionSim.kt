package frc.team2471.off2025.util.vision

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import frc.team2471.off2025.util.isReal
import org.photonvision.estimation.TargetModel
import org.photonvision.simulation.VisionSystemSim
import org.photonvision.simulation.VisionTargetSim

object QuixVisionSim {
    private val m_visionSim = VisionSystemSim("main")
    val aprilTags: Array<Fiducial> = Fiducials.aprilTagFiducials

    init {
        if (!isReal) {
            aprilTags.forEach {
                m_visionSim.addVisionTargets(
                    "apriltag",
                    VisionTargetSim(it.pose, TargetModel.kAprilTag36h11, it.id)
                )
            }
        }
    }

    fun addCamera(camera: PhotonVisionCamera) {
        m_visionSim.addCamera(camera.cameraSim, camera.transform)
        println("vision sim has ${m_visionSim.cameraSims.size} cameras")
    }

    fun resetSimPose(pose: Pose2d?) {
        m_visionSim.resetRobotPose(pose)
    }

    fun updatePose(pose: Pose2d) {
        m_visionSim.update(pose)
    }

    val simField: Field2d?
        get() = m_visionSim.debugField
}
