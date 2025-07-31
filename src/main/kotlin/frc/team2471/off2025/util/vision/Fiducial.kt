package frc.team2471.off2025.util.vision

import edu.wpi.first.math.geometry.Pose3d

class Fiducial(val type: Type, // An ID of -1 indicates this is an unlabeled fiducial (e.g. retroreflective tape)
    private val m_id: Int, val pose: Pose3d, val size: Double
) {
    enum class Type(@JvmField val value: Int) {
        RETROREFLECTIVE(0),
        APRILTAG(1)
    }

    fun id(): Int {
        return m_id
    }

    val x: Double
        get() = pose.getX()

    val y: Double
        get() = pose.getY()

    val z: Double
        get() = pose.getZ()

    val xRot: Double
        get() = pose.getRotation().getX()

    val yRot: Double
        get() = pose.getRotation().getY()

    val zRot: Double
        get() = pose.getRotation().getZ()

    companion object {
        // Struct for serialization.
        val struct: FiducialStruct = FiducialStruct()
    }
}
