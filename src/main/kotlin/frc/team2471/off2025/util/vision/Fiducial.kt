package frc.team2471.off2025.util.vision

import edu.wpi.first.math.geometry.Pose3d

// An ID of -1 indicates this is an unlabeled fiducial (e.g. retroreflective tape)
class Fiducial(val type: Type, val id: Int, val pose: Pose3d, val size: Double) {
    enum class Type(@JvmField val value: Int) {
        RETROREFLECTIVE(0),
        APRILTAG(1)
    }

    val x: Double
        get() = pose.x

    val y: Double
        get() = pose.y

    val z: Double
        get() = pose.z

    val xRot: Double
        get() = pose.rotation.x

    val yRot: Double
        get() = pose.rotation.y

    val zRot: Double
        get() = pose.rotation.z

    companion object {
        // Struct for serialization.
        val struct: FiducialStruct = FiducialStruct()
    }
}
