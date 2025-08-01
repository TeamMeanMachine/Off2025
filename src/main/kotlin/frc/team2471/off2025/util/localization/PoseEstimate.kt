package frc.team2471.off2025.util.localization

import edu.wpi.first.math.geometry.Pose2d

/**
 * The PoseEstimate class represents an estimate of a robot's pose (position and orientation) at a
 * given point in time. It includes an identifier, the pose itself, and a flag indicating whether
 * vision data was used in the estimation.
 */
class PoseEstimate @JvmOverloads constructor(
    val id: Int = 0,
    val pose: Pose2d = Pose2d.kZero,
    val hasVision: Boolean = false
) {
    override fun toString(): String = String.format("PoseEstimate(%s, %s, %s)", this.id, this.pose, hasVision)

    companion object {
        // Struct for serialization.
        @JvmField
        val struct: PoseEstimateStruct = PoseEstimateStruct()
    }
}
