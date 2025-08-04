package frc.team2471.off2025.util.vision

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.util.Units

object Fiducials {
    private val aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).tags
    private val aprilTagSize = Units.inchesToMeters(6.5) // m

    val aprilTagFiducials: Array<Fiducial> = Array(aprilTagFieldLayout.size) {
        val tag = aprilTagFieldLayout[it]
        Fiducial(
            Fiducial.Type.APRILTAG,
            tag.ID,
            tag.pose,
            aprilTagSize
        )
    }

    fun determineClosestTagID(robotPose: Pose2d, isBlue: Boolean): Int {
        // Define tag indices
        val tagIndices = if (isBlue) intArrayOf(16, 17, 18, 19, 20, 21) else intArrayOf(5, 6, 7, 8, 9, 10)

        // Evaluate each canidate
        var closestTagIndex = 0 // Is this the right initial value?
        var closestTagDist = Double.Companion.POSITIVE_INFINITY
        for (tagIndex in tagIndices) {
            val tag: Pose2d = aprilTagFiducials[tagIndex].pose.toPose2d()
            val tagDist: Double = tag.translation.getDistance(robotPose.translation)

            if (tagDist < closestTagDist) {
                closestTagIndex = tagIndex
                closestTagDist = tagDist
            }
        }
        return closestTagIndex + 1
    }
}