package frc.team2471.off2025.util.quix

import edu.wpi.first.math.geometry.Pose2d
import frc.team2471.off2025.Constants
import org.apache.commons.math3.geometry.euclidean.twod.Line
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D

object AlignmentUtilities {
    fun determineClosestTagID(robotPose: Pose2d, isBlue: Boolean): Int {
        // Define tag indices
        val tagIndices = if (isBlue) intArrayOf(16, 17, 18, 19, 20, 21) else intArrayOf(5, 6, 7, 8, 9, 10)

        // Evaluate each canidate
        var closestTagIndex = 0 // Is this the right initial value?
        var closestTagDist = Double.Companion.POSITIVE_INFINITY
        for (tagIndex in tagIndices) {
            val tag: Pose2d = Fiducials.aprilTagFiducials[tagIndex].pose.toPose2d()
            val tagDist: Double = tag.translation.getDistance(robotPose.translation)

            if (tagDist < closestTagDist) {
                closestTagIndex = tagIndex
                closestTagDist = tagDist
            }
        }
        return closestTagIndex + 1
    }

    private fun createReefLines(centerPose: Pose2d): Array<Line> {
        val center = Vector2D(centerPose.getX(), centerPose.getY())

        val lines: Array<Line> = Array(3) {
            val angle = Math.toRadians((30 + 60 * it).toDouble())
            Line(center, angle, 1e-6)
        }

        return lines
    }

    val reefBlueLines: Array<Line> = createReefLines(Constants.Field.reefCenterBlue)
    val reefRedLines: Array<Line> = createReefLines(Constants.Field.reefCenterRed)

    fun rumbleCondition(robotPose: Pose2d, isBlue: Boolean): Boolean {
        val reefLines: Array<Line> = if (isBlue) reefBlueLines else reefRedLines

        val robotPoint = Vector2D(robotPose.x, robotPose.y)

        var minDistance = Double.Companion.POSITIVE_INFINITY
        for (line in reefLines) {
            minDistance = minDistance.coerceAtMost(line.distance(robotPoint))
        }

        return minDistance < 0.2
    }

//    fun determineTipPoseFromTag(tagPose: Pose2d, stackChoice: AlignmentState.ReefStackChoice): Pose2d {
//        val tagToTop: Transform2d? =
//            if (stackChoice === AlignmentState.ReefStackChoice.RIGHT)
//                Constants.Field.tagToRightReefTipTransform
//            else
//                Constants.Field.tagToLeftReefTipTransform
//        return tagPose.transformBy(tagToTop)
//    }

    fun isClearOfReef(robotPose: Pose2d, isBlue: Boolean): Boolean {
        val closestTagID = determineClosestTagID(robotPose, isBlue)
        val closestTagPose: Pose2d = Fiducials.aprilTagFiducials[closestTagID - 1].pose.toPose2d()
        val distToClosestReefWall: Double = robotPose.relativeTo(closestTagPose).translation.x
        return distToClosestReefWall > Constants.Field.robotReefWallPrescoreClearanceDistance
    }
}
